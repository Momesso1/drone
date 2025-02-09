#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>
#include <vector>

class DijkstraController : public rclcpp::Node
{
public:
    DijkstraController()
        : Node("dijkstra_controller"), current_vertex_index(0)
    {
        // Publicador de comandos de velocidade
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Assinante do tópico de Odometria
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&DijkstraController::odom_callback, this, std::placeholders::_1));

        // Assinante do tópico de Vértices
        vertex_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/path", 10, std::bind(&DijkstraController::vertex_callback, this, std::placeholders::_1));

        // Assinante do tópico de Destinos
        destinos_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/destinations", 10, std::bind(&DijkstraController::destinos_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr vertex_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr destinos_sub_;

    geometry_msgs::msg::Pose current_pose_;
    std::vector<geometry_msgs::msg::Pose> route_;
    size_t current_vertex_index = 0;
   

    // Vetor para armazenar os destinos
    std::vector<geometry_msgs::msg::Pose> destinos_;

    size_t i_ = 0;
    size_t destinos_size_ = destinos_.size();

    void destinos_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        // Limpar os destinos anteriores (opcional)
        destinos_.clear();

        // Iterar sobre os destinos recebidos e armazená-los no vetor destinos_
        for (const auto& destino : msg->poses) {
            //RCLCPP_INFO(this->get_logger(), "Destino recebido - x: %.2f, y: %.2f, z: %.2f", 
             //           destino.position.x, destino.position.y, destino.position.z);
            
            // Adicionar o destino ao vetor
            destinos_.push_back(destino);
        }

        
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
    }

    void vertex_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        // Carregar a rota com as poses do PoseArray
        route_ = msg->poses;
        
        if (!route_.empty()) {
            RCLCPP_INFO(this->get_logger(), "Rota recebida com %zu vértices.", route_.size());
            move_to_goal();
        } else {
            RCLCPP_WARN(this->get_logger(), "Rota vazia recebida.");
        }
    }

    double normalize_angle(double angle)
    {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }

    void move_to_goal()
    {


        if (route_.empty() || current_vertex_index >= route_.size()) {
            RCLCPP_INFO(this->get_logger(), "Nenhum vértice ou rota concluída.");
            return;
        }


        for(size_t i = 0; i < route_.size(); i++)
        {
            
        // Pega o vértice atual
            auto goal = route_[current_vertex_index];
            
            // Calcular a distância e o ângulo para o próximo vértice
            double dx = goal.position.x - current_pose_.position.x;
            double dy = goal.position.y - current_pose_.position.y;
            double distance = std::sqrt(dx * dx + dy * dy);

            double angle_to_goal = std::atan2(dy, dx);
            double current_yaw = get_yaw_from_quaternion(current_pose_.orientation.x, current_pose_.orientation.y, current_pose_.orientation.z, current_pose_.orientation.w);
            double angle_diff = normalize_angle(angle_to_goal - current_yaw);

            // Definir a velocidade linear e angular
            geometry_msgs::msg::Twist velocity_msg;

            if (distance > 0.1) {
                velocity_msg.linear.x = 0.1;  // Velocidade linear

                RCLCPP_INFO(this->get_logger(), "Velocidade: %.2lf", velocity_msg.linear.x);
                // Limitar a velocidade angular para evitar giros rápidos
                const double max_angular_velocity = 2.0;  // Limite da velocidade angular
                velocity_msg.angular.z = std::max(std::min(0.45 * angle_diff, max_angular_velocity), -max_angular_velocity);  // Controle de orientação
            } 
            else {
                velocity_msg.linear.x = 0.05;
                velocity_msg.angular.z = 0.05;
                RCLCPP_INFO(this->get_logger(), "Chegou no vértice %zu.", current_vertex_index);
                current_vertex_index++;  // Avança para o próximo vértice
            }

            if (i_ < destinos_.size()) {
                double dx_to_destiny = destinos_[i_].position.x - current_pose_.position.x;
                double dy_to_destiny = destinos_[i_].position.y - current_pose_.position.y;
                double distance_to_destiny = std::sqrt(dx_to_destiny * dx_to_destiny + dy_to_destiny * dy_to_destiny);

                /*
                Distância para o destino alcançado. Se mudar aqui, tem que mudar no nó a_estrela também.
                */
                const double destination_threshold = 0.4;  
                if (distance_to_destiny < destination_threshold)
                {
                    velocity_msg.linear.x = 0.0;
                    velocity_msg.angular.z = 0.0;
                    cmd_vel_pub_->publish(velocity_msg);
                    RCLCPP_INFO(this->get_logger(), "Destino alcançado.");

                    

                    i_++;
                    if(i_ == destinos_.size()) {
                        i_ = 0;
                    }

            
                
                }
            }

            RCLCPP_INFO(this->get_logger(), "Mensagem publicada.");
            
            cmd_vel_pub_->publish(velocity_msg);
            
        }

    }

    double get_yaw_from_quaternion(float x, float y, float z, float w)
    {
        tf2::Quaternion q( x, y, z, w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);


    
        return yaw;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DijkstraController>());
    rclcpp::shutdown();
    return 0;
}

