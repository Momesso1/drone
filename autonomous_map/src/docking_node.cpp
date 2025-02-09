#include <string>
#include <random>
#include <algorithm>
#include <geometry_msgs/msg/point.hpp>
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <map>
#include <stack>
#include <unordered_map>
#include <optional>
#include <iostream>
#include <climits>
#include <iomanip>
#include <thread>
#include <queue>
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "subdrone_interfaces/msg/passar_vertices.hpp"
#include "subdrone_interfaces/msg/passar_arestas.hpp"
#include "subdrone_interfaces/msg/passar_array_vertices.hpp"
#include "subdrone_interfaces/msg/passar_array_arestas.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <cstring>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std::chrono_literals;

class DockingNode : public rclcpp::Node {

private:
    struct Vertex {
        int key;
        double x, y, z;
    };

    struct VertexDijkstra {
        double x, y, z;
        double orientation_x, orientation_y, orientation_z;
        double orientation_w;
    };

    struct Destinos {
        double x, y, z;
        double orientation_x, orientation_y, orientation_z;
        double orientation_w;
    };

    struct Edge {
        int v1, v2;
    };

    //Publishers.
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_docking_navegable_vertices_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_docking_vertices_;

    //Subscriptions.
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_navegable_vertices;

    //Timers.
    rclcpp::TimerBase::SharedPtr timer_docking_navegable_vertices_;
    rclcpp::TimerBase::SharedPtr timer_docking_vertices_;
    rclcpp::TimerBase::SharedPtr parameterTimer;


    std::vector<Vertex> navegableDockingVertices_;
    std::vector<Vertex> sendPositions_;
    std::unordered_map<std::string, Vertex> navegableVertices_;


    int temp_ = 1;
    double x_atual_, y_atual_;
    double distanceToObstacle_, resolution_;
    double dockingPositionX_, dockingPositionY_, dockingPositionZ_; 
    double dockingOrientationX_, dockingOrientationY_, dockingOrientationZ_, dockingOrientationW_;

    double roundToMultiple(double value, double multiple) 
    {
        return std::round(value / multiple) * multiple;
    }
    
    std::string concatenar(double x, double y, double z) 
    {
        std::string t = std::to_string(std::abs(x) * 100) + std::to_string(std::abs(y) * 100) + std::to_string(std::abs(z) * 100);


        int sign_x = (x >= 0) ? 0 : 1;
        int sign_y = (y >= 0) ? 0 : 1;
        int sign_z = (z >= 0) ? 0 : 1;

        return t + std::to_string(sign_x) + std::to_string(sign_y) + std::to_string(sign_z);
    }

    /*
    
        PUBLISHERS.
    
    */

    void publish_docking_navegable_vertices()
    {
        sensor_msgs::msg::PointCloud2 cloud_msg1;
        cloud_msg1.header.stamp = this->get_clock()->now();
        cloud_msg1.header.frame_id = "map";

        // Configuração dos campos do PointCloud2
        cloud_msg1.height = 1;  // Ponto único em cada linha
        cloud_msg1.width = navegableDockingVertices_.size(); // Quantidade de vértices
        cloud_msg1.is_dense = true;
        cloud_msg1.is_bigendian = false;
        cloud_msg1.point_step = 3 * sizeof(float); // x, y, z
        cloud_msg1.row_step = cloud_msg1.point_step * cloud_msg1.width;

        // Adicionar campos de x, y, z
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg1);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(cloud_msg1.width);

        // Preencher os dados do PointCloud2
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg1, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg1, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg1, "z");


        for (const auto& vertex : navegableDockingVertices_) 
        {        
            *iter_x = vertex.x;  
            *iter_y = vertex.y;  // Access the y coordinate of the current Vertex
            *iter_z = vertex.z;  // Access the z coordinate of the current Vertex

            ++iter_x;
            ++iter_y;
            ++iter_z;   
        }

        publisher_docking_navegable_vertices_->publish(cloud_msg1);
    }

    void publish_docking_positions()
    {
        if (!navegableDockingVertices_.empty()) 
        {

        // Preparar mensagem de PoseArray
            geometry_msgs::msg::PoseArray pose_array_msg;
            pose_array_msg.header.stamp = this->get_clock()->now();
            pose_array_msg.header.frame_id = "map";

            // Ponto inicial (vertex)
            geometry_msgs::msg::Pose pose_start;
            pose_start.position.x = x_atual_;
            pose_start.position.y = y_atual_;
            pose_start.position.z = 0.1;

            // Ponto final (vertex1)
            const auto& vertex1 = navegableDockingVertices_.back();
            geometry_msgs::msg::Pose pose_end;
            pose_end.position.x = vertex1.x;
            pose_end.position.y = vertex1.y;
            pose_end.position.z = vertex1.z;

            // Calcular orientação (quaternion) do vetor entre vertex e vertex1
            double dx = vertex1.x - x_atual_;
            double dy = vertex1.y - y_atual_;
            double dz = vertex1.z - 0.1;

            double yaw = std::atan2(dy, dx);
            double pitch = std::atan2(dz, std::sqrt(dx * dx + dy * dy));
            double roll = 0.0; // Supondo que não há rotação ao longo do eixo do vetor

            // Converter (roll, pitch, yaw) para quaternion
            tf2::Quaternion quaternion;
            quaternion.setRPY(roll, pitch, yaw);

            // Atribuir orientação ao ponto inicial
            pose_start.orientation.x = quaternion.x();
            pose_start.orientation.y = quaternion.y();
            pose_start.orientation.z = quaternion.z();
            pose_start.orientation.w = quaternion.w();

            // Ponto final terá a mesma orientação
            pose_end.orientation = pose_start.orientation;

            // Adicionar poses ao array
            pose_array_msg.poses.push_back(pose_end);
            pose_array_msg.poses.push_back(pose_start);

            // Publicar a mensagem
            publisher_docking_vertices_->publish(pose_array_msg);
          
        }

    }

    /*

        CALLBACKS.
    
    */

   void navegable_vertices_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

        while (iter_x != iter_x.end()) {
            Vertex vertices;
            vertices.key = 0;
            vertices.x = *iter_x;
            vertices.y = *iter_y;
            vertices.z = *iter_z;

            double new_x = roundToMultiple(vertices.x, distanceToObstacle_ * resolution_);
            double new_y = roundToMultiple(vertices.y, distanceToObstacle_ * resolution_);
            double new_z = roundToMultiple(vertices.z, distanceToObstacle_ * resolution_);
            std::string index = concatenar(new_x, new_y, new_z);

            navegableVertices_[index] = vertices;

            ++iter_x;
            ++iter_y;
            ++iter_z;
        }
    }


    void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        navegableDockingVertices_.clear();
        x_atual_ = msg->pose.position.x;
        y_atual_ = msg->pose.position.y;

        double qx = msg->pose.orientation.x;
        double qy = msg->pose.orientation.y;
        double qz = msg->pose.orientation.z;    
        double qw = msg->pose.orientation.w;

        // Calcular o ângulo de yaw a partir do quaternion
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        int distancia = 1;
        while (distanceToObstacle_ * distancia <= 0.9)
        {
            double x_novo = roundToMultiple(x_atual_ + (distanceToObstacle_ * distancia) * std::cos(yaw), distanceToObstacle_ * resolution_);
            double y_novo = roundToMultiple(y_atual_ + (distanceToObstacle_ * distancia) * std::sin(yaw), distanceToObstacle_ * resolution_);

            std::string index = concatenar(x_novo, y_novo, 0.1);

            if(navegableVertices_.find(index) == navegableVertices_.end())
            {
                RCLCPP_WARN(this->get_logger(), "The robot cannot dock here.");
                navegableDockingVertices_.clear();
                break;
            }
            
            distancia++;

            Vertex vertex;
            vertex.key = distancia;
            vertex.x = x_novo;
            vertex.y = y_novo;
            vertex.z = 0.1;

            navegableDockingVertices_.push_back(vertex);
        }
        
    }

    void check_parameters()
    {
      
        auto new_distanceToObstacle = this->get_parameter("distanceToObstacle").get_parameter_value().get<double>();
        auto new_resolution = this->get_parameter("resolution").get_parameter_value().get<int>();
        
        if (new_distanceToObstacle != distanceToObstacle_) 
        {
            distanceToObstacle_ = new_distanceToObstacle;
            resolution_ = 1;
            std::cout << "\n" << std::endl;
            RCLCPP_INFO(this->get_logger(), "Updated DistanceToObstacle: %.2f", distanceToObstacle_);
            RCLCPP_INFO(this->get_logger(), "Resolution set to 1.");           
        }
        else if(new_resolution != temp_)
        {   
            temp_ = new_resolution;

            std::cout << "\n" << std::endl;

            if(new_resolution <= -1)
            {
                resolution_ =  std::abs(1.0  / new_resolution);
                RCLCPP_INFO(this->get_logger(), "Updated resolution: %ld", new_resolution);

            }
            else if(new_resolution == 0)
            {
                RCLCPP_WARN(this->get_logger(), "Resolution cannot be 0.");
            }
            else
            {
                resolution_ = new_resolution;
                RCLCPP_INFO(this->get_logger(), "Updated resolution: %0.f", resolution_);

            }
          
        }
       
    }
    
   

public:
    DockingNode()
     : Node("docking_node"), distanceToObstacle_(0.1), resolution_(1)
    {
        this->declare_parameter<double>("distanceToObstacle", 0.1);
        this->declare_parameter<int>("resolution", 1);

        distanceToObstacle_ = this->get_parameter("distanceToObstacle").get_parameter_value().get<double>();
        resolution_ = this->get_parameter("resolution").get_parameter_value().get<int>();

        RCLCPP_INFO(this->get_logger(), "Updated DistanceToObstacle: %f", distanceToObstacle_);
        RCLCPP_INFO(this->get_logger(), "Resolution is set to: %0.f", resolution_);



        parameterTimer = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&DockingNode::check_parameters, this));


        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, 
                        std::bind(&DockingNode::goal_pose_callback, this, std::placeholders::_1));

        subscription_navegable_vertices = this->create_subscription<sensor_msgs::msg::PointCloud2>("/navegasdadable_vertices", 10, 
                        std::bind(&DockingNode::navegable_vertices_callback, this, std::placeholders::_1));


        publisher_docking_navegable_vertices_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/docking_vertices", 10);
        timer_docking_navegable_vertices_ = this->create_wall_timer(5ms, std::bind(&DockingNode::publish_docking_navegable_vertices, this));

        publisher_docking_vertices_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/docking_positions", 10);
        timer_docking_vertices_ = this->create_wall_timer(5ms, std::bind(&DockingNode::publish_docking_positions, this));



    }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<DockingNode>());
    rclcpp::shutdown();
    return 0;
}