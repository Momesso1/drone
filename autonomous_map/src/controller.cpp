#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>
#include <vector>

class DijkstraController3D : public rclcpp::Node
{
public:
    DijkstraController3D()
        : Node("dijkstra_controller_3d"), current_vertex_index(0)
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/simple_drone/cmd_vel", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/rtabmap/odom", 10, std::bind(&DijkstraController3D::odom_callback, this, std::placeholders::_1));
        vertex_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/path", 10, std::bind(&DijkstraController3D::vertex_callback, this, std::placeholders::_1));
        destinos_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/destinations", 10, std::bind(&DijkstraController3D::destinos_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr vertex_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr destinos_sub_;

    geometry_msgs::msg::Pose current_pose_;
    std::vector<geometry_msgs::msg::Pose> route_;
    std::vector<geometry_msgs::msg::Pose> destinos_;
    size_t current_vertex_index = 0;
    size_t i_ = 0;

    void destinos_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        destinos_.clear();
        destinos_.insert(destinos_.end(), msg->poses.begin(), msg->poses.end());
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
    }

    void vertex_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
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

        auto goal = route_[current_vertex_index];
        double dx = goal.position.x - current_pose_.position.x;
        double dy = goal.position.y - current_pose_.position.y;
        double dz = goal.position.z - current_pose_.position.z;
        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        double angle_to_goal = std::atan2(dy, dx);
        double current_yaw = get_yaw_from_quaternion(current_pose_.orientation);
        double angle_diff = normalize_angle(angle_to_goal - current_yaw);

        double pitch_angle = std::atan2(dz, std::sqrt(dx * dx + dy * dy));

        geometry_msgs::msg::Twist velocity_msg;

        if (distance > 0.1) {
            velocity_msg.linear.x = 0.15;
            velocity_msg.angular.z = std::clamp(0.45 * angle_diff, -0.25, 0.25);
            velocity_msg.linear.z = std::clamp(0.05 * pitch_angle, -0.25, 0.25);
        } else {
            velocity_msg.linear.x = 0.0;
            velocity_msg.angular.z = 0.0;
            velocity_msg.linear.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Chegou no vértice %zu.", current_vertex_index);
            current_vertex_index++;
        }

        if (i_ < destinos_.size()) {
            double dx_to_destiny = destinos_[i_].position.x - current_pose_.position.x;
            double dy_to_destiny = destinos_[i_].position.y - current_pose_.position.y;
            double dz_to_destiny = destinos_[i_].position.z - current_pose_.position.z;
            double distance_to_destiny = std::sqrt(dx_to_destiny * dx_to_destiny + dy_to_destiny * dy_to_destiny + dz_to_destiny * dz_to_destiny);
            if (distance_to_destiny < 0.4) {
                velocity_msg.linear.x = 0.0;
                velocity_msg.angular.z = 0.0;
                velocity_msg.linear.z = 0.0;
                cmd_vel_pub_->publish(velocity_msg);
                RCLCPP_INFO(this->get_logger(), "Destino alcançado.");
                i_ = (i_ + 1) % destinos_.size();
            }
        }

        cmd_vel_pub_->publish(velocity_msg);
    }

    double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q)
    {
        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 m(tf_q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DijkstraController3D>());
    rclcpp::shutdown();
    return 0;
}
