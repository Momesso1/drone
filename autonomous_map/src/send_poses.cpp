#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "yolo_msgs/msg/detection_array.hpp"
#include <map>
#include <stack>
#include <unordered_map>
#include <tf2/LinearMath/Quaternion.h>
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class SendPoses : public rclcpp::Node {
private:

    // Publishers.
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;

    // Subscriptions.
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr detection_sub_;

    // Timers.
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr parameterTimer;
    
    geometry_msgs::msg::Point drone_pos_;
    std::unordered_map<std::string, geometry_msgs::msg::Pose> detected_objects;
    std::vector<geometry_msgs::msg::Pose> destinations_;
    std::vector<geometry_msgs::msg::Pose> dockingDestinations_;

    bool docking_, person_detected = false;

    // Função para carregar as localizações do arquivo YAML
    void load_locations_from_yaml(const std::string &file_path)
    {
        try 
        {
            YAML::Node config = YAML::LoadFile(file_path);
            for (YAML::const_iterator it = config.begin(); it != config.end(); ++it) 
            {
                std::vector<double> coords = it->second.as<std::vector<double>>();
                if (coords.size() >= 3) 
                {
                    
                    geometry_msgs::msg::Pose pose;
                    pose.position.x = coords[0];
                    pose.position.y = coords[1];
                    pose.position.z = coords[2];
                    pose.orientation.x = 0.0;
                    pose.orientation.y = 0.0;
                    pose.orientation.z = 0.0;
                    pose.orientation.w = 1.0;
                    destinations_.push_back(pose);
                }
            }
        } 
        catch (const YAML::Exception &e) 
        {
        std::cerr << "Erro ao carregar o arquivo YAML: " << e.what() << std::endl;
        }
    }

    /*
        PUBLISHERS.
    */
    
    void publisher_poses()
    {   
        geometry_msgs::msg::PoseArray message;
        message.header.stamp = this->now();
        message.header.frame_id = "map";

        if(detected_objects.find("person") != detected_objects.end()) 
        {
            // Obtém a pose armazenada para "person"
            geometry_msgs::msg::Pose person_pose = detected_objects["person"];
            geometry_msgs::msg::Pose pose;
            pose.position.x = person_pose.position.x;
            pose.position.y = person_pose.position.y;
            pose.position.z = person_pose.position.z;
            // Se desejar, mantenha a orientação calculada; aqui apenas zeramos
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = 0.0;
            pose.orientation.w = 1.0; 
            message.poses.push_back(pose);
        }
        else if(docking_ == false) 
        {

            for (const auto &vertex : destinations_) 
            {
                geometry_msgs::msg::Pose pose;
                pose.position.x = vertex.position.x;
                pose.position.y = vertex.position.y;
                pose.position.z = vertex.position.z;
                pose.orientation.x = 0.0;
                pose.orientation.y = 0.0;
                pose.orientation.z = 0.0;
                pose.orientation.w = 1.0; 
                message.poses.push_back(pose);
            }

        }
        else if (docking_ == true) 
        {

            for (const auto &vertex : dockingDestinations_) 
            {
                geometry_msgs::msg::Pose pose;
                pose.position.x = vertex.position.x;
                pose.position.y = vertex.position.y;
                pose.position.z = vertex.position.z;
                pose.orientation.x = vertex.orientation.x;
                pose.orientation.y = vertex.orientation.y;
                pose.orientation.z = vertex.orientation.z;
                pose.orientation.w = vertex.orientation.w; 
                message.poses.push_back(pose);
            }

        }


        publisher_->publish(message);
    }

    /*
        CALLBACKS.
    */

    void docking_positions(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        dockingDestinations_.clear();

        for (const auto &pose : msg->poses) {
        geometry_msgs::msg::Pose vertex; // Correto: declaramos uma variável do tipo Pose
        vertex.position.x = pose.position.x;
        vertex.position.y = pose.position.y;
        vertex.position.z = pose.position.z;

        vertex.orientation.x = pose.orientation.x;
        vertex.orientation.y = pose.orientation.y;
        vertex.orientation.z = pose.orientation.z;
        vertex.orientation.w = pose.orientation.w;

        dockingDestinations_.push_back(vertex);
        }
    }

    void objects_positions(const yolo_msgs::msg::DetectionArray::SharedPtr msg) 
    {
        
        for (const auto &detection : msg->detections) 
        {
        
        auto bbox = detection.bbox3d;
        geometry_msgs::msg::Point object_position = bbox.center.position;
        
        // RCLCPP_INFO(this->get_logger(), "Class: %s, Score: %.2f", 
        //             detection.class_name.c_str(), detection.score);
        // RCLCPP_INFO(this->get_logger(), "BBox3D Center: [%.2f, %.2f, %.2f], Size: [%.2f, %.2f, %.2f]",
        //             object_position.x, object_position.y, object_position.z,
        //             bbox.size.x, bbox.size.y, bbox.size.z);
        
        
        double dx = drone_pos_.x - object_position.x;
        double dy = drone_pos_.y - object_position.y;
        double dz = drone_pos_.z - object_position.z;

        double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        
        geometry_msgs::msg::Point target_position;
        if(distance <= 1.0) 
        {
            // RCLCPP_INFO(this->get_logger(), "O drone já está a 1 metro ou menos do objeto.");
            target_position = drone_pos_;  
        } 
        else 
        {
            double factor = 1.0 / distance;
            target_position.x = object_position.x + dx * factor;
            target_position.y = object_position.y + dy * factor;
            target_position.z = object_position.z + dz * factor;
            // RCLCPP_INFO(this->get_logger(), "Posição alvo para o drone: [%.2f, %.2f, %.2f]", 
            //             target_position.x, target_position.y, target_position.z);
        }
        
        
        geometry_msgs::msg::Pose target_pose;
        target_pose.position = target_position;
        
        
        double delta_x = object_position.x - target_position.x;
        double delta_y = object_position.y - target_position.y;
        double yaw = std::atan2(delta_y, delta_x);
        
        
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
        target_pose.orientation.w = q.w();
        
        
        detected_objects[detection.class_name] = target_pose;
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) 
    {
        drone_pos_ = msg->pose.pose.position; // Atualiza a posição do drone
    }

    void check_parameters()
    {
        auto new_docking = this->get_parameter("docking").get_parameter_value().get<bool>();

        if(new_docking != docking_) 
        {
            docking_ = new_docking;
            if(docking_ == true) 
            {
                RCLCPP_INFO(this->get_logger(), "The robot will dock now.");
            } 
            else 
            {
                RCLCPP_INFO(this->get_logger(), "The robot will not dock now");
            }
        }
    }


public:
  SendPoses()
  : Node("send_poses")
  {
    this->declare_parameter<bool>("docking", false);
    docking_ = this->get_parameter("docking").get_parameter_value().get<bool>();


    

    parameterTimer = this->create_wall_timer(1s, std::bind(&SendPoses::check_parameters, this));

    publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/destinations", 10);
    timer_ = this->create_wall_timer(50ms, std::bind(&SendPoses::publisher_poses, this));

    subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/docking_positions", 10, std::bind(&SendPoses::docking_positions, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/rtabmap/odom", 10, std::bind(&SendPoses::odom_callback, this, std::placeholders::_1));

    detection_sub_ = this->create_subscription<yolo_msgs::msg::DetectionArray>(
      "/yolo/detections_3d", 10, std::bind(&SendPoses::objects_positions, this, std::placeholders::_1));

    const std::string yaml_file_path = "/home/momesso/autonomous/src/autonomous_map/config/locations.yaml";
    load_locations_from_yaml(yaml_file_path);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SendPoses>());
  rclcpp::shutdown();
  return 0;
}
