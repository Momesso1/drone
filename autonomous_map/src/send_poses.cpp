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

using namespace std::chrono_literals;

class SendPoses : public rclcpp::Node {
private:
    struct Pose
    {
        double x;
        double y;
        double z;
        double orientation_x;
        double orientation_y;
        double orientation_z;
        double orientation_w;
    };

    //Publishers.
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;


    //Subscriptions.
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;

    //Timers.
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr parameterTimer;
    

    std::vector<Pose> destinations_;
    std::vector<Pose> dockingDestinations_;

    bool docking_;

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
                    destinations_.push_back({coords[0], coords[1], coords[2], 0.0, 0.0, 0.0, 1.0});
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

        if(docking_ == false)
        {
            for (const auto& vertex : destinations_)
            {
                geometry_msgs::msg::Pose pose;

                pose.position.x = vertex.x;
                pose.position.y = vertex.y;
                pose.position.z = vertex.z;

                pose.orientation.x = 0.0;
                pose.orientation.y = 0.0;
                pose.orientation.z = 0.0;
                pose.orientation.w = 1.0; 

                message.poses.push_back(pose);
            }
        }
        else if (docking_ == true)
        {
            for (const auto& vertex : dockingDestinations_)
            {
                geometry_msgs::msg::Pose pose;

                pose.position.x = vertex.x;
                pose.position.y = vertex.y;
                pose.position.z = vertex.z;

                pose.orientation.x = vertex.orientation_x;
                pose.orientation.y = vertex.orientation_y;
                pose.orientation.z = vertex.orientation_z;
                pose.orientation.w = vertex.orientation_w; 

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

        for (const auto &pose : msg->poses)
        {
            Pose vertex;
            vertex.x = pose.position.x;
            vertex.y = pose.position.y;
            vertex.z = pose.position.z;

            vertex.orientation_x = pose.orientation.x;
            vertex.orientation_y = pose.orientation.y;
            vertex.orientation_z = pose.orientation.z;
            vertex.orientation_w = pose.orientation.w;

            dockingDestinations_.push_back(vertex);
        }


       
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
            else if(docking_ == false)
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

        parameterTimer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&SendPoses::check_parameters, this));

        publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/destinations", 10);
        timer_ = this->create_wall_timer(50ms, std::bind(&SendPoses::publisher_poses, this));

        subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/docking_positions", 10, std::bind(&SendPoses::docking_positions, this, std::placeholders::_1));


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
