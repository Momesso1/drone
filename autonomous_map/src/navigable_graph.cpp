#include <string>
#include <random>
#include <algorithm>
#include <geometry_msgs/msg/point.hpp>
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
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
#include "subdrone_interfaces/msg/passar_array_vertices.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <cstring>
#include <utility> 


using namespace std::chrono_literals;
namespace std 
{
    template <>
    struct hash<std::tuple<float, float, float>> 
    {
        size_t operator()(const std::tuple<float, float, float>& t) const 
        {
            size_t h1 = hash<float>()(std::get<0>(t));
            size_t h2 = hash<float>()(std::get<1>(t));
            size_t h3 = hash<float>()(std::get<2>(t));
            
            return h1 ^ (h2 << 1) ^ (h3 << 2);
        }
    };
}

class NavigableGraph : public rclcpp::Node {

private:

    struct ObstaclesVertices 
    {
        float x, y, z;
        bool verified;
    };

    struct Vertex {
        float x, y, z;
    };


    //Publisher.
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_removed_navigable_vertices_;

    //Subscription.
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

    //Timer.
    rclcpp::TimerBase::SharedPtr timer_removed_navigable_vertices;
    rclcpp::TimerBase::SharedPtr parameterTimer;


    float resolution_; 
    float distanceToObstacle_;
    float x_min_, x_max_;
    float y_min_, y_max_;
    float z_min_, z_max_; 
    int decimals = 0;


    std::vector<Vertex> removedNavigableVertices;
    std::unordered_map<std::tuple<float, float, float>, ObstaclesVertices> obstaclesVertices_;


    inline float roundToMultiple(float value, float multiple, int decimals) {
        if (multiple == 0.0) return value; // Evita divisão por zero
        
        float result = std::round(value / multiple) * multiple;
        float factor = std::pow(10.0, decimals);
        result = std::round(result * factor) / factor;
        
        return result;
    }
    
    inline float roundToMultipleFromBase(float value, float base, float multiple, int decimals) {
        if (multiple == 0.0) return value; 
        
        float result = base + std::round((value - base) / multiple) * multiple;
        float factor = std::pow(10.0, decimals);
        result = std::round(result * factor) / factor;
        
        return result;
    }
    

    int countDecimals(float number) 
    {
      
        float fractional = std::fabs(number - std::floor(number));
        int decimals = 0;
        const float epsilon = 1e-9; 
    
  
        while (fractional > epsilon && decimals < 20) {
            fractional *= 10;
            fractional -= std::floor(fractional);
            decimals++;
        }
        return decimals;
    }


    void adjustGraph()
    {

        auto start_time_ = std::chrono::high_resolution_clock::now();

        for (const auto& vertex : obstaclesVertices_) 
        {   
            if(vertex.second.verified == false)
            {
                if(vertex.second.x >= x_min_ && vertex.second.x <= x_max_ && vertex.second.y >= y_min_ && vertex.second.y <= y_max_ && vertex.second.z >= z_min_ && vertex.second.z <= z_max_)
                {
                    removedNavigableVertices.push_back({vertex.second.x, vertex.second.y, vertex.second.z});
                }
                obstaclesVertices_[vertex.first].verified = true;
            }
            

           
        }

        
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> duration = end_time - start_time_;
      
    }

   
    /*

        PUBLISHER.

    */
    
 
    void publish_removed_navigable_vertices() 
    {
        sensor_msgs::msg::PointCloud2 removedVerticesMessage;
        removedVerticesMessage.header.stamp = this->now();
        removedVerticesMessage.header.frame_id = "map";
        removedVerticesMessage.height = 1;
        removedVerticesMessage.width = removedNavigableVertices.size();
        
        sensor_msgs::PointCloud2Modifier modifier(removedVerticesMessage);
        modifier.setPointCloud2FieldsByString(1, "xyz");  
        modifier.resize(removedNavigableVertices.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(removedVerticesMessage, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(removedVerticesMessage, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(removedVerticesMessage, "z");

        for (const auto &vertex : removedNavigableVertices) 
        {
            *iter_x = vertex.x;
            *iter_y = vertex.y;
            *iter_z = vertex.z;
            ++iter_x;
            ++iter_y;
            ++iter_z;
        }

        publisher_removed_navigable_vertices_->publish(removedVerticesMessage);
    }

    /*
    
        CALLBACK.

    */
        

    void callback_obstacles(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
     
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    
        size_t i = 0;
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) 
        {

            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;

            auto index = std::make_tuple(
                roundToMultiple(x, distanceToObstacle_, decimals),
                roundToMultiple(y, distanceToObstacle_, decimals),
                roundToMultipleFromBase(z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals)
            );

            ObstaclesVertices vertexObstacles;
            vertexObstacles.x = *iter_x;
            vertexObstacles.y = *iter_y;
            vertexObstacles.z = *iter_z;
            vertexObstacles.verified = false;
            
            if(obstaclesVertices_.find(index) == obstaclesVertices_.end())
            {
                obstaclesVertices_.emplace(index, vertexObstacles);
                i++;
            }
            
        }
    
        adjustGraph();
    }
    

    void check_parameters()
    {
       
        auto new_x_min = static_cast<float>(this->get_parameter("x_min").get_parameter_value().get<double>());
        auto new_x_max = static_cast<float>(this->get_parameter("x_max").get_parameter_value().get<double>());
        auto new_y_min = static_cast<float>(this->get_parameter("y_min").get_parameter_value().get<double>());
        auto new_y_max = static_cast<float>(this->get_parameter("y_max").get_parameter_value().get<double>());
        auto new_z_min = static_cast<float>(this->get_parameter("z_min").get_parameter_value().get<double>());
        auto new_z_max = static_cast<float>(this->get_parameter("z_max").get_parameter_value().get<double>());


    

        if (new_x_min != x_min_) 
        {
            std::cout << "\n" << std::endl;
            x_min_ = new_x_min;
            RCLCPP_INFO(this->get_logger(), "Updated x_min: %.2f", x_min_);
        }
        if (new_x_max != x_max_) 
        {
            std::cout << "\n" << std::endl;
            x_max_ = new_x_max;
            RCLCPP_INFO(this->get_logger(), "Updated x_max: %.2f", x_max_);
        }
        if (new_y_min != y_min_) 
        {
            std::cout << "\n" << std::endl;
            y_min_ = new_y_min;
            RCLCPP_INFO(this->get_logger(), "Updated y_min: %.2f", y_min_);
        }
        if (new_y_max != y_max_) 
        {
            std::cout << "\n" << std::endl;
            y_max_ = new_y_max;
            RCLCPP_INFO(this->get_logger(), "Updated y_max: %.2f", y_max_);
        }        
        if (new_z_min != z_min_) 
        {
            std::cout << "\n" << std::endl;
            z_min_ = new_z_min;
            RCLCPP_INFO(this->get_logger(), "Updated z_min: %.2f", z_min_);
        }
        if (new_z_max != z_max_) 
        {
            std::cout << "\n" << std::endl;
            z_max_ = new_z_max;
            RCLCPP_INFO(this->get_logger(), "Updated z_max: %.2f", z_max_);
        }
    }


public:
    NavigableGraph()
     : Node("navigable_graph")
    {
    
        this->declare_parameter<double>("x_min", -10.0);
        this->declare_parameter<double>("x_max", 10.0);
        this->declare_parameter<double>("y_min", -10.0);
        this->declare_parameter<double>("y_max", 10.0);
        this->declare_parameter<double>("z_min", 0.2);
        this->declare_parameter<double>("z_max", 0.2);

        // Initialize parameters as float usando static_cast
        x_min_ = static_cast<float>(this->get_parameter("x_min").get_parameter_value().get<double>());
        x_max_ = static_cast<float>(this->get_parameter("x_max").get_parameter_value().get<double>());
        y_min_ = static_cast<float>(this->get_parameter("y_min").get_parameter_value().get<double>());
        y_max_ = static_cast<float>(this->get_parameter("y_max").get_parameter_value().get<double>());
        z_min_ = static_cast<float>(this->get_parameter("z_min").get_parameter_value().get<double>());
        z_max_ = static_cast<float>(this->get_parameter("z_max").get_parameter_value().get<double>());

        RCLCPP_INFO(this->get_logger(), "Updated x_min: %f", x_min_);
        RCLCPP_INFO(this->get_logger(), "Updated x_max: %f", x_max_);
        RCLCPP_INFO(this->get_logger(), "Updated y_min: %f", y_min_);
        RCLCPP_INFO(this->get_logger(), "Updated y_max: %f", y_max_);
        RCLCPP_INFO(this->get_logger(), "Updated z_min: %f", z_min_);
        RCLCPP_INFO(this->get_logger(), "Updated z_max: %f", z_max_);

       
        parameterTimer = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&NavigableGraph::check_parameters, this));

        decimals = countDecimals(distanceToObstacle_);
 
       subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/obstacles_vertices", 10, std::bind(&NavigableGraph::callback_obstacles, this, std::placeholders::_1));

   
        publisher_removed_navigable_vertices_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/removed_navigable_vertices", 10);
        timer_removed_navigable_vertices = this->create_wall_timer(1ms, std::bind(&NavigableGraph::publish_removed_navigable_vertices, this));


    }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<NavigableGraph>());
    rclcpp::shutdown();
    return 0;
}

