#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <cmath>
#include <map>
#include <unordered_map>
#include <optional>
#include <iostream>
#include <climits>
#include <iomanip>
#include <thread>
#include <queue>
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <limits>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <cstring>

using namespace std::chrono_literals;

namespace std 
{
    template <>
    struct hash<std::tuple<float, float>> 
    {
        size_t operator()(const std::tuple<float, float>& t) const 
        {
            size_t h1 = hash<float>()(std::get<0>(t));
            size_t h2 = hash<float>()(std::get<1>(t));
            return h1 ^ (h2 << 1);  // Combine the hashes
        }
    };

    template <>
    struct hash<std::tuple<float, float, float>> 
    {
        size_t operator()(const std::tuple<float, float, float>& t) const 
        {
            size_t h1 = hash<float>()(std::get<0>(t));
            size_t h2 = hash<float>()(std::get<1>(t));
            size_t h3 = hash<float>()(std::get<2>(t));
            return h1 ^ (h2 << 1) ^ (h3 << 2);  // Combine the hashes
        }
    };
}


class GraphPublisher : public rclcpp::Node {

private:
    struct TupleHash 
    {
        size_t operator()(const std::tuple<float, float, float>& t) const {
            return std::hash<float>()(std::get<0>(t)) ^ 
                (std::hash<float>()(std::get<1>(t)) << 1) ^ 
                (std::hash<float>()(std::get<2>(t)) << 2);
        }
    };

    struct CloudMapPoint
    {
        float x, y, z;
        bool verified;
        bool artificial;
    };

    struct Edge 
    {
        std::string v1, v2;
    };
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription1_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription2_;

     // Timers
    rclcpp::TimerBase::SharedPtr timer_point_cloud_;
    rclcpp::TimerBase::SharedPtr timer_vertices_arbitrary;
    rclcpp::TimerBase::SharedPtr timer_vertices_arbitrary121_;
    rclcpp::TimerBase::SharedPtr timer_dynamic_map_;
    rclcpp::TimerBase::SharedPtr parameterTimer;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_fixed_vertices;
    rclcpp::TimerBase::SharedPtr timer1_;

    // Publishers
 
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_vertices_arbitrary;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher121_;
    size_t count_;

    float poseX_ = 0.0;
    float poseY_ = 0.0;
    float poseZ_ = 0.0;
    float fixedNavigableVertices_;
    float maxSecurityDistance_;
    float distanceToObstacle_;
    float maxSecurityHeightDistance_;
    bool fixedFrames_;
    int decimals = 0;

    std::unordered_map<std::tuple<float, float, float>, geometry_msgs::msg::Point> verticesArbitrary;
    std::unordered_map<std::tuple<float, float, float>, CloudMapPoint> verticesCloudMap;

    float roundToDecimal(float value, int decimal_places) 
    {
        float factor = std::pow(10, decimal_places);
        return std::round(value * factor) / factor;
    }

    inline float roundToMultiple(float value, float multiple, int decimals) {
        if (multiple == 0.0) return value;
        
        float result = std::round(value / multiple) * multiple;
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

    const size_t HASH_TABLE_SIZE = 10000003; 

    
    bool saveIndicesToFile(const std::unordered_map<std::tuple<float, float, float>, geometry_msgs::msg::Point>& verticesArbitrary, const std::string& filename) 
    {
        std::vector<bool> hashTable(HASH_TABLE_SIZE, false);
        
        for (const auto& pair : verticesArbitrary) {
            const auto& tuple = pair.first;
            size_t hash = TupleHash()(tuple) % HASH_TABLE_SIZE;
            hashTable[hash] = true;
        }
        
        std::ofstream file(filename, std::ios::binary);
        if (!file.is_open()) return false;
        
        std::vector<uint8_t> bitArray(HASH_TABLE_SIZE / 8 + 1, 0);
        for (size_t i = 0; i < HASH_TABLE_SIZE; i++) {
            if (hashTable[i]) {
                bitArray[i / 8] |= (1 << (i % 8));
            }
        }
        
        file.write(reinterpret_cast<const char*>(&HASH_TABLE_SIZE), sizeof(size_t));
        
        file.write(reinterpret_cast<const char*>(bitArray.data()), bitArray.size());
        
        file.close();
        return true;
    }
    
  


    void createGraphFromPointCloud() 
    {
        auto start_time_ = std::chrono::high_resolution_clock::now();
          
        for(auto it = verticesCloudMap.begin(); it != verticesCloudMap.end(); it++)
        {
            if(it->second.verified == false)
            {
                it->second.verified = true;
                
                float toma = 0.0, maxToma = 0.0;
                int opa = 0;
                float new_x, new_y, new_z = 0.0;
        
                if(maxSecurityHeightDistance_ >= maxSecurityDistance_)
                {
                    maxToma = maxSecurityHeightDistance_;
                }
                else
                {
                    maxToma = maxSecurityDistance_;
                }
              

                while(toma <= maxToma)
                {
                    if(toma <= maxSecurityHeightDistance_ && it->second.artificial == false)
                    {
                            
                        new_x = roundToMultiple(it->second.x, distanceToObstacle_, decimals);
                        new_y = roundToMultiple(it->second.y, distanceToObstacle_, decimals);
                        new_z = roundToMultiple(it->second.z + toma, distanceToObstacle_, decimals);
                        auto index5 = std::make_tuple(new_x, new_y, new_z);

                        if (verticesCloudMap.find(index5) == verticesCloudMap.end() )
                        {
                            verticesCloudMap[index5] = {static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z), false, true};
                        }
                        
                            
                    }

                    if(toma <= maxSecurityHeightDistance_ && it->second.artificial == false)
                    {
                        new_x = roundToMultiple(it->second.x, distanceToObstacle_, decimals);
                        new_y = roundToMultiple(it->second.y, distanceToObstacle_, decimals);
                        new_z = roundToMultiple(it->second.z - toma, distanceToObstacle_, decimals);
                        auto index6 = std::make_tuple(new_x, new_y, new_z);

                        if (verticesCloudMap.find(index6) == verticesCloudMap.end())
                        {
                            verticesCloudMap[index6] = {static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z), false, true};
                        }
                       
                    }

                    if(toma <= maxSecurityDistance_)
                    {
                        

                        for(int eita = 0; eita <= opa * 2; eita++)
                        {   
                            auto index10 = std::make_tuple((it->second.x + toma) - (distanceToObstacle_ * eita), (it->second.y + toma), it->second.z);
                            auto index11 = std::make_tuple((it->second.x + toma), (it->second.y + toma) - (distanceToObstacle_ * eita), it->second.z);

                            auto index12 = std::make_tuple((it->second.x - toma), (it->second.y - toma) + (distanceToObstacle_ * eita), it->second.z);
                            auto index13 = std::make_tuple((it->second.x - toma) + (distanceToObstacle_ * eita), (it->second.y - toma), it->second.z);

                            
                            geometry_msgs::msg::Point point10;
                            point10.x = (it->second.x + toma) - (distanceToObstacle_ * eita);
                            point10.y = (it->second.y + toma);
                            point10.z = it->second.z;

                            geometry_msgs::msg::Point point11;
                            point11.x = (it->second.x + toma);
                            point11.y = (it->second.y + toma) - (distanceToObstacle_ * eita);
                            point11.z = it->second.z;

                            geometry_msgs::msg::Point point12;
                            point12.x = (it->second.x - toma);
                            point12.y = (it->second.y - toma) + (distanceToObstacle_ * eita);
                            point12.z = it->second.z;

                            geometry_msgs::msg::Point point13;
                            point13.x = (it->second.x - toma) + (distanceToObstacle_ * eita);
                            point13.y = (it->second.y - toma);
                            point13.z = it->second.z;

                            
                            verticesArbitrary[index10] = (point10);

                            verticesArbitrary[index11] = (point11);

                            verticesArbitrary[index12] = (point12);

                            verticesArbitrary[index13] = (point13);
                        
                            
                        }

                        opa++;
                    }
                   
                
    
                    toma += distanceToObstacle_;
                }
              
            }
            

            
        }

                    

        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> duration = end_time - start_time_;
        RCLCPP_INFO(this->get_logger(), "Tempo para criar o grafo de obstaculos: %.6lf", duration.count());
    }

    /*
    
        PUBLISHERS.

    */

    void publish_obstacles_vertices()
    {
        
        sensor_msgs::msg::PointCloud2 cloud_msg1;
        cloud_msg1.header.stamp = this->get_clock()->now();
        cloud_msg1.header.frame_id = "map";

        // Configuração dos campos do PointCloud2
        cloud_msg1.height = 1;  // Ponto único em cada linha
        cloud_msg1.width = verticesArbitrary.size(); // Quantidade de vértices
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
        for (const auto& [key, vertex] : verticesArbitrary) {
           
                *iter_x = vertex.x;
                *iter_y = vertex.y;
                *iter_z = vertex.z;

                ++iter_x;
                ++iter_y;
                ++iter_z;

        }

         publisher_vertices_arbitrary->publish(cloud_msg1);
    
    }

   



    /*
    
        CALLBACKS.

    */


    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) 
    {
        poseX_ = msg->pose.pose.position.x;
        poseY_ = msg->pose.pose.position.y;
        poseZ_ = msg->pose.pose.position.z;
    }

   void callback_cloud_map(const sensor_msgs::msg::PointCloud2::SharedPtr msg)  
    {
       
        // Converter PointCloud2 para pcl::PointCloud<pcl::PointXYZ>
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);
       
        for (const auto& point : pcl_cloud.points) 
        {
        
            float x1 = roundToMultiple(static_cast<float>(point.x), distanceToObstacle_, decimals);
            float y1 = roundToMultiple(static_cast<float>(point.y), distanceToObstacle_, decimals);
            float z1 = roundToMultiple(static_cast<float>(point.z), distanceToObstacle_, decimals);

            auto index1 = std::make_tuple(x1, y1, z1);

            geometry_msgs::msg::Point point1;
            point1.x = x1;
            point1.x = y1;
            point1.z = z1;

            if(verticesCloudMap.find(index1) == verticesCloudMap.end())
            {
                verticesCloudMap[index1] = {x1, y1, z1, false, false};
                
                verticesArbitrary[index1] = (point1);
            }
            
            if(verticesCloudMap[index1].artificial == true)
            {
                
                verticesCloudMap[index1] = {x1, y1, z1, false, false};
                verticesArbitrary[index1] = (point1);
            }
            
            
        }

        
        createGraphFromPointCloud();

        if(!verticesArbitrary.empty())
        {
            saveIndicesToFile(verticesArbitrary, "/home/momesso/autonomous/src/map/config/obstacles.bin");
        }
    }

    void check_parameters()
    {
        // Obtém os valores dos parâmetros
        auto new_distanceToObstacle = static_cast<float>(this->get_parameter("distanceToObstacle").get_parameter_value().get<double>());
        auto new_maxSecurityDistance = static_cast<float>(this->get_parameter("maxSecurityDistance").get_parameter_value().get<double>());
        auto new_maxSecurityHeightDistance = static_cast<float>(this->get_parameter("maxSecurityHeightDistance").get_parameter_value().get<double>());
        
        if (new_distanceToObstacle != distanceToObstacle_) 
        {
            verticesCloudMap.clear();
            verticesArbitrary.clear();
            distanceToObstacle_ = new_distanceToObstacle;
            RCLCPP_INFO(this->get_logger(), "Updated distanceToObstacle: %f", distanceToObstacle_);
        }

        if(new_maxSecurityDistance != maxSecurityDistance_)
        {
            verticesCloudMap.clear();
            verticesArbitrary.clear();
            maxSecurityDistance_ = new_maxSecurityDistance;
            RCLCPP_INFO(this->get_logger(), "Updated maxSecurityDistance: %f", maxSecurityDistance_);
        }

        if(new_maxSecurityHeightDistance != maxSecurityHeightDistance_)
        {
            verticesCloudMap.clear();
            verticesArbitrary.clear();
            maxSecurityHeightDistance_ = new_maxSecurityHeightDistance;
            RCLCPP_INFO(this->get_logger(), "Updated maxSecurityHeightDistance: %f", maxSecurityHeightDistance_);
        }
  

    }


public:
    GraphPublisher()
    : Node("graph_publisher"), count_(0)
    {   
       
        this->declare_parameter<double>("distanceToObstacle", 0.05);
        this->declare_parameter<double>("maxSecurityDistance", 0.25);
        this->declare_parameter<double>("maxSecurityHeightDistance", 0.0);


        distanceToObstacle_ = static_cast<float>(this->get_parameter("distanceToObstacle").get_parameter_value().get<double>());
        maxSecurityDistance_ = static_cast<float>(this->get_parameter("maxSecurityDistance").get_parameter_value().get<double>());
        maxSecurityHeightDistance_ = static_cast<float>(this->get_parameter("maxSecurityHeightDistance").get_parameter_value().get<double>());
       

        RCLCPP_INFO(this->get_logger(), "Updated DistanceToObstacle: %2f", distanceToObstacle_);
        RCLCPP_INFO(this->get_logger(), "Updated maxSecurityDistance %2f", maxSecurityDistance_);
        RCLCPP_INFO(this->get_logger(), "Updated maxSecurityHeightDistance: %2f", maxSecurityHeightDistance_);

       

      
        // Timer para verificar alterações nos parâmetros
        parameterTimer = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&GraphPublisher::check_parameters, this));

        decimals = countDecimals(distanceToObstacle_);




        publisher_vertices_arbitrary = this->create_publisher<sensor_msgs::msg::PointCloud2>("/obstacles_vertices", 10);
        timer_vertices_arbitrary = this->create_wall_timer(1000ms, std::bind(&GraphPublisher::publish_obstacles_vertices, this));
    
        subscription1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/rtabmap/cloud_map", 10, std::bind(&GraphPublisher::callback_cloud_map, this, std::placeholders::_1));

        publisher121_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/received_cloud_map", 10);
        

         subscription2_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/rtabmap/odom", 10, std::bind(&GraphPublisher::odomCallback, this, std::placeholders::_1));


    }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
 
    rclcpp::spin(std::make_shared<GraphPublisher>());

    rclcpp::shutdown();

    return 0;
}