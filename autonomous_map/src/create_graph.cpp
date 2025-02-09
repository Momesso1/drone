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
#include "subdrone_interfaces/msg/passar_vertices.hpp"
#include "subdrone_interfaces/msg/passar_arestas.hpp"
#include "subdrone_interfaces/msg/passar_array_vertices.hpp"
#include "subdrone_interfaces/msg/passar_array_arestas.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <cstring>

using namespace std::chrono_literals;




class GraphPublisher : public rclcpp::Node {

private:
    struct Vertex 
    {
        std::string key;
        double x, y, z;
        std::unordered_map<std::string, std::string> linkedCloudMapPoints;
    };

    struct VertexPointCloud 
    {
        float x, y, z;
    };

    struct CloudMapPoint
    {
        std::string key;
        float x, y, z;
        bool verified;
        bool higher;
        std::unordered_map<std::string, std::string> linkedArbitraryVertices;
    };

    struct CloudMapHigher
    {
        std::string higher;
        float z;
    };

    struct Edge 
    {
        std::string v1, v2;
    };

        
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription1_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription2_;
    
    //timers
    rclcpp::TimerBase::SharedPtr timer_point_cloud_;
    rclcpp::TimerBase::SharedPtr timer_vertices_arbitrary;
    rclcpp::TimerBase::SharedPtr timer_vertices_arbitrary121_;
    rclcpp::TimerBase::SharedPtr timer_dynamic_map_;
    rclcpp::TimerBase::SharedPtr parameterTimer;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_fixed_vertices;
    rclcpp::TimerBase::SharedPtr timer1_;

    // Publishers
    rclcpp::Publisher<subdrone_interfaces::msg::PassarArrayVertices>::SharedPtr publisher_;
    rclcpp::Publisher<subdrone_interfaces::msg::PassarArrayVertices>::SharedPtr publisher_fixed_vertices;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_vertices_arbitrary;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher121_;
    size_t count_;

    double poseX_ = 0.0;
    double poseY_ = 0.0;
    double poseZ_ = 0.0;
    double fixedNavigableVertices_;
    double maxSecurityDistance_;
    double distanceToObstacle_;
    double maxHeightSecurityDistance_;
    bool fixedFrames_;

    std::unordered_map<std::string, CloudMapHigher> higherCloudMapPointXY;
    std::unordered_map<std::string, Vertex> publishedFixedVertices;
    std::unordered_map<std::string, Vertex> fixedVertices;
    std::unordered_map<std::string, Vertex> publishedVerticesArbitrary;
    std::unordered_map<std::string, Vertex> verticesArbitrary;
    std::unordered_map<std::string, CloudMapPoint> verticesCloudMap;
    std::unordered_map<std::string, CloudMapPoint> receivedCloudMap;

    float roundToDecimal(float value, int decimal_places) 
    {
        float factor = std::pow(10, decimal_places);
        return std::round(value * factor) / factor;
    }

    double roundToMultiple(double value, double multiple) 
    {
        return std::round(value / multiple) * multiple;
    }

    std::string concatenar(double x, double y, double z) 
    {
        std::string t = std::to_string(std::abs(x)) + std::to_string(std::abs(y)) + std::to_string(std::abs(z));


        int sign_x = (x >= 0) ? 0 : 1;
        int sign_y = (y >= 0) ? 0 : 1;
        int sign_z = (z >= 0) ? 0 : 1;

        return t + std::to_string(sign_x) + std::to_string(sign_y) + std::to_string(sign_z);
    }

    std::string concatenarXY(float x, float y) 
    {
        std::string t = std::to_string(std::abs(x)) + std::to_string(std::abs(y));

        int sign_x = (x >= 0) ? 0 : 1;
        int sign_y = (y >= 0) ? 0 : 1;

        return t + std::to_string(sign_x) + std::to_string(sign_y);
    }



    void createGraphFromPointCloud() 
    {
        auto start_time_ = std::chrono::high_resolution_clock::now();
          
        for(auto it = verticesCloudMap.begin(); it != verticesCloudMap.end(); it++)
        {
            if(it->second.verified == false)
            {
                it->second.verified = true;
                
                double toma = 0.0, maxToma = 0.0;
                int opa = 0, opa2 = 0;
                double new_x, new_y, new_z = 0.0;
                std::string index, index1, index2, index3, index4, index5, index6, index7, index8, index9;
                if(maxHeightSecurityDistance_ >= maxSecurityDistance_ + fixedNavigableVertices_)
                {
                    maxToma = maxHeightSecurityDistance_;
                }
                else
                {
                    maxToma = maxSecurityDistance_ + fixedNavigableVertices_;
                }

                while(toma <= maxToma)
                {
                    if(toma <= maxHeightSecurityDistance_ && it->second.higher == true)
                    {
                            
                        new_x = roundToMultiple(it->second.x, distanceToObstacle_);
                        new_y = roundToMultiple(it->second.y, distanceToObstacle_);
                        new_z = roundToMultiple(it->second.z + toma, distanceToObstacle_);
                        index5 = concatenar(new_x, new_y, new_z);

                        if (verticesCloudMap.find(index5) == verticesCloudMap.end())
                        {
                            verticesCloudMap[index5] = {index5, static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z), false, false, {}};
                        }
                            
                    }

                    if(toma <= maxSecurityDistance_)
                    {
                        

                        for(int eita = 0; eita <= opa * 2; eita++)
                        {         
                            std::string index10 = concatenar((it->second.x + toma) - (distanceToObstacle_ * eita), (it->second.y + toma), it->second.z);
                            std::string index11 = concatenar((it->second.x + toma), (it->second.y + toma) - (distanceToObstacle_ * eita), it->second.z);

                            std::string index12 = concatenar((it->second.x - toma), (it->second.y - toma) + (distanceToObstacle_ * eita), it->second.z);
                            std::string index13 = concatenar((it->second.x - toma) + (distanceToObstacle_ * eita), (it->second.y - toma), it->second.z);
                            
                            verticesArbitrary[index10] = {index10, (it->second.x + toma) - (distanceToObstacle_ * eita), (it->second.y + toma), it->second.z, {}};
                            verticesArbitrary[index10].linkedCloudMapPoints[it->second.key] = it->second.key;
                            it->second.linkedArbitraryVertices[index10] = index10;

                            verticesArbitrary[index11] = {index11, (it->second.x + toma), (it->second.y + toma) - (distanceToObstacle_ * eita), it->second.z, {}};
                            verticesArbitrary[index11].linkedCloudMapPoints[it->second.key] = it->second.key;
                            it->second.linkedArbitraryVertices[index11] = index11;

                            verticesArbitrary[index12] = {index12, (it->second.x - toma), (it->second.y - toma) + (distanceToObstacle_ * eita), it->second.z, {}};
                            verticesArbitrary[index12].linkedCloudMapPoints[it->second.key] = it->second.key;
                            it->second.linkedArbitraryVertices[index12] = index12;

                            verticesArbitrary[index13] = {index13, (it->second.x - toma) + (distanceToObstacle_ * eita), (it->second.y - toma), it->second.z, {}};
                            verticesArbitrary[index13].linkedCloudMapPoints[it->second.key] = it->second.key;
                            it->second.linkedArbitraryVertices[index13] = index13;
                        }

                        opa++;
                    }
                    else if(toma > maxSecurityDistance_ && fixedFrames_ == true)
                    {
                        


                        for(int eita2 = 0; eita2 <= (opa2 * 2) + (opa * 2); eita2++)
                        {         
                            std::string index10 = concatenar((it->second.x + toma) - (distanceToObstacle_ * eita2), (it->second.y + toma), it->second.z);
                            std::string index11 = concatenar((it->second.x + toma), (it->second.y + toma) - (distanceToObstacle_ * eita2), it->second.z);

                            std::string index12 = concatenar((it->second.x - toma), (it->second.y - toma) + (distanceToObstacle_ * eita2), it->second.z);
                            std::string index13 = concatenar((it->second.x - toma) + (distanceToObstacle_ * eita2), (it->second.y - toma), it->second.z);

                            fixedVertices[index10] = {index10, (it->second.x + toma) - (distanceToObstacle_ * eita2), (it->second.y + toma), it->second.z, {}};
                            fixedVertices[index11] = {index11, (it->second.x + toma), (it->second.y + toma) - (distanceToObstacle_ * eita2), it->second.z, {}};
                            
                            fixedVertices[index12] = {index12, (it->second.x - toma), (it->second.y - toma) + (distanceToObstacle_ * eita2), it->second.z, {}};           
                            fixedVertices[index13] = {index13, (it->second.x - toma) + (distanceToObstacle_ * eita2), (it->second.y - toma), it->second.z, {}};
                        }

                        opa2++;
                    }
                
    
                    toma += distanceToObstacle_;
                }
              
            }
            

            
        }

                    

        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end_time - start_time_;
        //RCLCPP_INFO(this->get_logger(), "Tempo para criar o grafo de obstaculos: %.6lf", duration.count());
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

   
    void publish_vertices() 
    {
        subdrone_interfaces::msg::PassarArrayVertices verticesMessage;

        
        for (const auto& [key, vertex] : verticesArbitrary) 
        {
            
            
                Vertex vertex1;
                subdrone_interfaces::msg::PassarVertices Vertex;
                Vertex.key = vertex.key;
                Vertex.x = vertex.x;
                Vertex.y = vertex.y;
                Vertex.z = vertex.z;

                vertex1.key = vertex.key;
                vertex1.x = vertex.x;
                vertex1.y = vertex.y;
                vertex1.z = vertex.z;
        

                verticesMessage.data.push_back(Vertex);
                publishedVerticesArbitrary[vertex.key] = (vertex1);
            
        
    
                
        }
        

        
       
        publisher_->publish(verticesMessage);
    }

    void publish_fixed_vertices() 
    {
        subdrone_interfaces::msg::PassarArrayVertices verticesMessage1;

        if(!fixedVertices.empty())
        {
            for (const auto& [key, vertex] : fixedVertices) 
            {
                // if(publishedFixedVertices.find(key) == publishedFixedVertices.end())
                // {
                //     Vertex Vertex1;
                //     subdrone_interfaces::msg::PassarVertices Vertex;
                //     Vertex.key = vertex.key;
                //     Vertex.x = vertex.x;
                //     Vertex.y = vertex.y;
                //     Vertex.z = vertex.z;
                    
                //     Vertex1.key = vertex.key;
                //     Vertex1.x = vertex.x;
                //     Vertex1.y = vertex.y;
                //     Vertex1.z = vertex.z;

                //     verticesMessage1.data.push_back(Vertex);
                //     publishedFixedVertices[vertex.key] = (Vertex1);
          
                // }

                Vertex Vertex1;
                    subdrone_interfaces::msg::PassarVertices Vertex;
                    Vertex.key = vertex.key;
                    Vertex.x = vertex.x;
                    Vertex.y = vertex.y;
                    Vertex.z = vertex.z;
                    
                    Vertex1.key = vertex.key;
                    Vertex1.x = vertex.x;
                    Vertex1.y = vertex.y;
                    Vertex1.z = vertex.z;

                    verticesMessage1.data.push_back(Vertex);
                    publishedFixedVertices[vertex.key] = (Vertex1);
                    
                   
            }
        }

       
        publisher_fixed_vertices->publish(verticesMessage1);
    }

    void publish_received_cloud_map()
    {
        sensor_msgs::msg::PointCloud2 cloud_msgs2;
        cloud_msgs2.header.stamp = this->get_clock()->now();
        cloud_msgs2.header.frame_id = "map";

        // Configuração dos campos do PointCloud2
        cloud_msgs2.height = 1;  // Ponto único em cada linha
        cloud_msgs2.width = receivedCloudMap.size(); // Quantidade de vértices
        cloud_msgs2.is_dense = true;
        cloud_msgs2.is_bigendian = false;
        cloud_msgs2.point_step = 3 * sizeof(float); // x, y, z
        cloud_msgs2.row_step = cloud_msgs2.point_step * cloud_msgs2.width;

        // Adicionar campos de x, y, z
        sensor_msgs::PointCloud2Modifier modifier(cloud_msgs2);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(cloud_msgs2.width);

        // Preencher os dados do PointCloud2
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msgs2, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msgs2, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msgs2, "z");
        for (const auto& [key, vertex] : receivedCloudMap) {
           
                *iter_x = vertex.x;
                *iter_y = vertex.y;
                *iter_z = vertex.z;

                ++iter_x;
                ++iter_y;
                ++iter_z;
   
            
        }


         publisher121_->publish(cloud_msgs2);
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
        std::string index1, index2;

        for (const auto& point : pcl_cloud.points) 
        {
        
            float x1 = roundToMultiple(point.x, distanceToObstacle_);
            float y1 = roundToMultiple(point.y, distanceToObstacle_);
            float z1 = roundToMultiple(point.z, distanceToObstacle_);


            index1 = concatenar(x1, y1, z1);
            index2 = concatenarXY(x1, y1);            

            if(verticesCloudMap.find(index1) == verticesCloudMap.end())
            {
                if(higherCloudMapPointXY.find(index2) == higherCloudMapPointXY.end())
                {
                    higherCloudMapPointXY[index2] = {index1, z1};
                    CloudMapPoint newCloudMap = {index1, x1, y1, z1, false, true, {}};
                    verticesCloudMap[index1] = (newCloudMap);
                }
                else if(z1 > higherCloudMapPointXY[index2].z)
                {
                    verticesCloudMap[higherCloudMapPointXY[index2].higher].higher = false;
                    higherCloudMapPointXY[index2] = {index1, z1};
                    CloudMapPoint newCloudMap = {index1, x1, y1, z1, false, true, {}};
                    verticesCloudMap[index1] = (newCloudMap);
                }
                
            }

            receivedCloudMap[index1] = {index1, x1, y1, z1, false, false, {}};
            verticesArbitrary[index1] = {index1, x1, y1 ,z1, {}};
                
                
        } 

        // std::cout << "verticesCloudMap: " << verticesCloudMap.size() << std::endl;
        // std::cout << "receivedCloudMap: " << receivedCloudMap.size() << std::endl;

        publish_received_cloud_map();
        createGraphFromPointCloud();
    }

    void check_parameters()
    {
        // Obtém os valores dos parâmetros
        auto new_distanceToObstacle = this->get_parameter("distanceToObstacle").as_double();
        auto new_maxSecurityDistance = this->get_parameter("maxSecurityDistance").as_double();
        auto new_maxHeightSecurityDistance = this->get_parameter("maxHeightSecurityDistance").as_double();
        auto new_fixedNavigableVertices = this->get_parameter("fixedNavigableVerticesDistance").as_double();
        auto new_fixedFrames = this->get_parameter("fixedNavigableVertices").as_bool();
      
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

        if(new_maxHeightSecurityDistance != maxHeightSecurityDistance_)
        {
            verticesCloudMap.clear();
            verticesArbitrary.clear();
            maxHeightSecurityDistance_ = new_maxHeightSecurityDistance;
            RCLCPP_INFO(this->get_logger(), "Updated maxHeightSecurityDistance: %f", maxHeightSecurityDistance_);
        }

        if(new_fixedNavigableVertices != fixedNavigableVertices_)
        {
            verticesCloudMap.clear();
            verticesArbitrary.clear();
            fixedNavigableVertices_ = new_fixedNavigableVertices;
            RCLCPP_INFO(this->get_logger(), "Updated fixedNavigableVerticesDistance: %f", fixedNavigableVertices_);
        }

        if(new_fixedFrames != fixedFrames_)
        {
            fixedVertices.clear();
            verticesCloudMap.clear();
            verticesArbitrary.clear();
            fixedFrames_ = new_fixedFrames;
            if(fixedFrames_ == true)
            {
                RCLCPP_INFO(this->get_logger(), "Fixed navigable vertices are activated");
            }
            else if(fixedFrames_ == false)
            {
                RCLCPP_INFO(this->get_logger(), "Fixed navigable vertices are not activated");
            }
        }


    }


public:
    GraphPublisher()
    : Node("graph_publisher"), count_(0)
    {   
       
        this->declare_parameter<double>("distanceToObstacle", 0.05);
        this->declare_parameter<double>("maxSecurityDistance", 0.25);
        this->declare_parameter<double>("maxHeightSecurityDistance", 0.0);
        this->declare_parameter<double>("fixedNavigableVerticesDistance", 0.20);
        this->declare_parameter<bool>("fixedNavigableVertices", true);

        // Obtém os valores iniciais dos parâmetros
        distanceToObstacle_ = this->get_parameter("distanceToObstacle").as_double();
        maxSecurityDistance_ = this->get_parameter("maxSecurityDistance").as_double();
        maxHeightSecurityDistance_ = this->get_parameter("maxHeightSecurityDistance").as_double();
        fixedNavigableVertices_ = this->get_parameter("fixedNavigableVerticesDistance").as_double();
        fixedFrames_ = this->get_parameter("fixedNavigableVertices").as_bool();
        // Verifica consistência inicial dos parâmetros
       

        RCLCPP_INFO(this->get_logger(), "Updated DistanceToObstacle: %2f", distanceToObstacle_);
        RCLCPP_INFO(this->get_logger(), "Updated maxSecurityDistance %2f", maxSecurityDistance_);
        RCLCPP_INFO(this->get_logger(), "Updated maxHeightSecurityDistance: %2f", maxHeightSecurityDistance_);
        RCLCPP_INFO(this->get_logger(), "Updated fixedNavigableVertice %2f", fixedNavigableVertices_);

        if(fixedFrames_ == false)
        {
            RCLCPP_INFO(this->get_logger(), "Fixed navigable vertices are NOT activated");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Fixed navigable vertices are activated");
        }

        // Timer para verificar alterações nos parâmetros
        parameterTimer = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&GraphPublisher::check_parameters, this));


        publisher_ = this->create_publisher<subdrone_interfaces::msg::PassarArrayVertices>("/vertices", 10);
        timer_ = this->create_wall_timer(15ms, std::bind(&GraphPublisher::publish_vertices, this));

        publisher_fixed_vertices = this->create_publisher<subdrone_interfaces::msg::PassarArrayVertices>("/fixed_vertices", 10);
        timer_fixed_vertices = this->create_wall_timer(15ms, std::bind(&GraphPublisher::publish_fixed_vertices, this));

        publisher_vertices_arbitrary = this->create_publisher<sensor_msgs::msg::PointCloud2>("/obstacles_vertices", 10);
        timer_vertices_arbitrary = this->create_wall_timer(1000ms, std::bind(&GraphPublisher::publish_obstacles_vertices, this));
    
        subscription1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_map", 10, std::bind(&GraphPublisher::callback_cloud_map, this, std::placeholders::_1));

        publisher121_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/received_cloud_map", 10);
        

         subscription2_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&GraphPublisher::odomCallback, this, std::placeholders::_1));


    }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
 
    rclcpp::spin(std::make_shared<GraphPublisher>());

    rclcpp::shutdown();

    return 0;
}