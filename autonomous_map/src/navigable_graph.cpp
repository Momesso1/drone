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
    struct hash<std::tuple<double, double, double>> 
    {
        size_t operator()(const std::tuple<double, double, double>& t) const 
        {
            size_t h1 = hash<double>()(std::get<0>(t));
            size_t h2 = hash<double>()(std::get<1>(t));
            size_t h3 = hash<double>()(std::get<2>(t));
            
            return h1 ^ (h2 << 1) ^ (h3 << 2);
        }
    };
}

class NavigableGraph : public rclcpp::Node {

private:
    struct Vertex {
        int key;
        double x, y, z;
        bool up;
        bool down;
    };

    struct ObstaclesVertices 
    {
        int key;
        double x, y, z;
        bool verified;
        bool up;
        bool down;
    };

    struct StringVertex {
        std::tuple<double, double, double> key;
        double x, y, z;
        bool up;
        bool down;
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

    struct StringEdge {
        std::string v1, v2;
    };

    struct pair_hash 
    {
        template <class T1, class T2>
        std::size_t operator()(const std::pair<T1, T2>& p) const {
            return std::hash<T1>()(p.first) ^ (std::hash<T2>()(p.second) << 1);
        }
    };

    //Publishers.
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_path_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_visualize_navigable_graph;
    rclcpp::Publisher<subdrone_interfaces::msg::PassarArrayVertices>::SharedPtr publisher_navigable_vertices_;
    rclcpp::Publisher<subdrone_interfaces::msg::PassarArrayVertices>::SharedPtr publisher_removed_navigable_vertices_;

    //Subscriptions.
    rclcpp::Subscription<subdrone_interfaces::msg::PassarArrayVertices>::SharedPtr subscription_;
    rclcpp::Subscription<subdrone_interfaces::msg::PassarArrayVertices>::SharedPtr subscription_fixed_vertices;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription3_;

    //Timers.
    rclcpp::TimerBase::SharedPtr timer_visualize_navigable_graph;
    rclcpp::TimerBase::SharedPtr timer_navigable_vertices;
    rclcpp::TimerBase::SharedPtr timer_removed_navigable_vertices;
    rclcpp::TimerBase::SharedPtr timer_navigable_edges;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_path_;
    rclcpp::TimerBase::SharedPtr parameterTimer;


    int i_ = 0, temp_ = 1, enviados = 0, vertices_enviados = 0, contador = 0;
    double resolution_; 
    double pose_x_ = 0, pose_y_ = 0, pose_z_ = 0;
    double distanceToObstacle_;
    double x_min_, x_max_;
    double y_min_, y_max_;
    double z_min_, z_max_; 
    int decimals = 0;

    std::vector<Edge> edges_;
    std::string deleteKey;
    std::vector<VertexDijkstra> verticesDestino_;
    std::vector<VertexDijkstra> verticesDijkstra;
    std::vector<Edge> shortestPathEdges;
    std::vector<Edge> navigableEdges_;
    std::vector<StringEdge> stringNavigableEdges_;  
    std::vector<Vertex> navigableVertices_;
    std::unordered_map<int, double> g_score, f_score;
    std::unordered_map<int, int> came_from;
    std::unordered_map<std::pair<int, int>, double, pair_hash> distances;
    std::unordered_map<int, std::vector<int>> adjacency_list;
    std::unordered_map<std::tuple<double, double, double>, StringVertex> removedNavigableVertices;
    std::unordered_map<std::tuple<double, double, double>, ObstaclesVertices> fixedNavigableVerticesMap;
    std::unordered_map<std::tuple<double, double, double>, ObstaclesVertices> obstaclesVertices_;
    std::unordered_map<std::tuple<double, double, double>, Vertex> navigableVerticesMap;
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> open_set;
   

    inline double roundToMultiple(double value, double multiple, int decimals) {
        if (multiple == 0.0) return value; // Evita divisão por zero
        
        double result = std::round(value / multiple) * multiple;
        double factor = std::pow(10.0, decimals);
        result = std::round(result * factor) / factor;
        
        return result;
    }
    
    inline double roundToMultipleFromBase(double value, double base, double multiple, int decimals) {
        if (multiple == 0.0) return value; // Evita divisão por zero
        
        double result = base + std::round((value - base) / multiple) * multiple;
        double factor = std::pow(10.0, decimals);
        result = std::round(result * factor) / factor;
        
        return result;
    }

    int countDecimals(double number) 
    {
        // Separa a parte fracionária (trabalha com o valor absoluto)
        double fractional = std::fabs(number - std::floor(number));
        int decimals = 0;
        const double epsilon = 1e-9; // tolerância para determinar quando a parte fracionária é zero
    
        // Enquanto houver parte fracionária significativa e um limite para evitar loops infinitos
        while (fractional > epsilon && decimals < 20) {
            fractional *= 10;
            fractional -= std::floor(fractional);
            decimals++;
        }
        return decimals;
    }
    


    void createGraph() 
    {   
        auto start_time_ = std::chrono::high_resolution_clock::now();
        navigableVerticesMap.clear();
        navigableEdges_.clear();
        navigableVertices_.clear();
        int id_counter = navigableVerticesMap.size();
        double new_x, new_y, new_z = 0.0;

        for (double x = x_min_; x <= x_max_; x += distanceToObstacle_) {
            for (double y = y_min_; y <= y_max_; y += distanceToObstacle_) {
                for (double z = z_min_; z <= z_max_; z += distanceToObstacle_) {
                   
                    
                    //Arredondando as posições dos vértices navegáveis para ficar em posições sincronizadas com os vértices que são obstaculos.
                    new_x = roundToMultiple(x, distanceToObstacle_, decimals);
                    new_y = roundToMultiple(y, distanceToObstacle_, decimals);
                    new_z = roundToMultipleFromBase(z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);    
                    auto index = std::make_tuple(static_cast<double>(new_x), 
                    static_cast<double>(new_y), 
                    static_cast<double>(new_z));
                    
                    if (obstaclesVertices_.find(index) == obstaclesVertices_.end() && navigableVerticesMap.find(index) == navigableVerticesMap.end())
                    {
                        Vertex v;
                        v.key = id_counter;
                        v.x = new_x;
                        v.y = new_y;
                        v.z = new_z;
                                            
                        navigableVertices_.push_back(v);
                        navigableVerticesMap[index] = v;
                        
                        id_counter++;
                    }
                }
            }
        }

    
    }

    void adjustGraph()
    {
        /*
            Caso exista um vértice em obstacleVertices com o mesmo index de algum vértice em navigableVerticesMap, 
            então o vértice em navigableVerticesMap será removido.
        */
        auto start_time_ = std::chrono::high_resolution_clock::now();

        for (const auto& [key, _] : obstaclesVertices_) {

            if(_.verified == false)
            {
                 if (navigableVerticesMap.find(key) != navigableVerticesMap.end()) 
                {
                    navigableVertices_[navigableVerticesMap[key].key] = navigableVertices_.back();
                    navigableVertices_.pop_back();
                    
                    navigableVerticesMap.erase(key);
                    removedNavigableVertices[key] = {key, _.x, _.y, _.z, _.up, _.down};
                    
                    //apague o indice key de adjancy list aqui
                }
                else if(fixedNavigableVerticesMap.find(key) != fixedNavigableVerticesMap.end())
                {
                    if(navigableVerticesMap.find(key) != navigableVerticesMap.end()) 
                    {
                        navigableVertices_[navigableVerticesMap[key].key] = navigableVertices_.back();
                        navigableVertices_.pop_back();
                        navigableVerticesMap.erase(key);
                    }

                    fixedNavigableVerticesMap.erase(key);
                    removedNavigableVertices[key] = {key, _.x, _.y, _.z, _.up, _.down};
                }


                obstaclesVertices_[key].verified = true;
            }
           
        }

        

        
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end_time - start_time_;

      
    }

   
    /*

        PUBLISHERS.

    */

    // void publish_visualize_navigable_vertices()
    // {
    //     auto start_time_ = std::chrono::high_resolution_clock::now();
    //     sensor_msgs::msg::PointCloud2 cloud_msg1;
    //     cloud_msg1.header.stamp = this->get_clock()->now();
    //     cloud_msg1.header.frame_id = "map";

    //     // Configuração dos campos do PointCloud2
    //     cloud_msg1.height = 1;  // Ponto único em cada linha
    //     cloud_msg1.width = navigableVerticesMap.size(); // Quantidade de vértices
    //     cloud_msg1.is_dense = true;
    //     cloud_msg1.is_bigendian = false;
    //     cloud_msg1.point_step = 3 * sizeof(float); // x, y, z
    //     cloud_msg1.row_step = cloud_msg1.point_step * cloud_msg1.width;

    //     // Adicionar campos de x, y, z
    //     sensor_msgs::PointCloud2Modifier modifier(cloud_msg1);
    //     modifier.setPointCloud2FieldsByString(1, "xyz");
    //     modifier.resize(cloud_msg1.width);

    //     // Preencher os dados do PointCloud2
    //     sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg1, "x");
    //     sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg1, "y");
    //     sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg1, "z");

    //     for (const auto& [key, vertex] : navigableVerticesMap) 
    //     {
                
    //         *iter_x = vertex.x;  // Access the x coordinate of the current Vertex
    //         *iter_y = vertex.y;  // Access the y coordinate of the current Vertex
    //         *iter_z = vertex.z;  // Access the z coordinate of the current Vertex

    //         ++iter_x;
    //         ++iter_y;
    //         ++iter_z;
            
 
    //     }
    //     publisher_visualize_navigable_graph->publish(cloud_msg1);

    //     auto end_time = std::chrono::high_resolution_clock::now();
    //     std::chrono::duration<double> duration = end_time - start_time_;
    // }


    void publish_removed_navigable_vertices()
    { 
        subdrone_interfaces::msg::PassarArrayVertices removedVerticesMessage;
        //size_t contador = 0;
        for (const auto& [key, vertex] : removedNavigableVertices) 
        {
            

            subdrone_interfaces::msg::PassarVertices vertex_msg;
            vertex_msg.up = vertex.up;
            vertex_msg.down = vertex.down;
            vertex_msg.x   = vertex.x;
            vertex_msg.y   = vertex.y;
            vertex_msg.z   = vertex.z;

            removedVerticesMessage.data.push_back(vertex_msg);  
            //contador++;
        }

       // std::cout << contador << std::endl;

        
        publisher_removed_navigable_vertices_->publish(removedVerticesMessage);
    }



    /*
    
        CALLBACKS.

    */



    void callback_obstacles(const subdrone_interfaces::msg::PassarArrayVertices::SharedPtr msg)
    {
        
 

        size_t i = 0;
        for (const auto& vertex : msg->data) 
        {
            auto index = std::make_tuple(vertex.x, vertex.y, vertex.z);
            ObstaclesVertices vertexObstacles;
            vertexObstacles.key = i;  
            vertexObstacles.up = vertex.up;
            vertexObstacles.down = vertex.down;
            vertexObstacles.x = vertex.x;
            vertexObstacles.y = vertex.y;
            vertexObstacles.z = vertex.z;
            vertexObstacles.verified = false;
            
            obstaclesVertices_.emplace(index, vertexObstacles);
            i++;
        }


       
        
        
        adjustGraph();
    
    }
   

    void callback_fixed_vertices(const subdrone_interfaces::msg::PassarArrayVertices::SharedPtr msg)
    {
        size_t i = 0;
        for (const auto& vertex : msg->data) 
        {  
            auto index = std::make_tuple(vertex.x, vertex.y, vertex.z);
            if(obstaclesVertices_.find(index) == obstaclesVertices_.end())
            {
                ObstaclesVertices fixedVertex;
                fixedVertex.key = i;  // ou std::stoi(index) se necessário
                fixedVertex.x   = vertex.x;
                fixedVertex.y   = vertex.y;
                fixedVertex.z   = vertex.z;
                fixedVertex.verified = false;
                fixedNavigableVerticesMap.emplace(index, fixedVertex);
            }
            else
            {
                fixedNavigableVerticesMap.erase(index);
            }
        

            i++;
        
    
           
        }

        
        
        adjustGraph();
    }

    void check_parameters()
    {
        // Obtendo parâmetros como double e convertendo para double
        auto new_distanceToObstacle = static_cast<double>(this->get_parameter("distanceToObstacle").get_parameter_value().get<double>());
        auto new_x_min = static_cast<double>(this->get_parameter("x_min").get_parameter_value().get<double>());
        auto new_x_max = static_cast<double>(this->get_parameter("x_max").get_parameter_value().get<double>());
        auto new_y_min = static_cast<double>(this->get_parameter("y_min").get_parameter_value().get<double>());
        auto new_y_max = static_cast<double>(this->get_parameter("y_max").get_parameter_value().get<double>());
        auto new_z_min = static_cast<double>(this->get_parameter("z_min").get_parameter_value().get<double>());
        auto new_z_max = static_cast<double>(this->get_parameter("z_max").get_parameter_value().get<double>());

        auto new_resolution = this->get_parameter("resolution").get_parameter_value().get<int>();

        if (new_distanceToObstacle != distanceToObstacle_) 
        {
            distanceToObstacle_ = new_distanceToObstacle;
            resolution_ = 1;
            std::cout << "\n" << std::endl;
            RCLCPP_INFO(this->get_logger(), "Updated DistanceToObstacle: %.2f", distanceToObstacle_);
            RCLCPP_INFO(this->get_logger(), "Resolution set to 1.");
            createGraph();           
        }
        else if (new_resolution != temp_)
        {   
            temp_ = new_resolution;
            std::cout << "\n" << std::endl;

            if (new_resolution <= -1)
            {
                resolution_ = std::abs(1.0 / new_resolution);
                RCLCPP_INFO(this->get_logger(), "Updated resolution: %ld", new_resolution);
                createGraph();
            }
            else if (new_resolution == 0)
            {
                RCLCPP_WARN(this->get_logger(), "Resolution cannot be 0.");
            }
            else
            {
                resolution_ = new_resolution;
                RCLCPP_INFO(this->get_logger(), "Updated resolution: %f", resolution_);
                createGraph();
            }
        }

        if (new_x_min != x_min_) 
        {
            std::cout << "\n" << std::endl;
            x_min_ = new_x_min;
            RCLCPP_INFO(this->get_logger(), "Updated x_min: %.2f", x_min_);
            createGraph();
        }
        if (new_x_max != x_max_) 
        {
            std::cout << "\n" << std::endl;
            x_max_ = new_x_max;
            RCLCPP_INFO(this->get_logger(), "Updated x_max: %.2f", x_max_);
            createGraph();
        }
        if (new_y_min != y_min_) 
        {
            std::cout << "\n" << std::endl;
            y_min_ = new_y_min;
            RCLCPP_INFO(this->get_logger(), "Updated y_min: %.2f", y_min_);
            createGraph();
        }
        if (new_y_max != y_max_) 
        {
            std::cout << "\n" << std::endl;
            y_max_ = new_y_max;
            RCLCPP_INFO(this->get_logger(), "Updated y_max: %.2f", y_max_);
            createGraph();
        }        
        if (new_z_min != z_min_) 
        {
            std::cout << "\n" << std::endl;
            z_min_ = new_z_min;
            RCLCPP_INFO(this->get_logger(), "Updated z_min: %.2f", z_min_);
            createGraph();
        }
        if (new_z_max != z_max_) 
        {
            std::cout << "\n" << std::endl;
            z_max_ = new_z_max;
            RCLCPP_INFO(this->get_logger(), "Updated z_max: %.2f", z_max_);
            createGraph();
        }
    }


public:
    NavigableGraph()
     : Node("navigable_graph")
    {
    
        this->declare_parameter<double>("distanceToObstacle", 0.05);
        this->declare_parameter<int>("resolution", 1);
        this->declare_parameter<double>("x_min", -10.0);
        this->declare_parameter<double>("x_max", 10.0);
        this->declare_parameter<double>("y_min", -10.0);
        this->declare_parameter<double>("y_max", 10.0);
        this->declare_parameter<double>("z_min", 0.2);
        this->declare_parameter<double>("z_max", 0.2);

        // Initialize parameters as double usando static_cast
        distanceToObstacle_ = static_cast<double>(this->get_parameter("distanceToObstacle").get_parameter_value().get<double>());
        resolution_ = this->get_parameter("resolution").get_parameter_value().get<int>();
        x_min_ = static_cast<double>(this->get_parameter("x_min").get_parameter_value().get<double>());
        x_max_ = static_cast<double>(this->get_parameter("x_max").get_parameter_value().get<double>());
        y_min_ = static_cast<double>(this->get_parameter("y_min").get_parameter_value().get<double>());
        y_max_ = static_cast<double>(this->get_parameter("y_max").get_parameter_value().get<double>());
        z_min_ = static_cast<double>(this->get_parameter("z_min").get_parameter_value().get<double>());
        z_max_ = static_cast<double>(this->get_parameter("z_max").get_parameter_value().get<double>());

        RCLCPP_INFO(this->get_logger(), "Updated DistanceToObstacle: %f", distanceToObstacle_);
        RCLCPP_INFO(this->get_logger(), "Resolution is set to: %0.f", resolution_);
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

       subscription_ = this->create_subscription<subdrone_interfaces::msg::PassarArrayVertices>(
            "/vertices", 10, std::bind(&NavigableGraph::callback_obstacles, this, std::placeholders::_1));

        subscription_fixed_vertices = this->create_subscription<subdrone_interfaces::msg::PassarArrayVertices>(
            "/fixed_vertices", 10, std::bind(&NavigableGraph::callback_fixed_vertices, this, std::placeholders::_1));

        // publisher_visualize_navigable_graph = this->create_publisher<sensor_msgs::msg::PointCloud2>("/visualize_navigable_vertices", 10);
        // timer_visualize_navigable_graph = this->create_wall_timer(1ms, std::bind(&NavigableGraph::publish_visualize_navigable_vertices, this));

        publisher_removed_navigable_vertices_ = this->create_publisher<subdrone_interfaces::msg::PassarArrayVertices>("/removed_navigable_vertices", 10);
        timer_removed_navigable_vertices = this->create_wall_timer(1ms, std::bind(&NavigableGraph::publish_removed_navigable_vertices, this));



        createGraph();
    }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<NavigableGraph>());
    rclcpp::shutdown();
    return 0;
}

