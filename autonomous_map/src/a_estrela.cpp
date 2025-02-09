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
#include "subdrone_interfaces/msg/passar_arestas.hpp"
#include "subdrone_interfaces/msg/passar_array_vertices.hpp"
#include "subdrone_interfaces/msg/passar_array_arestas.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/path.hpp>
#include <cmath>
#include <cstring>
#include <utility> 

using namespace std::chrono_literals;

class AStar : public rclcpp::Node {

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

    struct pair_hash 
    {
        template <class T1, class T2>
        std::size_t operator()(const std::pair<T1, T2>& p) const {
            return std::hash<T1>()(p.first) ^ (std::hash<T2>()(p.second) << 1);
        }
    };

    //Publishers.
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_path_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_navegable_vertices_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_nav_path_;

    //Subscriptions.
    rclcpp::Subscription<subdrone_interfaces::msg::PassarArrayVertices>::SharedPtr subscription_;
    rclcpp::Subscription<subdrone_interfaces::msg::PassarArrayVertices>::SharedPtr subscription_fixed_vertices;
    rclcpp::Subscription<subdrone_interfaces::msg::PassarArrayVertices>::SharedPtr subscription_navigable_vertices;
    rclcpp::Subscription<subdrone_interfaces::msg::PassarArrayVertices>::SharedPtr subscription_navigable_removed_vertices;
    rclcpp::Subscription<subdrone_interfaces::msg::PassarArrayArestas>::SharedPtr subscription_navigable_edges;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription3_;

    //Timers.
    rclcpp::TimerBase::SharedPtr timer_navegable_vertices_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_path_;
    rclcpp::TimerBase::SharedPtr timer_visualize_path_;
    rclcpp::TimerBase::SharedPtr parameterTimer;


    int i_ = 0, temp_ = 1;
    double resolution_; 
    double pose_x_ = 0, pose_y_ = 0, pose_z_ = 0;
    double distanceToObstacle_;
    double x_min_, x_max_;
    double y_min_, y_max_;
    double z_min_, z_max_; 

    std::vector<Edge> edges_;
    std::string deleteKey;
    std::vector<VertexDijkstra> verticesDestino_;
    std::vector<VertexDijkstra> verticesDijkstra;
    std::vector<Edge> shortestPathEdges;
    std::vector<Edge> navigableEdges_; 
    std::vector<Vertex> navigableVertices_;
    std::unordered_map<int, double> g_score, f_score;
    std::unordered_map<int, int> came_from;
    std::unordered_map<std::pair<int, int>, double, pair_hash> distances;
    std::unordered_map<int, std::vector<int>> adjacency_list;
    std::unordered_map<std::string, Vertex> fixedNavigableVerticesMap;
    std::unordered_map<std::string, Vertex> obstaclesVertices_;
    std::unordered_map<std::string, Vertex> navigableVerticesMap_;
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> open_set;
   

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

      

   
    std::vector<int> runAStar(double start[3], double goal[3]) 
    {
       
        open_set = std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>>();
        g_score.clear();
        f_score.clear();
        came_from.clear();
        

        // Heuristic function to calculate the estimated distance
        auto heuristic = [&](int a, int b) 
        {
            auto key = std::make_pair(std::min(a, b), std::max(a, b));
            if (distances.find(key) == distances.end()) 
            {
                const auto &pa = navigableVertices_[a];
                const auto &pb = navigableVertices_[b];
                distances[key] = std::sqrt(std::pow(pa.x - pb.x, 2) + std::pow(pa.y - pb.y, 2) + std::pow(pa.z - pb.z, 2));
            }
            return distances[key];
        };


        int start_index = -1, end_index = -1;
     
        std::string indexGoal = concatenar(goal[0], goal[1], goal[2]);
        std::string indexStart = concatenar(start[0], start[1], start[2]);    

        if(navigableVerticesMap_.find(indexGoal) != navigableVerticesMap_.end())
        {
            end_index = navigableVerticesMap_[indexGoal].key;
        }
      
        if(navigableVerticesMap_.find(indexStart) != navigableVerticesMap_.end())
        {
            start_index = navigableVerticesMap_[indexStart].key;           
        }
        else if(obstaclesVertices_.find(indexStart) != navigableVerticesMap_.end())
        {
            /*

                ESSE LIXO TODO AQUI É PARA CASO ALGUM HORA O ROBÔ FIQUE EM CIMA DE UM VÉRTICE QUE
                FAZ PARTE DA ZONA DE SEGURANÇA DE UM OBJETO.

            */


            Vertex v;
            v.key = navigableVerticesMap_.size();
            v.x = obstaclesVertices_[indexStart].x;
            v.y = obstaclesVertices_[indexStart].y;
            v.z = obstaclesVertices_[indexStart].z;

            navigableVerticesMap_[indexStart] = v;
            deleteKey = indexStart;

            double offsets1[10][3] = {
                {-distanceToObstacle_, 0.0, 0.0}, {distanceToObstacle_, 0.0, 0.0},{0.0, distanceToObstacle_, 0.0}, {0.0, -distanceToObstacle_, 0.0},  {0.0, 0.0, distanceToObstacle_}, {0.0, 0.0, -distanceToObstacle_},{-distanceToObstacle_, distanceToObstacle_, 0.0},{distanceToObstacle_, distanceToObstacle_, 0.0},
                {-distanceToObstacle_, -distanceToObstacle_, 0.0},
                {distanceToObstacle_, -distanceToObstacle_, 0.0}
            };

            for (int a = 0; a < 10; a++) 
            {
                double new_x = obstaclesVertices_[indexStart].x + offsets1[a][0];
                double new_y = obstaclesVertices_[indexStart].y + offsets1[a][1];
                double new_z = obstaclesVertices_[indexStart].z + offsets1[a][2];

                std::string index = concatenar(new_x, new_y, new_z);

                if (navigableVerticesMap_.find(index) != navigableVerticesMap_.end())
                { 
                    adjacency_list[navigableVerticesMap_[index].key].push_back(navigableVerticesMap_[indexStart].key);
                    Edge newEdge = {navigableVerticesMap_[index].key, navigableVerticesMap_[indexStart].key};
                    navigableEdges_.push_back(newEdge);
                }        
            }

            start_index = navigableVerticesMap_[indexStart].key;
        }

        if(start_index == -1 )
        {
            RCLCPP_WARN(this->get_logger(), "O ROBÔ NÃO ESTÁ NO GRAFO.");
            return {};
        }

        
        if (end_index == -1 ) {
           
            RCLCPP_WARN(this->get_logger(), "Destination does not exist in current graph, increase graph size or ensure that the destination is not in the same place as an obstacle.");
            return {};
        }

       
        // Starting conditions
        g_score[start_index] = 0;
        f_score[start_index] = g_score[start_index] + heuristic(start_index, end_index);
        open_set.emplace(f_score[start_index], start_index);

        
        // A* algorithm loop
        while (!open_set.empty()) {
            int current = open_set.top().second;
            open_set.pop();
                     
            if (current == end_index) {

                return reconstructPath(came_from, current);
            }
            
            // Explore neighbors
            for (int neighbor : adjacency_list[current]) 
            {
                double tentative_g_score = 
                    (g_score.find(current) != g_score.end() ? g_score[current] : std::numeric_limits<double>::infinity()) 
                    + heuristic(current, neighbor);

                if (tentative_g_score < (g_score.find(neighbor) != g_score.end() ? g_score[neighbor] : std::numeric_limits<double>::infinity())) {
                    came_from[neighbor] = current;
                    g_score[neighbor] = tentative_g_score;
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, end_index);
                    open_set.emplace(f_score[neighbor], neighbor);
                }
            }
        }

        
        RCLCPP_WARN(this->get_logger(), "Unable to reach destination.");
        return {};
    }

    std::vector<int> reconstructPath(const std::unordered_map<int, int> &came_from, int current) 
    {
        std::vector<int> path;
        while (came_from.find(current) != came_from.end()) {
            path.push_back(current);
            current = came_from.at(current);
        }
        path.push_back(current);
        std::reverse(path.begin(), path.end());
        storeEdgesInPath(path);

        
        return path;
    }

    void storeEdgesInPath(const std::vector<int>& path) 
    {
        shortestPathEdges.clear();
        verticesDijkstra.clear();

        if(path.empty())
        {
            return;
        }
        else
        {
            for (size_t i = 0; i < path.size() - 1; i++) 
            {
                int u = path[i];
                int v = path[i + 1];
                shortestPathEdges.push_back({u, v});
            }

            for (size_t i = 0; i < path.size(); i++) 
            {
                VertexDijkstra vertex;
                vertex.x = navigableVertices_[path[i]].x;
                vertex.y = navigableVertices_[path[i]].y;
                vertex.z = navigableVertices_[path[i]].z;

                // Calculate orientation (quaternion) between consecutive vertices
                if (i < path.size() - 1) {
                    const Vertex &current_vertex = navigableVertices_[path[i]];
                    const Vertex &next_vertex = navigableVertices_[path[i + 1]];

                    // Calculate direction vector
                    double dx = next_vertex.x - current_vertex.x;
                    double dy = next_vertex.y - current_vertex.y;
                    double dz = next_vertex.z - current_vertex.z;
                    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

                    // Normalize direction vector
                    if (distance > 0.0f) {
                        dx /= distance;
                        dy /= distance;
                        dz /= distance;
                    }

                    // Define reference direction (e.g., forward vector, z-axis)
                    Eigen::Vector3f direction(dx, dy, dz);
                    Eigen::Vector3f reference(1.0f, 0.0f, 0.0f);

                    // Calculate rotation needed to align with desired direction
                    Eigen::Quaternionf quaternion = Eigen::Quaternionf::FromTwoVectors(reference, direction);

                    vertex.orientation_x = quaternion.x();
                    vertex.orientation_y = quaternion.y();
                    vertex.orientation_z = quaternion.z();
                    vertex.orientation_w = quaternion.w();
                } else {
                    // For the last vertex, keep orientation as identity (no rotation)
                    vertex.orientation_x = 0.0;
                    vertex.orientation_y = 0.0;
                    vertex.orientation_z = 0.0;
                    vertex.orientation_w = 1.0;
                }

                verticesDijkstra.push_back(vertex);
            }
        }


    }

    /*

        PUBLISHERS.

    */

    

    
    void publisher_dijkstra()
    {   
        geometry_msgs::msg::PoseArray message;
        message.header.stamp = this->now();
        message.header.frame_id = "map";

        for (const auto& vertex : verticesDijkstra) {
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

        publisher_path_->publish(message);
    }

    void publisher_dijkstra_path()
    {
        nav_msgs::msg::Path path_msg;
        // Configura o header
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";

        // Para cada vértice em verticesDijkstra, cria um PoseStamped e adiciona ao path
        for (const auto& vertex : verticesDijkstra)
        {
            geometry_msgs::msg::PoseStamped pose_stamped;
            // O header pode ter o mesmo timestamp do path ou um timestamp específico
            pose_stamped.header.stamp = this->now();
            pose_stamped.header.frame_id = "map";
            
            pose_stamped.pose.position.x = vertex.x;
            pose_stamped.pose.position.y = vertex.y;
            pose_stamped.pose.position.z = vertex.z;
            pose_stamped.pose.orientation.x = vertex.orientation_x;
            pose_stamped.pose.orientation.y = vertex.orientation_y;
            pose_stamped.pose.orientation.z = vertex.orientation_z;
            pose_stamped.pose.orientation.w = vertex.orientation_w;
            
            path_msg.poses.push_back(pose_stamped);
        }

        // Publica a mensagem
        publisher_nav_path_->publish(path_msg);
    }



    /*
    
        CALLBACKS.

    */

 
    void callback_destinations(const geometry_msgs::msg::PoseArray::SharedPtr msg) 
    {
        
        for (const auto& pose_in : msg->poses) {
            VertexDijkstra destino;

            destino.x = pose_in.position.x;
            destino.y = pose_in.position.y;
            destino.z = pose_in.position.z;

            destino.orientation_x = pose_in.orientation.x;
            destino.orientation_y = pose_in.orientation.y;
            destino.orientation_z = pose_in.orientation.z;
            destino.orientation_w = pose_in.orientation.w;

            verticesDestino_.push_back(destino);
        }
        

         
        if(!verticesDestino_.empty())
        {
             /*
                O dz não será utilizado aqui AINDA, porque no momento eu estou usando
            apenas o turtlebot3.
            */
            float dx = pose_x_ - verticesDestino_[i_].x;
            float dy = pose_y_ - verticesDestino_[i_].y;

            float distanciaAteODestino = sqrt(dx * dx + dy * dy);

            if(distanciaAteODestino < 0.10)
            {
                i_ = i_ + 1;
            }

            double rounded_pose_x = roundToMultiple(pose_x_, distanceToObstacle_);
            double rounded_pose_y = roundToMultiple(pose_y_, distanceToObstacle_);
            double rounded_pose_z = roundToMultiple(0.25, distanceToObstacle_);



            double array_inicial[3] = {rounded_pose_x, rounded_pose_y, rounded_pose_z};
            double array_final[3] = {roundToMultiple(verticesDestino_[i_].x, distanceToObstacle_), roundToMultiple(verticesDestino_[i_].y,distanceToObstacle_),roundToMultiple(verticesDestino_[i_].z, distanceToObstacle_)};

           // std::cout << pose_x_ << " " << pose_y_ << " " << pose_z_ << std::endl;

            if(i_ == verticesDestino_.size())
            {
                i_ = 0;
            }


            auto start_time_ = std::chrono::high_resolution_clock::now();
            std::vector<int> shortestPath = runAStar(array_inicial, array_final);
           
            storeEdgesInPath(shortestPath);

            navigableVerticesMap_.erase(deleteKey);

            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> duration = end_time - start_time_;
            std::cout << "A* execution time: " << duration.count() << " seconds" << std::endl;
        }
    }

    void callback_navigable_vertices(const subdrone_interfaces::msg::PassarArrayVertices::SharedPtr msg)
    {
        navigableVerticesMap_.clear();
        navigableVertices_.clear();

        size_t i = 0;
        for (const auto& vertex : msg->data) 
        {
            Vertex navigableVertices;
            navigableVertices.key = i;  // Usando a chave (v1 ou v2)
            navigableVertices.x = vertex.x;
            navigableVertices.y = vertex.y;
            navigableVertices.z = vertex.z;
        
        
            if(navigableVerticesMap_.find(vertex.key) == navigableVerticesMap_.end())
            {
                navigableVerticesMap_[vertex.key] = (navigableVertices);
                navigableVertices_.push_back(navigableVertices);
            }
              
            i++;
            
        }
    }

    void callback_navigable_edges(const subdrone_interfaces::msg::PassarArrayArestas::SharedPtr msg)
    {
        navigableEdges_.clear();
        adjacency_list.clear();
        size_t i = 0;
        for (const auto& edge : msg->data) 
        {       

            if(navigableVerticesMap_.find(edge.v1) != navigableVerticesMap_.end() && navigableVerticesMap_.find(edge.v2) != navigableVerticesMap_.end())
            {
                adjacency_list[navigableVerticesMap_[edge.v1].key].push_back(navigableVerticesMap_[edge.v2].key);
                Edge newEdge = {navigableVerticesMap_[edge.v1].key, navigableVerticesMap_[edge.v2].key};
                navigableEdges_.push_back(newEdge);
            }

        }
    }

    void callback_removed_navigable_vertices(const subdrone_interfaces::msg::PassarArrayVertices::SharedPtr msg)
    {
        size_t i = 0;
        for (const auto& vertex : msg->data) 
        {     
            if(navigableVerticesMap_.find(vertex.key) != navigableVerticesMap_.end())
            {
                adjacency_list.erase(navigableVerticesMap_[vertex.key].key);
                navigableVertices_[navigableVerticesMap_[vertex.key].key] = navigableVertices_.back();
                navigableVertices_.pop_back();
                navigableEdges_[navigableVerticesMap_[vertex.key].key] = navigableEdges_.back();
                navigableEdges_.pop_back();

    
                navigableVerticesMap_.erase(vertex.key);

            } 
            Vertex v;
            v.key = 0;
            v.x = vertex.x;
            v.y = vertex.y;
            v.z = vertex.z;


            obstaclesVertices_[vertex.key] = (v);
        }

       
    }
   

    void callback_odom(const nav_msgs::msg::Odometry::SharedPtr msg) 
    {
        pose_x_ = msg->pose.pose.position.x;
        pose_y_ = msg->pose.pose.position.y;
        pose_z_ = msg->pose.pose.position.z;

     
    }


    void check_parameters()
    {
      
        auto new_distanceToObstacle = this->get_parameter("distanceToObstacle").get_parameter_value().get<double>();
       
        if (new_distanceToObstacle != distanceToObstacle_) 
        {
            distanceToObstacle_ = new_distanceToObstacle;
           
            RCLCPP_INFO(this->get_logger(), "Updated DistanceToObstacle: %.2f", distanceToObstacle_);     
        }
    
    }
    
   
public:
    AStar()
     : Node("a_start")
    {
    
     
        this->declare_parameter<double>("distanceToObstacle", 0.05);
   

        // Initialize parameters
        distanceToObstacle_ = this->get_parameter("distanceToObstacle").get_parameter_value().get<double>();
       

        RCLCPP_INFO(this->get_logger(), "Updated DistanceToObstacle: %f", distanceToObstacle_);
 
        subscription_navigable_vertices = this->create_subscription<subdrone_interfaces::msg::PassarArrayVertices>(
            "/navigable_vertices", 10, std::bind(&AStar::callback_navigable_vertices, this, std::placeholders::_1));

        subscription_navigable_edges = this->create_subscription<subdrone_interfaces::msg::PassarArrayArestas>(
            "/navigable_edges", 10, std::bind(&AStar::callback_navigable_edges, this, std::placeholders::_1));

        subscription_navigable_removed_vertices = this->create_subscription<subdrone_interfaces::msg::PassarArrayVertices>(
            "/removed_navigable_vertices", 10, std::bind(&AStar::callback_removed_navigable_vertices, this, std::placeholders::_1));

        publisher_nav_path_ = this->create_publisher<nav_msgs::msg::Path>("visualize_path", 10);
        timer_visualize_path_ = this->create_wall_timer(100ms, std::bind(&AStar::publisher_dijkstra_path, this));

        publisher_path_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/path", 10);
        timer_path_ = this->create_wall_timer(1ms, std::bind(&AStar::publisher_dijkstra, this));
        

        subscription_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&AStar::callback_odom, this, std::placeholders::_1));

        subscription3_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/destinations", 10, std::bind(&AStar::callback_destinations, this, std::placeholders::_1));



    }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<AStar>());
    rclcpp::shutdown();
    return 0;
}




