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
#include <tuple>
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "subdrone_interfaces/msg/passar_vertices.hpp"
#include "subdrone_interfaces/msg/passar_array_vertices.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/path.hpp>
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

template<typename T1, typename T2, typename T3>
std::ostream& operator<<(std::ostream& os, const std::tuple<T1, T2, T3>& t) {
    os << "(" << std::get<0>(t) << ", " 
       << std::get<1>(t) << ", " 
       << std::get<2>(t) << ")";
    return os;
}


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
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription3_;

    //Timers.
    rclcpp::TimerBase::SharedPtr timer_navegable_vertices_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_path_;
    rclcpp::TimerBase::SharedPtr timer_visualize_path_;
    rclcpp::TimerBase::SharedPtr parameterTimer;


    size_t i_ = 0; 
    int temp_ = 1, maxSize = 0;
    double resolution_;
    double pose_x_ = 0.0, pose_y_ = 0.0, pose_z_ = 0.0;
    double distanceToObstacle_;
    double x_min_, x_max_;
    double y_min_, y_max_;
    double z_min_, z_max_;
    
    int decimals = 0;

    std::tuple<double, double, double> globalGoalIndex;
    std::tuple<double, double, double> globalIndex;

    std::vector<std::tuple<double, double, double>> destinationEdges;
    std::vector<VertexDijkstra> verticesDestino_;
    std::vector<VertexDijkstra> verticesDijkstra;
    std::vector<Edge> shortestPathEdges;
    std::vector<Edge> navigableEdges_; 

    std::unordered_map<std::pair<int, int>, double, pair_hash> distances;
    std::unordered_map<int, std::vector<int>> adjacency_list;
    std::unordered_map<std::string, Vertex> fixedNavigableVerticesMap;
    std::unordered_map<std::tuple<double, double, double>, Vertex> obstaclesVertices_;
    std::unordered_map<std::tuple<double, double, double>, Vertex> navigableVerticesMap;
    std::unordered_map<int, Vertex> navigableVerticesMapInteger;
    
  
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
        int id_counter = 0;
        navigableVerticesMapInteger.clear();
        
        /*
            Pegar a quantidade de decimais de distanceToObstacle.
        */
       
    
        double new_x, new_y, new_z = 0.0;
        
        for (double x = x_min_; x <= x_max_; x += distanceToObstacle_) {
            for (double y = y_min_; y <= y_max_; y += distanceToObstacle_) {
                for (double z = z_min_; z <= z_max_; z += distanceToObstacle_) {
                   
                    new_x = roundToMultiple(x, distanceToObstacle_, decimals);
                    new_y = roundToMultiple(y, distanceToObstacle_, decimals);
                    new_z = roundToMultipleFromBase(z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);    

                    auto index = std::make_tuple(static_cast<double>(new_x), 
                        static_cast<double>(new_y), 
                        static_cast<double>(new_z));

                    

                    if (navigableVerticesMap.find(index) == navigableVerticesMap.end())
                    {
                        Vertex v;
                        v.key = id_counter;
                        v.x = new_x;
                        v.y = new_y;
                        v.z = new_z;
    
                        navigableVerticesMapInteger[id_counter] = v;
                        navigableVerticesMap[index] = v;
                        id_counter++;
                    }
                }
            }
        }

       
        maxSize = id_counter;

         double offsets1[26][3] = {
            {-distanceToObstacle_, 0.0, 0.0}, 
            {distanceToObstacle_, 0.0, 0.0},
            {0.0, distanceToObstacle_, 0.0}, 
            {0.0, -distanceToObstacle_, 0.0},  
            {0.0, 0.0, distanceToObstacle_}, 
            {0.0, 0.0, -distanceToObstacle_},
            {-distanceToObstacle_, distanceToObstacle_, 0.0},
            {distanceToObstacle_, distanceToObstacle_, 0.0},
            {-distanceToObstacle_, -distanceToObstacle_, 0.0},
            {distanceToObstacle_, -distanceToObstacle_, 0.0},

            {-distanceToObstacle_, 0.0, distanceToObstacle_}, 
            {distanceToObstacle_, 0.0, distanceToObstacle_},
            {0.0, distanceToObstacle_, distanceToObstacle_}, 
            {0.0, -distanceToObstacle_, distanceToObstacle_},  
            {-distanceToObstacle_, distanceToObstacle_, distanceToObstacle_},
            {distanceToObstacle_, distanceToObstacle_, distanceToObstacle_},
            {-distanceToObstacle_, -distanceToObstacle_, distanceToObstacle_},
            {distanceToObstacle_, -distanceToObstacle_, distanceToObstacle_},

            {-distanceToObstacle_, 0.0, -distanceToObstacle_}, 
            {distanceToObstacle_, 0.0, -distanceToObstacle_},
            {0.0, distanceToObstacle_, -distanceToObstacle_}, 
            {0.0, -distanceToObstacle_, -distanceToObstacle_},  
            {-distanceToObstacle_, distanceToObstacle_, -distanceToObstacle_},
            {distanceToObstacle_, distanceToObstacle_, -distanceToObstacle_},
            {-distanceToObstacle_, -distanceToObstacle_, -distanceToObstacle_},
            {distanceToObstacle_, -distanceToObstacle_, -distanceToObstacle_},
    
  
        };

        auto end_time1 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration1 = end_time1 - start_time_;
        std::cout << "Vertices created in: " << duration1.count() << " seconds" << std::endl;
        
    
        for (auto it = navigableVerticesMap.begin(); it != navigableVerticesMap.end(); ++it) 
        {
            for (int a = 0; a < 26; a++) 
            {
                 new_x = roundToMultiple(it->second.x + offsets1[a][0], distanceToObstacle_, decimals);
                 new_y = roundToMultiple(it->second.y + offsets1[a][1], distanceToObstacle_, decimals);
                 new_z = roundToMultipleFromBase(it->second.z + offsets1[a][2], roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);  
    
                auto index = std::make_tuple(static_cast<double>(new_x), 
                static_cast<double>(new_y), 
                static_cast<double>(new_z));

               
                if (navigableVerticesMap.find(index) != navigableVerticesMap.end())
                { 
                    adjacency_list[navigableVerticesMap[index].key].push_back(it->second.key);
                   
                } 
            }
        }
     
     

        RCLCPP_INFO(this->get_logger(), "Graph created with %zu vertices AND %zu edges", navigableVerticesMapInteger.size(), adjacency_list.size());
        RCLCPP_INFO(this->get_logger(), "NAVIGABLEVERTICESMAP: %zu ", navigableVerticesMap.size());

        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end_time - start_time_;
        std::cout << "Graph created in: " << duration.count() << " seconds" << std::endl;
    
    }
    
   
    std::vector<int> runAStar(double start[3], double goal[3]) 
    {
        destinationEdges.clear();
        std::unordered_map<int, int> came_from;
        std::unordered_map<int, double> g_score;
        std::unordered_map<int, double> f_score;
        std::unordered_set<int> closed_set;
       

        // Determina os índices de start e goal
        int start_index = -1, end_index = -1;

    
        /*

            EU PODERIA COLOCAR TUDO ISSO EM UMA FUNÇÃOO E SIMPLIFICAR TUDO,
            MAS FIQUEI COM MEDO DE QUEBRAR O CÓDIGO.

        */

    
        Vertex v;
        v.key = maxSize;
        start_index = maxSize;
        v.x = start[0];
        v.y = start[1];
        v.z = start[2];

        auto index = std::make_tuple(v.x, v.y, v.z);
    
        navigableVerticesMap[index] = v;
        navigableVerticesMapInteger[start_index] = v;

        globalIndex = index;
        double offsets1[26][3] = {
            {-distanceToObstacle_, 0.0, 0.0}, 
            {distanceToObstacle_, 0.0, 0.0},
            {0.0, distanceToObstacle_, 0.0}, 
            {0.0, -distanceToObstacle_, 0.0},  
            {0.0, 0.0, distanceToObstacle_}, 
            {0.0, 0.0, -distanceToObstacle_},
            {-distanceToObstacle_, distanceToObstacle_, 0.0},
            {distanceToObstacle_, distanceToObstacle_, 0.0},
            {-distanceToObstacle_, -distanceToObstacle_, 0.0},
            {distanceToObstacle_, -distanceToObstacle_, 0.0},

            {-distanceToObstacle_, 0.0, distanceToObstacle_}, 
            {distanceToObstacle_, 0.0, distanceToObstacle_},
            {0.0, distanceToObstacle_, distanceToObstacle_}, 
            {0.0, -distanceToObstacle_, distanceToObstacle_},  
            {-distanceToObstacle_, distanceToObstacle_, distanceToObstacle_},
            {distanceToObstacle_, distanceToObstacle_, distanceToObstacle_},
            {-distanceToObstacle_, -distanceToObstacle_, distanceToObstacle_},
            {distanceToObstacle_, -distanceToObstacle_, distanceToObstacle_},

            {-distanceToObstacle_, 0.0, -distanceToObstacle_}, 
            {distanceToObstacle_, 0.0, -distanceToObstacle_},
            {0.0, distanceToObstacle_, -distanceToObstacle_}, 
            {0.0, -distanceToObstacle_, -distanceToObstacle_},  
            {-distanceToObstacle_, distanceToObstacle_, -distanceToObstacle_},
            {distanceToObstacle_, distanceToObstacle_, -distanceToObstacle_},
            {-distanceToObstacle_, -distanceToObstacle_, -distanceToObstacle_},
            {distanceToObstacle_, -distanceToObstacle_, -distanceToObstacle_},

        };
      
        double new_x = 0.0, new_y = 0.0, new_z = 0.0;
        bool findNavigableVertice = false;

        for(int i = 1; i <= 2; i++)
        {
            for (int a = 0; a < 26; a++) 
            {
                new_x = roundToMultiple(navigableVerticesMap[index].x + (offsets1[a][0] * i), distanceToObstacle_, decimals);
                new_y = roundToMultiple(navigableVerticesMap[index].y + (offsets1[a][1] * i), distanceToObstacle_, decimals);
                new_z = roundToMultipleFromBase(navigableVerticesMap[index].z + (offsets1[a][2] * i), roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);  

                auto index1 = std::make_tuple(static_cast<double>(new_x), 
                static_cast<double>(new_y), 
                static_cast<double>(new_z));

                
                if (navigableVerticesMap.find(index1) != navigableVerticesMap.end())
                { 
                    adjacency_list[navigableVerticesMap[index].key].push_back(navigableVerticesMap[index1].key);
                    findNavigableVertice = true;
                } 
            }

            if(findNavigableVertice == true)
            {
                break;
            }
        }

        if(findNavigableVertice == false) 
        {
            RCLCPP_WARN(this->get_logger(), "The robot is too far of the navigable area.");
            return {};
        }


        Vertex vGoal;
        vGoal.key = maxSize + 1;
        end_index = maxSize + 1;
        vGoal.x = goal[0];
        vGoal.y = goal[1];
        vGoal.z = goal[2];

    
        auto goalIndex = std::make_tuple(vGoal.x, vGoal.y, vGoal.z);

        navigableVerticesMap[goalIndex] = vGoal;
        navigableVerticesMapInteger[end_index] = vGoal;
        
        bool findNavigableGoalVertice = false;
        globalGoalIndex = goalIndex;
        double new_x1 = 0.0, new_y1 = 0.0, new_z1 = 0.0;

     
        for(int i = 1; i <= 2; i++)
        {
            for (int a = 0; a < 26; a++) 
            {
                new_x1 = roundToMultiple(navigableVerticesMap[goalIndex].x + (offsets1[a][0] * i), distanceToObstacle_, decimals);
                new_y1 = roundToMultiple(navigableVerticesMap[goalIndex].y + (offsets1[a][1] * i), distanceToObstacle_, decimals);
                new_z1 = roundToMultipleFromBase(navigableVerticesMap[goalIndex].z + (offsets1[a][2] * i), roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);  

                auto index2 = std::make_tuple(static_cast<double>(new_x1), 
                static_cast<double>(new_y1), 
                static_cast<double>(new_z1));

                
                if (navigableVerticesMap.find(index2) != navigableVerticesMap.end())
                { 

                    /*
                        Tem que apagar esssa caras quando chegar ao destino.
                    */
                    adjacency_list[navigableVerticesMap[index2].key].push_back(navigableVerticesMap[goalIndex].key);
                    destinationEdges.push_back(index2);
                    
                    findNavigableGoalVertice = true;
                } 
            }

            if(findNavigableGoalVertice == true)
            {
                break;
            }
        }
        
        if(findNavigableGoalVertice == false)
        {
            RCLCPP_WARN(this->get_logger(), "Destination is too far of the navigable area. Increase navigable area.");
            return {};   
        }

        // Define a função heurística (distância Euclidiana)
        auto heuristic = [&](int a, int b) 
        {

            // auto key = std::make_pair(std::min(a, b), std::max(a, b));

             //if (distances.find(key) == distances.end()) {
              //  const auto &pa = navigableVerticesMapInteger[a];
               // const auto &pb = navigableVerticesMapInteger[b];


                //distances[key] = std::sqrt(std::pow(pa.x - pb.x, 2) + std::pow(pa.y - pb.y, 2) + std::pow(pa.z - pb.z, 2));
            //}

    
            //return distances[key];
            double distance = 0.0;

            const auto &pa = navigableVerticesMapInteger[a];
            const auto &pb = navigableVerticesMapInteger[b];


        

            distance = std::sqrt(std::pow(pa.x - pb.x, 2) + std::pow(pa.y - pb.y, 2) + std::pow(pa.z - pb.z, 2));

            return distance;
        };

        // Inicializa os scores e o open_set (fila de prioridade com lazy deletion)
        g_score[start_index] = 0;
        f_score[start_index] = heuristic(start_index, end_index);

    
        // Fila de prioridade: par (f_score, node)
        std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<> > open_set;
        open_set.push({f_score[start_index], start_index});

        while (!open_set.empty()) {
            auto current_pair = open_set.top();
            open_set.pop();
            int current = current_pair.second;
            
            // Se o nó já foi visitado, pule
            if (closed_set.find(current) != closed_set.end())
                continue;
                
            // Verifica se a entrada está desatualizada
            if (current_pair.first > f_score[current])
                continue;
                
            // Adiciona ao conjunto fechado
            closed_set.insert(current);
            
            if (current == end_index) {
                return reconstructPath(came_from, current);
            }
            
            for (int neighbor : adjacency_list[current]) {
                // Pula vizinhos já processados
                if (closed_set.find(neighbor) != closed_set.end())
                    continue;
                    
                double tentative_g_score = g_score[current];
                
                if (g_score.find(neighbor) == g_score.end() || tentative_g_score < g_score[neighbor]) {
                    came_from[neighbor] = current;
                    g_score[neighbor] = tentative_g_score;
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, end_index);
                    open_set.push({f_score[neighbor], neighbor});
                }
            }
        }

        RCLCPP_WARN(this->get_logger(), "Não foi possível alcançar o destino.");
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
                vertex.x = navigableVerticesMapInteger[path[i]].x;
                vertex.y = navigableVerticesMapInteger[path[i]].y;
                vertex.z = navigableVerticesMapInteger[path[i]].z;

                // Calculate orientation (quaternion) between consecutive vertices
                if (i < path.size() - 1) {
                    const Vertex &current_vertex = navigableVerticesMapInteger[path[i]];
                    const Vertex &next_vertex = navigableVerticesMapInteger[path[i + 1]];

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
        verticesDestino_.clear();
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
            double dx = pose_x_ - static_cast<double>(verticesDestino_[i_].x);
            double dy = pose_y_ - static_cast<double>(verticesDestino_[i_].y);
            double dz = pose_z_ - static_cast<double>(verticesDestino_[i_].z);

            double distanciaAteODestino = sqrt(dx * dx + dy * dy + dz * dz);

            if(distanciaAteODestino <= distanceToObstacle_)
            {
                i_ = i_ + 1;
                adjacency_list.erase(navigableVerticesMap[globalGoalIndex].key);
                navigableVerticesMapInteger.erase(navigableVerticesMap[globalGoalIndex].key);
                navigableVerticesMap.erase(globalGoalIndex);
            
            }

       
            double array_inicial[3] = {pose_x_, pose_y_, pose_z_};
            double array_final[3] = {static_cast<double>(verticesDestino_[i_].x), static_cast<double>(verticesDestino_[i_].y), static_cast<double>(verticesDestino_[i_].z)};
            
            if(i_ == verticesDestino_.size())
            {
                i_ = 0;
            }

            

            auto start_time_ = std::chrono::high_resolution_clock::now();
            std::vector<int> shortestPath = runAStar(array_inicial, array_final);
           
            storeEdgesInPath(shortestPath);

            adjacency_list.erase(navigableVerticesMap[globalIndex].key);            
            navigableVerticesMapInteger.erase(navigableVerticesMap[globalIndex].key);
            navigableVerticesMap.erase(globalIndex);

           
            for(const auto& tuple : destinationEdges)
            {
               
                /*
                    Removendo o vértice de destino da lista de adjacência das tuplas em destinationEdges.
                    Isso ocorre porque o vértice de destino pode ser trocado e se ele for trocado, usará
                    a mesma key do vértice de destino anterior, então isso ocorre para não causar conflito.
                */
                adjacency_list[navigableVerticesMap[tuple].key].pop_back();
            }

            navigableVerticesMapInteger.erase(navigableVerticesMap[globalGoalIndex].key);
            
            navigableVerticesMap.erase(globalGoalIndex);
            
           
            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> duration = end_time - start_time_;
            std::cout << "A* execution time: " << duration.count() << " seconds" << std::endl;
        }
    }

   
    void callback_removed_navigable_vertices(const subdrone_interfaces::msg::PassarArrayVertices::SharedPtr msg)
    {
       
        for (const auto& vertex : msg->data) 
        {   
            auto index = std::make_tuple(roundToMultiple(vertex.x, distanceToObstacle_, decimals), roundToMultiple(vertex.y, distanceToObstacle_, decimals), roundToMultipleFromBase(vertex.z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals));
            

            if(navigableVerticesMap.find(index) != navigableVerticesMap.end())
            {
                adjacency_list.erase(navigableVerticesMap[index].key);
             
                navigableVerticesMapInteger.erase(navigableVerticesMap[index].key);
                navigableVerticesMap.erase(index);
            
            } 
         
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
        auto new_resolution = this->get_parameter("resolution").get_parameter_value().get<int>();
        auto new_x_min = this->get_parameter("x_min").get_parameter_value().get<double>();
        auto new_x_max = this->get_parameter("x_max").get_parameter_value().get<double>();
        auto new_y_min = this->get_parameter("y_min").get_parameter_value().get<double>();
        auto new_y_max = this->get_parameter("y_max").get_parameter_value().get<double>();
        auto new_z_min = this->get_parameter("z_min").get_parameter_value().get<double>();
        auto new_z_max = this->get_parameter("z_max").get_parameter_value().get<double>();
        
        
        if (new_distanceToObstacle != distanceToObstacle_) 
        {
            distanceToObstacle_ = new_distanceToObstacle;
            resolution_ = 1;
            std::cout << "\n" << std::endl;
            RCLCPP_INFO(this->get_logger(), "Updated DistanceToObstacle: %.2f", distanceToObstacle_);
            RCLCPP_INFO(this->get_logger(), "Resolution set to 1.");
            createGraph();           
        }
        else if(new_resolution != temp_)
        {   
            temp_ = new_resolution;

            std::cout << "\n" << std::endl;

            if(new_resolution <= -1)
            {
                resolution_ =  std::abs(1.0  / new_resolution);
                RCLCPP_INFO(this->get_logger(), "Updated resolution: %ld", new_resolution);
                createGraph();
            }
            else if(new_resolution == 0)
            {
                RCLCPP_WARN(this->get_logger(), "Resolution cannot be 0.");
            }
            else
            {
                resolution_ = new_resolution;
                RCLCPP_INFO(this->get_logger(), "Updated resolution: %0.f", resolution_);
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
    AStar()
     : Node("a_start")
    {
    
     
        this->declare_parameter<double>("distanceToObstacle", 0.05);
        this->declare_parameter<int>("resolution", 1);
        this->declare_parameter<double>("x_min", -10.0);
        this->declare_parameter<double>("x_max", 10.0);
        this->declare_parameter<double>("y_min", -10.0);
        this->declare_parameter<double>("y_max", 10.0);
        this->declare_parameter<double>("z_min", 0.2);
        this->declare_parameter<double>("z_max", 0.2);

        // Initialize parameters
        distanceToObstacle_ =  this->get_parameter("distanceToObstacle").get_parameter_value().get<double>();
        resolution_ = this->get_parameter("resolution").get_parameter_value().get<int>();
        x_min_ = this->get_parameter("x_min").get_parameter_value().get<double>();
        x_max_ = this->get_parameter("x_max").get_parameter_value().get<double>();
        y_min_ = this->get_parameter("y_min").get_parameter_value().get<double>();
        y_max_ = this->get_parameter("y_max").get_parameter_value().get<double>();
        z_min_ = this->get_parameter("z_min").get_parameter_value().get<double>();
        z_max_ = this->get_parameter("z_max").get_parameter_value().get<double>();

        RCLCPP_INFO(this->get_logger(), "Updated DistanceToObstacle: %f", distanceToObstacle_);
        RCLCPP_INFO(this->get_logger(), "Resolution is set to: %0.f", resolution_);
        RCLCPP_INFO(this->get_logger(), "Updated x_min: %f", x_min_);
        RCLCPP_INFO(this->get_logger(), "Updated x_max: %f", x_max_);
        RCLCPP_INFO(this->get_logger(), "Updated y_min: %f", y_min_);
        RCLCPP_INFO(this->get_logger(), "Updated y_max: %f", y_max_);
        RCLCPP_INFO(this->get_logger(), "Updated z_min: %f", z_min_);
        RCLCPP_INFO(this->get_logger(), "Updated z_max: %f", z_max_);


        parameterTimer = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&AStar::check_parameters, this));

        decimals = countDecimals(distanceToObstacle_);
       
 
        subscription_navigable_removed_vertices = this->create_subscription<subdrone_interfaces::msg::PassarArrayVertices>(
            "/removed_navigable_vertices", 10, std::bind(&AStar::callback_removed_navigable_vertices, this, std::placeholders::_1));

        publisher_nav_path_ = this->create_publisher<nav_msgs::msg::Path>("visualize_path", 10);
        timer_visualize_path_ = this->create_wall_timer(100ms, std::bind(&AStar::publisher_dijkstra_path, this));

        publisher_path_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/path", 10);
        timer_path_ = this->create_wall_timer(1ms, std::bind(&AStar::publisher_dijkstra, this));
        

        subscription_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/rtabmap/odom", 10, std::bind(&AStar::callback_odom, this, std::placeholders::_1));

        subscription3_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/destinations", 10, std::bind(&AStar::callback_destinations, this, std::placeholders::_1));


        createGraph();
    }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<AStar>());
    rclcpp::shutdown();
    return 0;
}