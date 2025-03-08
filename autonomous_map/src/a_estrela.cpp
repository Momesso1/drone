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
#include <iomanip>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <filesystem>


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

namespace std {
    template<>
    struct hash<std::tuple<std::pair<int, int>, bool>> {
        size_t operator()(const std::tuple<std::pair<int, int>, bool>& t) const {
            const auto& p = std::get<0>(t);
            bool b = std::get<1>(t);
            size_t h1 = std::hash<int>{}(p.first);
            size_t h2 = std::hash<int>{}(p.second);
            size_t h3 = std::hash<bool>{}(b);
            size_t seed = h1;
            seed ^= h2 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= h3 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            return seed;
        }
    };
}

template <typename T1, typename T2>
struct pair_hash {
    std::size_t operator ()(const std::pair<T1, T2>& p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        return h1 ^ (h2 << 1);  // Combinando os hashes
    }
};



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
        float x, y, z;
    };

    struct VertexDijkstra {
        float x, y, z;
        float orientation_x, orientation_y, orientation_z;
        float orientation_w;
    };

    struct Destinos {
        float x, y, z;
        float orientation_x, orientation_y, orientation_z;
        float orientation_w;
    };

    struct Edge {
        int v1, v2;
    };

    struct CompareWithTieBreaker {
        bool operator()(const std::pair<float, int>& a, const std::pair<float, int>& b) const {
            if (std::abs(a.first - b.first) < 1e-6) {
                // Desempata pelo índice do nó
                return a.second > b.second;
            }
            return a.first > b.first;
        }
    };
    struct PairHash {
        std::size_t operator()(const std::pair<int, int>& p) const {
            // Combinar os hashes dos dois elementos do par
            auto h1 = std::hash<int>{}(p.first);
            auto h2 = std::hash<int>{}(p.second);
            return h1 ^ (h2 << 1);
        }
    };

    

 
    //Publishers.
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_path_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_navegable_vertices_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_nav_path_;

    //Subscriptions.
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_navigable_removed_vertices;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription3_;

    //Timers.
    rclcpp::TimerBase::SharedPtr timer_navegable_vertices_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_path_;
    rclcpp::TimerBase::SharedPtr timer_visualize_path_;
    rclcpp::TimerBase::SharedPtr parameterTimer;


    size_t i_ = 0; 
    int temp_ = 1, maxSize = 0, diagonalEdges_, xVertices, yVertices;
    float resolution_;
    float pose_x_ = 0.0, pose_y_ = 0.0, pose_z_ = 0.0;
    float distanceToObstacle_;
    float x_min_, x_max_;
    float y_min_, y_max_;
    float z_min_, z_max_;
    int totalVertices;
    int zVertices;

    int decimals = 0;

    std::tuple<float, float, float> globalGoalIndex;
    std::tuple<float, float, float> globalIndex;

    std::vector<std::tuple<float, float, float>> destinationEdges;
    std::vector<VertexDijkstra> verticesDestino_;
    std::vector<VertexDijkstra> verticesDijkstra;
    std::vector<Edge> shortestPathEdges;
    std::vector<Edge> navigableEdges_; 

    
    std::unordered_map<int, std::vector<std::pair<int, int>>> relatedEdges;
    
    std::unordered_map<int, std::vector<int>> adjacency_list;
    //std::unordered_map<int, std::unordered_set<std::pair<int, int>, PairHash>> adjacency_list;
    std::unordered_map<std::string, Vertex> fixedNavigableVerticesMap;
    std::unordered_map<std::tuple<float, float, float>, Vertex> obstaclesVertices_;
    std::unordered_map<std::tuple<float, float, float>, Vertex> navigableVerticesMap;
    std::unordered_map<int, Vertex> navigableVerticesMapInteger;

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
    

    

    std::vector<std::array<float, 3>> getOffsets(float distanceToObstacle) {
        return {
           
            
            
            {0.0, 0.0, -distanceToObstacle},
            
 
            {-distanceToObstacle, 0.0, distanceToObstacle},
            {distanceToObstacle, 0.0, distanceToObstacle},
            {0.0, 0.0, distanceToObstacle},
            
            
            
            
            {-distanceToObstacle, distanceToObstacle, distanceToObstacle},
            {0.0, distanceToObstacle, distanceToObstacle},
            {-distanceToObstacle, -distanceToObstacle, distanceToObstacle},
            {0.0, -distanceToObstacle, distanceToObstacle},
            {distanceToObstacle, -distanceToObstacle, distanceToObstacle},
            {-distanceToObstacle, 0.0, -distanceToObstacle},
            
            
            {0.0, distanceToObstacle, -distanceToObstacle},
            {-distanceToObstacle, -distanceToObstacle, -distanceToObstacle},
            
            {0.0, -distanceToObstacle, -distanceToObstacle},
            {distanceToObstacle, -distanceToObstacle, -distanceToObstacle},

            {distanceToObstacle, 0.0, -distanceToObstacle},
            {-distanceToObstacle, distanceToObstacle, -distanceToObstacle},
            {distanceToObstacle, distanceToObstacle, -distanceToObstacle},

            {-distanceToObstacle, -distanceToObstacle, 0.0},
            {distanceToObstacle, -distanceToObstacle, 0.0},
            {distanceToObstacle, distanceToObstacle, 0.0},
            {-distanceToObstacle, distanceToObstacle, 0.0}, 

            {-distanceToObstacle, 0.0, 0.0},
            {distanceToObstacle, 0.0, 0.0},


            {0.0, distanceToObstacle, 0.0},
            {0.0, -distanceToObstacle, 0.0},
          

        };
    }
    
    
    void createGraph() 
    {
        auto start_time_ = std::chrono::high_resolution_clock::now();
        navigableVerticesMap.clear();
        navigableEdges_.clear();
        int id_counter = 0;
        navigableVerticesMapInteger.clear();

        

       
        
        float new_x, new_y, new_z = 0.0;
        int z_counter = 0;
        for (float z = z_min_; z <= z_max_; z += distanceToObstacle_) 
        {
            z_counter++;
            int y_counter = 0;
            for (float y = y_min_; y <= y_max_; y += distanceToObstacle_) 
            {
                int x_counter = 0;
                
                y_counter++;
                for (float x = x_min_; x <= x_max_; x += distanceToObstacle_) 
                {
                
                    new_x = roundToMultiple(x, distanceToObstacle_, decimals);
                    new_y = roundToMultiple(y, distanceToObstacle_, decimals);
                    new_z = roundToMultipleFromBase(z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);    

                    auto index = std::make_tuple(static_cast<float>(new_x), 
                        static_cast<float>(new_y), 
                        static_cast<float>(new_z));

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
                        x_counter++;
                    }
                    
                }

                
                xVertices = x_counter;
            }
            
            yVertices = y_counter;
        }

        totalVertices = xVertices * yVertices * z_counter;
        zVertices = xVertices * yVertices;

        
        auto end_time1 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> duration1 = end_time1 - start_time_;
        std::cout << "Vertices created in: " << duration1.count() << " seconds" << std::endl;
        
        maxSize = id_counter;


        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> duration = end_time - start_time_;
        std::cout << "Graph created in: " << duration.count() << " seconds" << std::endl;

    }
    

   
    std::vector<int> runAStar(float start[3], float goal[3]) 
    {
        
        destinationEdges.clear();
        std::unordered_map<int, int> came_from;
        std::unordered_map<int, float> g_score;
        std::unordered_map<int, float> f_score;
        std::unordered_set<int> closed_set;
       

        // Determina os índices de start e goal
        int start_index = -1, end_index = -1;

        /*
            Criando um vértice na posição atual do robô.
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

        auto offsets1 = getOffsets(distanceToObstacle_);
      
        float new_x = 0.0, new_y = 0.0, new_z = 0.0;
        bool findNavigableVertice = false;
        for(int i = 1; i <= 2; i++)
        {
            std::vector<int> edges;
            for (int a = 0; a < 26; a++) 
            {
                new_x = roundToMultiple(navigableVerticesMap[index].x + (offsets1[a][0] * i), distanceToObstacle_, decimals);
                new_y = roundToMultiple(navigableVerticesMap[index].y + (offsets1[a][1] * i), distanceToObstacle_, decimals);
                new_z = roundToMultipleFromBase(navigableVerticesMap[index].z + (offsets1[a][2] * i), roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);  

                auto index1 = std::make_tuple(static_cast<float>(new_x), 
                static_cast<float>(new_y), 
                static_cast<float>(new_z));

                
                if (navigableVerticesMap.find(index1) != navigableVerticesMap.end())
                { 
                    
                    adjacency_list[start_index].push_back(navigableVerticesMap[index1].key);
                    findNavigableVertice = true;
                 
                } 
            }

        }
    
        if(findNavigableVertice == false) 
        {
            RCLCPP_WARN(this->get_logger(), "The robot is too far of the navigable area.");
            return {};
        }
        

        /*
            Criando um vértice na posição do destino.
        */
       

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
        float new_x1 = 0.0, new_y1 = 0.0, new_z1 = 0.0;



        for(int i = 1; i <= 2; i++)
        {
            for (int a = 0; a < 26; a++) 
            {
                new_x1 = roundToMultiple(navigableVerticesMap[goalIndex].x + (offsets1[a][0] * i), distanceToObstacle_, decimals);
                new_y1 = roundToMultiple(navigableVerticesMap[goalIndex].y + (offsets1[a][1] * i), distanceToObstacle_, decimals);
                new_z1 = roundToMultipleFromBase(navigableVerticesMap[goalIndex].z + (offsets1[a][2] * i), roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);  

                auto index2 = std::make_tuple(static_cast<float>(new_x1), 
                static_cast<float>(new_y1), 
                static_cast<float>(new_z1));

                
                if (navigableVerticesMap.find(index2) != navigableVerticesMap.end())
                { 
                    adjacency_list[navigableVerticesMap[index2].key].push_back(end_index);
                    findNavigableGoalVertice = true;
                } 
            }

            
        }

        if(findNavigableGoalVertice == false)
        {
            RCLCPP_WARN(this->get_logger(), "Destination is too far of the navigable area. Increase navigable area.");
            return {};   
        }

        float distance = 0.0;
        auto heuristic = [&](int a, int b) {
            const auto &pa = navigableVerticesMapInteger[a];
            const auto &pb = navigableVerticesMapInteger[b];
        
            
            distance = std::sqrt(std::pow(pa.x - pb.x, 2) + std::pow(pa.y - pb.y, 2) + std::pow(pa.z - pb.z, 2));

            return distance;
        };

        

    
        g_score[start_index] = 0;
        f_score[start_index] = heuristic(start_index, end_index);


        std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, std::greater<> > open_set;
        open_set.push({f_score[start_index], start_index});

        while (!open_set.empty()) 
        {
            auto current_pair = open_set.top();
            open_set.pop();
            int current = current_pair.second;
            if(current != start_index && current != end_index)
            {
                
                for (int a = 0; a < 26; a++) 
                {
                    new_x = roundToMultiple(navigableVerticesMapInteger[current].x + offsets1[a][0], distanceToObstacle_, decimals);
                    new_y = roundToMultiple(navigableVerticesMapInteger[current].y + offsets1[a][1], distanceToObstacle_, decimals);
                    new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z + offsets1[a][2], roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);  
    
                    auto index1 = std::make_tuple(static_cast<float>(new_x), 
                    static_cast<float>(new_y), 
                    static_cast<float>(new_z));
    
                    
                    if (navigableVerticesMap.find(index1) != navigableVerticesMap.end())
                    { 
                        
                        adjacency_list[current].push_back(navigableVerticesMap[index1].key); 
                        
                    
                    } 
                }

                int i = 2;
                bool pode1 = true, pode2 = true, pode3 = true, pode4 = true, pode5 = true, pode6 = true, pode7 = true, pode8 = true;
                /*
                
                
                    SIM, EU VOU REDUZIR A QUANTIDADE ABSURDA DE CÓDIGO QUE TEM AQUI.
                
                
                */
                while(i <= diagonalEdges_)
                {
                   
                    if(i > 1 || i < -1)
                    {
                   
                        new_x = roundToMultiple(navigableVerticesMapInteger[current].x + (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(navigableVerticesMapInteger[current].y + distanceToObstacle_, distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        auto index = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        new_x = roundToMultiple(navigableVerticesMapInteger[current].x + (distanceToObstacle_* (i - 1)), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(navigableVerticesMapInteger[current].y + distanceToObstacle_, distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        auto index1 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));
                        
                        new_x = roundToMultiple(navigableVerticesMapInteger[current].x + (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(navigableVerticesMapInteger[current].y, distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        auto index2 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        new_x = roundToMultiple(navigableVerticesMapInteger[current].x + (distanceToObstacle_ * (i - 1)), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(navigableVerticesMapInteger[current].y, distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        auto index3 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));
                        if(navigableVerticesMap.find(index) == navigableVerticesMap.end() || navigableVerticesMap.find(index1) == navigableVerticesMap.end() || navigableVerticesMap.find(index2) == navigableVerticesMap.end() || navigableVerticesMap.find(index3) == navigableVerticesMap.end())
                        {
                            pode1 = false;
                        }

                        if(navigableVerticesMap.find(index) != navigableVerticesMap.end() && navigableVerticesMap.find(index1) != navigableVerticesMap.end() && navigableVerticesMap.find(index2) != navigableVerticesMap.end() && navigableVerticesMap.find(index3) != navigableVerticesMap.end())
                        {
                            if(pode1 == true)
                            {
                                adjacency_list[current].push_back(navigableVerticesMap[index].key);   
                            }
                        }
                        

                        new_x = roundToMultiple(navigableVerticesMapInteger[current].x + (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(navigableVerticesMapInteger[current].y - distanceToObstacle_, distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        new_x = roundToMultiple(navigableVerticesMapInteger[current].x + (distanceToObstacle_* (i - 1)), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(navigableVerticesMapInteger[current].y - distanceToObstacle_, distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index1 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));
                        
                        new_x = roundToMultiple(navigableVerticesMapInteger[current].x + (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(navigableVerticesMapInteger[current].y, distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index2 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        new_x = roundToMultiple(navigableVerticesMapInteger[current].x + (distanceToObstacle_ * (i - 1)), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(navigableVerticesMapInteger[current].y, distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index3 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));
                        if(navigableVerticesMap.find(index) == navigableVerticesMap.end() || navigableVerticesMap.find(index1) == navigableVerticesMap.end() || navigableVerticesMap.find(index2) == navigableVerticesMap.end() || navigableVerticesMap.find(index3) == navigableVerticesMap.end())
                        {
                            pode2 = false;
                        }

                        if(navigableVerticesMap.find(index) != navigableVerticesMap.end() && navigableVerticesMap.find(index1) != navigableVerticesMap.end() && navigableVerticesMap.find(index2) != navigableVerticesMap.end() && navigableVerticesMap.find(index3) != navigableVerticesMap.end())
                        {
                            if(pode2 == true)
                            {
                                adjacency_list[current].push_back(navigableVerticesMap[index].key); 
                            }
                         
                                
                        }


                        

                        new_x = roundToMultiple(navigableVerticesMapInteger[current].x - (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(navigableVerticesMapInteger[current].y + distanceToObstacle_, distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        new_x = roundToMultiple(navigableVerticesMapInteger[current].x + (distanceToObstacle_* (i - 1)), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(navigableVerticesMapInteger[current].y + distanceToObstacle_, distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index1 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));
                        
                        new_x = roundToMultiple(navigableVerticesMapInteger[current].x + (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(navigableVerticesMapInteger[current].y, distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index2 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        new_x = roundToMultiple(navigableVerticesMapInteger[current].x + (distanceToObstacle_ * (i - 1)), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(navigableVerticesMapInteger[current].y, distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index3 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));
                        if(navigableVerticesMap.find(index) == navigableVerticesMap.end() || navigableVerticesMap.find(index1) == navigableVerticesMap.end() || navigableVerticesMap.find(index2) == navigableVerticesMap.end() || navigableVerticesMap.find(index3) == navigableVerticesMap.end())
                        {
                            pode3 = false;
                        }

                        if(navigableVerticesMap.find(index) != navigableVerticesMap.end() && navigableVerticesMap.find(index1) != navigableVerticesMap.end() && navigableVerticesMap.find(index2) != navigableVerticesMap.end() && navigableVerticesMap.find(index3) != navigableVerticesMap.end())
                        {
                            if(pode3 == true)
                            {
                                adjacency_list[current].push_back(navigableVerticesMap[index].key);   
                            }
                        }
                        

                        new_x = roundToMultiple(navigableVerticesMapInteger[current].x - (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(navigableVerticesMapInteger[current].y - distanceToObstacle_, distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        new_x = roundToMultiple(navigableVerticesMapInteger[current].x - (distanceToObstacle_* (i - 1)), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(navigableVerticesMapInteger[current].y - distanceToObstacle_, distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index1 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));
                        
                        new_x = roundToMultiple(navigableVerticesMapInteger[current].x - (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(navigableVerticesMapInteger[current].y, distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index2 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        new_x = roundToMultiple(navigableVerticesMapInteger[current].x - (distanceToObstacle_ * (i - 1)), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(navigableVerticesMapInteger[current].y, distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index3 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));
                        if(navigableVerticesMap.find(index) == navigableVerticesMap.end() || navigableVerticesMap.find(index1) == navigableVerticesMap.end() || navigableVerticesMap.find(index2) == navigableVerticesMap.end() || navigableVerticesMap.find(index3) == navigableVerticesMap.end())
                        {
                            pode4 = false;
                        }

                        if(navigableVerticesMap.find(index) != navigableVerticesMap.end() && navigableVerticesMap.find(index1) != navigableVerticesMap.end() && navigableVerticesMap.find(index2) != navigableVerticesMap.end() && navigableVerticesMap.find(index3) != navigableVerticesMap.end())
                        {
                            if(pode4 == true)
                            {
                                adjacency_list[current].push_back(navigableVerticesMap[index].key); 
                            }
                         
                                
                        }


    

                        new_x = roundToMultiple(navigableVerticesMapInteger[current].x + distanceToObstacle_, distanceToObstacle_, decimals);
                        new_y = roundToMultiple(navigableVerticesMapInteger[current].y + (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        
                        new_x = roundToMultiple(navigableVerticesMapInteger[current].x + distanceToObstacle_, distanceToObstacle_, decimals);
                        new_y = roundToMultiple(navigableVerticesMapInteger[current].y + (distanceToObstacle_ * (i - 1) ), distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index1 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        new_x = roundToMultiple(navigableVerticesMapInteger[current].x, distanceToObstacle_, decimals);
                        new_y = roundToMultiple(navigableVerticesMapInteger[current].y + (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index2 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        new_x = roundToMultiple(navigableVerticesMapInteger[current].x, distanceToObstacle_, decimals);
                        new_y = roundToMultiple(navigableVerticesMapInteger[current].y + (distanceToObstacle_ * (i - 1)), distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index3 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));
                    
                        if(navigableVerticesMap.find(index) == navigableVerticesMap.end() || navigableVerticesMap.find(index1) == navigableVerticesMap.end() || navigableVerticesMap.find(index2) == navigableVerticesMap.end() || navigableVerticesMap.find(index3) == navigableVerticesMap.end())
                        {
                            pode5 = false;
                        }


                        if(navigableVerticesMap.find(index) != navigableVerticesMap.end() && navigableVerticesMap.find(index1) != navigableVerticesMap.end() && navigableVerticesMap.find(index2) != navigableVerticesMap.end() && navigableVerticesMap.find(index3) != navigableVerticesMap.end())
                        {
                            if(pode5 == true)
                            {
                                adjacency_list[current].push_back(navigableVerticesMap[index].key); 
                            }
                            
                        }
                        
                        



                        new_x = roundToMultiple(navigableVerticesMapInteger[current].x - distanceToObstacle_, distanceToObstacle_, decimals);
                        new_y = roundToMultiple(navigableVerticesMapInteger[current].y + (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        
                        new_x = roundToMultiple(navigableVerticesMapInteger[current].x - distanceToObstacle_, distanceToObstacle_, decimals);
                        new_y = roundToMultiple(navigableVerticesMapInteger[current].y + (distanceToObstacle_ * (i - 1) ), distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index1 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        new_x = roundToMultiple(navigableVerticesMapInteger[current].x, distanceToObstacle_, decimals);
                        new_y = roundToMultiple(navigableVerticesMapInteger[current].y + (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index2 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        new_x = roundToMultiple(navigableVerticesMapInteger[current].x, distanceToObstacle_, decimals);
                        new_y = roundToMultiple(navigableVerticesMapInteger[current].y + (distanceToObstacle_ * (i - 1)), distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index3 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));
                    

                        if(navigableVerticesMap.find(index) == navigableVerticesMap.end() || navigableVerticesMap.find(index1) == navigableVerticesMap.end() || navigableVerticesMap.find(index2) == navigableVerticesMap.end() || navigableVerticesMap.find(index3) == navigableVerticesMap.end())
                        {
                            pode6 = false;
                        }

                        if(navigableVerticesMap.find(index) != navigableVerticesMap.end() && navigableVerticesMap.find(index1) != navigableVerticesMap.end() && navigableVerticesMap.find(index2) != navigableVerticesMap.end() && navigableVerticesMap.find(index3) != navigableVerticesMap.end())
                        {       
                            if(pode6 == true)
                            {
                                adjacency_list[current].push_back(navigableVerticesMap[index].key); 
                            }
                        }



                        new_x = roundToMultiple(navigableVerticesMapInteger[current].x + distanceToObstacle_, distanceToObstacle_, decimals);
                        new_y = roundToMultiple(navigableVerticesMapInteger[current].y - (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        
                        new_x = roundToMultiple(navigableVerticesMapInteger[current].x + distanceToObstacle_, distanceToObstacle_, decimals);
                        new_y = roundToMultiple(navigableVerticesMapInteger[current].y - (distanceToObstacle_ * (i - 1) ), distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index1 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        new_x = roundToMultiple(navigableVerticesMapInteger[current].x, distanceToObstacle_, decimals);
                        new_y = roundToMultiple(navigableVerticesMapInteger[current].y - (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index2 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        new_x = roundToMultiple(navigableVerticesMapInteger[current].x, distanceToObstacle_, decimals);
                        new_y = roundToMultiple(navigableVerticesMapInteger[current].y - (distanceToObstacle_ * (i - 1)), distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index3 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));
                    
                        if(navigableVerticesMap.find(index) == navigableVerticesMap.end() || navigableVerticesMap.find(index1) == navigableVerticesMap.end() || navigableVerticesMap.find(index2) == navigableVerticesMap.end() || navigableVerticesMap.find(index3) == navigableVerticesMap.end())
                        {
                            pode7 = false;
                        }


                        if(navigableVerticesMap.find(index) != navigableVerticesMap.end() && navigableVerticesMap.find(index1) != navigableVerticesMap.end() && navigableVerticesMap.find(index2) != navigableVerticesMap.end() && navigableVerticesMap.find(index3) != navigableVerticesMap.end())
                        {
                            if(pode7 == true)
                            {
                                adjacency_list[current].push_back(navigableVerticesMap[index].key); 
                            }
                            
                        }
                        
                        



                        new_x = roundToMultiple(navigableVerticesMapInteger[current].x - distanceToObstacle_, distanceToObstacle_, decimals);
                        new_y = roundToMultiple(navigableVerticesMapInteger[current].y - (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        
                        new_x = roundToMultiple(navigableVerticesMapInteger[current].x - distanceToObstacle_, distanceToObstacle_, decimals);
                        new_y = roundToMultiple(navigableVerticesMapInteger[current].y - (distanceToObstacle_ * (i - 1) ), distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index1 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        new_x = roundToMultiple(navigableVerticesMapInteger[current].x, distanceToObstacle_, decimals);
                        new_y = roundToMultiple(navigableVerticesMapInteger[current].y - (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index2 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        new_x = roundToMultiple(navigableVerticesMapInteger[current].x, distanceToObstacle_, decimals);
                        new_y = roundToMultiple(navigableVerticesMapInteger[current].y - (distanceToObstacle_ * (i - 1)), distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index3 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));
                    

                        if(navigableVerticesMap.find(index) == navigableVerticesMap.end() || navigableVerticesMap.find(index1) == navigableVerticesMap.end() || navigableVerticesMap.find(index2) == navigableVerticesMap.end() || navigableVerticesMap.find(index3) == navigableVerticesMap.end())
                        {
                            pode8 = false;
                        }

                        if(navigableVerticesMap.find(index) != navigableVerticesMap.end() && navigableVerticesMap.find(index1) != navigableVerticesMap.end() && navigableVerticesMap.find(index2) != navigableVerticesMap.end() && navigableVerticesMap.find(index3) != navigableVerticesMap.end())
                        {       
                            if(pode8 == true)
                            {
                                adjacency_list[current].push_back(navigableVerticesMap[index].key); 
                            }
                        }
                        



                    }
                    i++;
                }

              






            
            }

            
            
            if (closed_set.find(current) != closed_set.end())
                continue;
                
          
            if (current_pair.first > f_score[current])
                continue;
                
           
            closed_set.insert(current);
            
            if (current == end_index) 
            {
               
                return reconstructPath(came_from, current);
            } 
            
          

            for (const auto& neighbor : adjacency_list[current])
            {
                if (closed_set.find(neighbor) != closed_set.end())
                continue;
           
                float tentative_g_score = g_score[current] + heuristic(current, neighbor);
                if (g_score.find(neighbor) == g_score.end() || tentative_g_score < g_score[neighbor]) 
                {
                    came_from[neighbor] = current;
                    g_score[neighbor] = tentative_g_score;
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, end_index);
                    open_set.push({f_score[neighbor], neighbor});
                }
            }

           
    
                
            adjacency_list.erase(current);
        
        
        }
        

        RCLCPP_WARN(this->get_logger(), "Não foi possível alcançar o destino.");
        return {};
    }

    std::vector<int> reconstructPath(const std::unordered_map<int, int> &came_from, int current) 
    {
        std::vector<int> path;
        while (came_from.find(current) != came_from.end()) 
        {
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

               

                
                if (i < path.size() - 1) {
                    const Vertex &current_vertex = navigableVerticesMapInteger[path[i]];
                    const Vertex &next_vertex = navigableVerticesMapInteger[path[i + 1]];

                   
                    float dx = next_vertex.x - current_vertex.x;
                    float dy = next_vertex.y - current_vertex.y;
                    float dz = next_vertex.z - current_vertex.z;
                    float distance = std::sqrt(dx * dx + dy * dy + dz * dz);

                    
                    if (distance > 0.0f) {
                        dx /= distance;
                        dy /= distance;
                        dz /= distance;
                    }

                    Eigen::Vector3f direction(dx, dy, dz);
                    Eigen::Vector3f reference(1.0f, 0.0f, 0.0f);

                    
                    Eigen::Quaternionf quaternion = Eigen::Quaternionf::FromTwoVectors(reference, direction);

                    vertex.orientation_x = quaternion.x();
                    vertex.orientation_y = quaternion.y();
                    vertex.orientation_z = quaternion.z();
                    vertex.orientation_w = quaternion.w();
                } 
                else 
                {
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
            float dx = pose_x_ - static_cast<float>(verticesDestino_[i_].x);
            float dy = pose_y_ - static_cast<float>(verticesDestino_[i_].y);
            float dz = pose_z_ - static_cast<float>(verticesDestino_[i_].z);

            float distanciaAteODestino = sqrt(dx * dx + dy * dy + dz * dz);

            if(distanciaAteODestino <= distanceToObstacle_)
            {
                i_ = i_ + 1;
               // adjacency_list.erase(navigableVerticesMap[globalGoalIndex].key);
                navigableVerticesMapInteger.erase(navigableVerticesMap[globalGoalIndex].key);
                navigableVerticesMap.erase(globalGoalIndex);
            
            }

       
            float array_inicial[3] = {pose_x_, pose_y_, pose_z_};
            float array_final[3] = {static_cast<float>(verticesDestino_[i_].x), static_cast<float>(verticesDestino_[i_].y), static_cast<float>(verticesDestino_[i_].z)};
            
            if(i_ == verticesDestino_.size())
            {
                i_ = 0;
            }

            

            auto start_time_ = std::chrono::high_resolution_clock::now();
            std::vector<int> shortestPath = runAStar(array_inicial, array_final);
           
            storeEdgesInPath(shortestPath);

            //adjacency_list.erase(navigableVerticesMap[globalIndex].key);            
            navigableVerticesMapInteger.erase(navigableVerticesMap[globalIndex].key);
            navigableVerticesMap.erase(globalIndex);

            navigableVerticesMapInteger.erase(navigableVerticesMap[globalGoalIndex].key);
            navigableVerticesMap.erase(globalGoalIndex);
            
           
            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<float> duration = end_time - start_time_;        
            
            RCLCPP_INFO(this->get_logger(), "A* execution time: %.10f", duration.count());
       
        }
    }

    void callback_removed_navigable_vertices(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
   
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

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

            if (navigableVerticesMap.find(index) != navigableVerticesMap.end()) 
            {
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
      
        auto new_distanceToObstacle = static_cast<float>(this->get_parameter("distanceToObstacle").get_parameter_value().get<double>());
        auto new_diagonalEdges = this->get_parameter("diagonalEdges").get_parameter_value().get<int>();
        auto new_x_min = static_cast<float>(this->get_parameter("x_min").get_parameter_value().get<double>());
        auto new_x_max = static_cast<float>(this->get_parameter("x_max").get_parameter_value().get<double>());
        auto new_y_min = static_cast<float>(this->get_parameter("y_min").get_parameter_value().get<double>());
        auto new_y_max = static_cast<float>(this->get_parameter("y_max").get_parameter_value().get<double>());
        auto new_z_min = static_cast<float>(this->get_parameter("z_min").get_parameter_value().get<double>());
        auto new_z_max = static_cast<float>(this->get_parameter("z_max").get_parameter_value().get<double>());
        
        
        if (new_distanceToObstacle != distanceToObstacle_) 
        {
            distanceToObstacle_ = new_distanceToObstacle;
            resolution_ = 1;
            std::cout << "\n" << std::endl;
            RCLCPP_INFO(this->get_logger(), "Updated DistanceToObstacle: %.2f", distanceToObstacle_);
            RCLCPP_INFO(this->get_logger(), "Resolution set to 1.");
            createGraph();           
        }
        if(new_diagonalEdges != diagonalEdges_)
        {
            diagonalEdges_ = new_diagonalEdges;

            std::cout << "\n" << std::endl;

            RCLCPP_INFO(this->get_logger(), "Updated diagonalEdges: %d", diagonalEdges_);
            createGraph();
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
     : Node("a_star")
    {
    
     
        this->declare_parameter<double>("distanceToObstacle", 0.05);
        this->declare_parameter<int>("diagonalEdges", 1);
        this->declare_parameter<double>("x_min", -10.0);
        this->declare_parameter<double>("x_max", 10.0);
        this->declare_parameter<double>("y_min", -10.0);
        this->declare_parameter<double>("y_max", 10.0);
        this->declare_parameter<double>("z_min", 0.2);
        this->declare_parameter<double>("z_max", 0.2);

        // Initialize parameters 
        distanceToObstacle_ =  static_cast<float>(this->get_parameter("distanceToObstacle").get_parameter_value().get<double>());
        diagonalEdges_ = this->get_parameter("diagonalEdges").get_parameter_value().get<int>();
        x_min_ = static_cast<float>(this->get_parameter("x_min").get_parameter_value().get<double>());
        x_max_ = static_cast<float>(this->get_parameter("x_max").get_parameter_value().get<double>());
        y_min_ = static_cast<float>(this->get_parameter("y_min").get_parameter_value().get<double>());
        y_max_ = static_cast<float>(this->get_parameter("y_max").get_parameter_value().get<double>());
        z_min_ = static_cast<float>(this->get_parameter("z_min").get_parameter_value().get<double>());
        z_max_ = static_cast<float>(this->get_parameter("z_max").get_parameter_value().get<double>());

        RCLCPP_INFO(this->get_logger(), "Updated DistanceToObstacle: %f", distanceToObstacle_);
        RCLCPP_INFO(this->get_logger(), "Updated diagonalEdges: %d", diagonalEdges_);
        RCLCPP_INFO(this->get_logger(), "Resolution is set to: %0.f", resolution_);
        RCLCPP_INFO(this->get_logger(), "Updated x_min: %f", x_min_);
        RCLCPP_INFO(this->get_logger(), "Updated x_max: %f", x_max_);
        RCLCPP_INFO(this->get_logger(), "Updated y_min: %f", y_min_);
        RCLCPP_INFO(this->get_logger(), "Updated y_max: %f", y_max_);
        RCLCPP_INFO(this->get_logger(), "Updated z_min: %f", z_min_);
        RCLCPP_INFO(this->get_logger(), "Updated z_max: %f", z_max_);


        parameterTimer = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&AStar::check_parameters, this));

        decimals = countDecimals(distanceToObstacle_);
       
 
        subscription_navigable_removed_vertices = this->create_subscription<sensor_msgs::msg::PointCloud2>(
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