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
    

    
    std::unordered_map<int, std::vector<int>> adjacency_list;
    //std::unordered_map<int, std::unordered_set<std::pair<int, int>, PairHash>> adjacency_list;
    std::unordered_set<std::tuple<float, float, float>> obstaclesVertices;
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
    
    
   

   
    std::vector<int> runAStar(float start[3], float goal[3]) 
    {
        
        destinationEdges.clear();
        std::unordered_map<int, int> came_from;
        std::unordered_map<int, float> g_score;
        std::unordered_map<int, float> f_score;
        std::unordered_set<int> closed_set;
       
        auto offsets1 = getOffsets(distanceToObstacle_);
        int start_index = 0, end_index = 1;
        int k = 2;

        Vertex vStart;
        vStart.key = start_index;
        vStart.x = start[0];
        vStart.y = start[1];
        vStart.z = start[2];

        auto index = std::make_tuple(vStart.x, vStart.y, vStart.z);
        
        
        navigableVerticesMap[index] = vStart;
        navigableVerticesMapInteger[start_index] = vStart;

      
        float new_x = 0.0, new_y = 0.0, new_z = 0.0;
        bool findNavigableVertice = false;

        
        for(int i = 1; i <= 2; i++)
        {
            for (int a = 0; a < 26; a++) 
            {
                new_x = roundToMultiple(navigableVerticesMapInteger[start_index].x + (offsets1[a][0] * i), distanceToObstacle_, decimals);
                new_y = roundToMultiple(navigableVerticesMapInteger[start_index].y + (offsets1[a][1] * i), distanceToObstacle_, decimals);
                new_z = roundToMultipleFromBase(navigableVerticesMapInteger[start_index].z + (offsets1[a][2] * i), roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);  

                auto index1 = std::make_tuple(static_cast<float>(new_x), 
                static_cast<float>(new_y), 
                static_cast<float>(new_z));

                
                if (obstaclesVertices.find(index1) == obstaclesVertices.end())
                { 
                    
                    if(navigableVerticesMap.find(index1) == navigableVerticesMap.end())
                    {

                        navigableVerticesMap[index1] = {k, new_x, new_y, new_z};

                        navigableVerticesMapInteger[navigableVerticesMap[index1].key] = {k, new_x, new_y, new_z};
                        
                        adjacency_list[start_index].push_back(navigableVerticesMap[index1].key);
                        
                        k++;
                        
                        findNavigableVertice = true;
                    }
                    else
                    {
                 
                        adjacency_list[start_index].push_back(navigableVerticesMap[index1].key);

                        findNavigableVertice = true;
                    }
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
        vGoal.key = end_index;
        vGoal.x = goal[0];
        vGoal.y = goal[1];
        vGoal.z = goal[2];

        
        auto index4 = std::make_tuple(vStart.x, vStart.y, vStart.z);
        
        
        navigableVerticesMap[index4] = vGoal;
        navigableVerticesMapInteger[end_index] = vGoal;
        
        bool findNavigableGoalVertice = false;
        
        float new_x1 = 0.0, new_y1 = 0.0, new_z1 = 0.0;
        


        for(int i = 1; i <= 2; i++)
        {
            for (int a = 0; a < 26; a++) 
            {
                new_x1 = roundToMultiple(navigableVerticesMapInteger[end_index].x + (offsets1[a][0] * i), distanceToObstacle_, decimals);
                new_y1 = roundToMultiple(navigableVerticesMapInteger[end_index].y + (offsets1[a][1] * i), distanceToObstacle_, decimals);
                new_z1 = roundToMultipleFromBase(navigableVerticesMapInteger[end_index].z + (offsets1[a][2] * i), roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);  

                auto index2 = std::make_tuple(static_cast<float>(new_x1), 
                static_cast<float>(new_y1), 
                static_cast<float>(new_z1));

                
                if (obstaclesVertices.find(index2) == obstaclesVertices.end())
                { 
                    if(navigableVerticesMap.find(index2) == navigableVerticesMap.end())
                    {

                        navigableVerticesMap[index2] = {k, new_x1, new_y1, new_z1};

                        navigableVerticesMapInteger[navigableVerticesMap[index2].key] = {k, new_x1, new_y1, new_z1};
                        
                        adjacency_list[navigableVerticesMap[index2].key].push_back(end_index);
                        
                        k++;
                        
                        findNavigableGoalVertice = true;
                    }
                    else
                    {
                  
                        adjacency_list[navigableVerticesMap[index2].key].push_back(end_index);

                        findNavigableGoalVertice = true;
                    }

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
    
                   
                    if (obstaclesVertices.find(index1) == obstaclesVertices.end())
                    {
                       
                        if(navigableVerticesMap.find(index1) == navigableVerticesMap.end())
                        {
                            
                            navigableVerticesMap[index1] = {k, new_x, new_y, new_z};

                            navigableVerticesMapInteger[navigableVerticesMap[index1].key] = {k, new_x, new_y, new_z};

                            adjacency_list[current].push_back(navigableVerticesMap[index1].key);
                            
                            k++;
                            
                          
                        }
                        else
                        {
                                                    
                            adjacency_list[current].push_back(navigableVerticesMap[index1].key);

                        }

     
                    } 


               
                /*
                
                
                    SIM, EU VOU REDUZIR A QUANTIDADE ABSURDA DE CÓDIGO QUE TEM AQUI.
                
                
                */
               
              

                    
                }
                
                int i = 2;
                float new_x2 = 0.0, new_y2 = 0.0, new_z2 = 0.0;
                bool pode1 = true, pode2 = true, pode3 = true, pode4 = true, pode5 = true, pode6 = true, pode7 = true, pode8 = true;
                
                while(i <= diagonalEdges_)
                {
                   
                    if(i > 1 || i < -1)
                    {
                   
                        new_x2 = roundToMultiple(navigableVerticesMapInteger[current].x + (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_y2 = roundToMultiple(navigableVerticesMapInteger[current].y + distanceToObstacle_, distanceToObstacle_, decimals);
                        new_z2 = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        auto index = std::make_tuple(static_cast<float>(new_x2), static_cast<float>(new_y2), static_cast<float>(new_z2));

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
                        if(obstaclesVertices.find(index) != obstaclesVertices.end() || obstaclesVertices.find(index1) != obstaclesVertices.end() || obstaclesVertices.find(index2) != obstaclesVertices.end() || obstaclesVertices.find(index3) != obstaclesVertices.end())
                        {
                            pode1 = false;
                        }

                        if(obstaclesVertices.find(index) == obstaclesVertices.end() && obstaclesVertices.find(index1) == obstaclesVertices.end() && obstaclesVertices.find(index2) == obstaclesVertices.end() && obstaclesVertices.find(index3) == obstaclesVertices.end())
                        {
                            if(pode1 == true)
                            {
                                if(navigableVerticesMap.find(index) == navigableVerticesMap.end())
                                {
                                    
                                    navigableVerticesMap[index] = {k, new_x2, new_y2, new_z2};
        
                                    navigableVerticesMapInteger[navigableVerticesMap[index].key] = {k, new_x2, new_y2, new_z2};
        
                                    adjacency_list[current].push_back(navigableVerticesMap[index].key);
                                    
                                    k++;
                                    
                                  
                                }
                                else
                                {
                                                            
                                    adjacency_list[current].push_back(navigableVerticesMap[index].key);
        
                                }   
                            }
                        }
                        

                        new_x2 = roundToMultiple(navigableVerticesMapInteger[current].x + (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_y2 = roundToMultiple(navigableVerticesMapInteger[current].y - distanceToObstacle_, distanceToObstacle_, decimals);
                        new_z2 = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index = std::make_tuple(static_cast<float>(new_x2), static_cast<float>(new_y2), static_cast<float>(new_z2));

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
                        if(obstaclesVertices.find(index) != obstaclesVertices.end() || obstaclesVertices.find(index1) != obstaclesVertices.end() || obstaclesVertices.find(index2) != obstaclesVertices.end() || obstaclesVertices.find(index3) != obstaclesVertices.end())
                        {
                            pode2 = false;
                        }

                        if(obstaclesVertices.find(index) == obstaclesVertices.end() && obstaclesVertices.find(index1) == obstaclesVertices.end() && obstaclesVertices.find(index2) == obstaclesVertices.end() && obstaclesVertices.find(index3) == obstaclesVertices.end())
                        {
                            if(pode2 == true)
                            {
                                if(navigableVerticesMap.find(index) == navigableVerticesMap.end())
                                {
                                    
                                    navigableVerticesMap[index] = {k, new_x2, new_y2, new_z2};
        
                                    navigableVerticesMapInteger[navigableVerticesMap[index].key] = {k, new_x2, new_y2, new_z2};
        
                                    adjacency_list[current].push_back(navigableVerticesMap[index].key);
                                    
                                    k++;
                                    
                                  
                                }
                                else
                                {
                                                            
                                    adjacency_list[current].push_back(navigableVerticesMap[index].key);
        
                                }   
                            }
                         
                                
                        }


                        

                        new_x2 = roundToMultiple(navigableVerticesMapInteger[current].x - (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_y2 = roundToMultiple(navigableVerticesMapInteger[current].y + distanceToObstacle_, distanceToObstacle_, decimals);
                        new_z2 = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index = std::make_tuple(static_cast<float>(new_x2), static_cast<float>(new_y2), static_cast<float>(new_z2));

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
                        if(obstaclesVertices.find(index) != obstaclesVertices.end() || obstaclesVertices.find(index1) != obstaclesVertices.end() || obstaclesVertices.find(index2) != obstaclesVertices.end() || obstaclesVertices.find(index3) != obstaclesVertices.end())
                        {
                            pode3 = false;
                        }

                        if(obstaclesVertices.find(index) == obstaclesVertices.end() && obstaclesVertices.find(index1) == obstaclesVertices.end() && obstaclesVertices.find(index2) == obstaclesVertices.end() && obstaclesVertices.find(index3) == obstaclesVertices.end())
                        {
                            if(pode3 == true)
                            {
                                if(navigableVerticesMap.find(index) == navigableVerticesMap.end())
                                {
                                    
                                    navigableVerticesMap[index] = {k, new_x2, new_y2, new_z2};
        
                                    navigableVerticesMapInteger[navigableVerticesMap[index].key] = {k, new_x2, new_y2, new_z2};
        
                                    adjacency_list[current].push_back(navigableVerticesMap[index].key);
                                    
                                    k++;
                                    
                                  
                                }
                                else
                                {
                                                            
                                    adjacency_list[current].push_back(navigableVerticesMap[index].key);
        
                                }   
                            }
                        }
                        

                        new_x2 = roundToMultiple(navigableVerticesMapInteger[current].x - (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_y2 = roundToMultiple(navigableVerticesMapInteger[current].y - distanceToObstacle_, distanceToObstacle_, decimals);
                        new_z2 = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index = std::make_tuple(static_cast<float>(new_x2), static_cast<float>(new_y2), static_cast<float>(new_z2));

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
                        if(obstaclesVertices.find(index) != obstaclesVertices.end() || obstaclesVertices.find(index1) != obstaclesVertices.end() || obstaclesVertices.find(index2) != obstaclesVertices.end() || obstaclesVertices.find(index3) != obstaclesVertices.end())
                        {
                            pode4 = false;
                        }

                        if(obstaclesVertices.find(index) == obstaclesVertices.end() && obstaclesVertices.find(index1) == obstaclesVertices.end() && obstaclesVertices.find(index2) == obstaclesVertices.end() && obstaclesVertices.find(index3) == obstaclesVertices.end())
                        {
                            if(pode4 == true)
                            {
                                if(navigableVerticesMap.find(index) == navigableVerticesMap.end())
                                {
                                    
                                    navigableVerticesMap[index] = {k, new_x2, new_y2, new_z2};
        
                                    navigableVerticesMapInteger[navigableVerticesMap[index].key] = {k, new_x2, new_y2, new_z2};
        
                                    adjacency_list[current].push_back(navigableVerticesMap[index].key);
                                    
                                    k++;
                                    
                                  
                                }
                                else
                                {
                                                            
                                    adjacency_list[current].push_back(navigableVerticesMap[index].key);
        
                                }   
                            }
                         
                                
                        }


    

                        new_x2 = roundToMultiple(navigableVerticesMapInteger[current].x + distanceToObstacle_, distanceToObstacle_, decimals);
                        new_y2 = roundToMultiple(navigableVerticesMapInteger[current].y + (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_z2 = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index = std::make_tuple(static_cast<float>(new_x2), static_cast<float>(new_y2), static_cast<float>(new_z2));

                        
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
                    
                        if(obstaclesVertices.find(index) != obstaclesVertices.end() || obstaclesVertices.find(index1) != obstaclesVertices.end() || obstaclesVertices.find(index2) != obstaclesVertices.end() || obstaclesVertices.find(index3) != obstaclesVertices.end())
                        {
                            pode5 = false;
                        }


                        if(obstaclesVertices.find(index) == obstaclesVertices.end() && obstaclesVertices.find(index1) == obstaclesVertices.end() && obstaclesVertices.find(index2) == obstaclesVertices.end() && obstaclesVertices.find(index3) == obstaclesVertices.end())
                        {
                            if(pode5 == true)
                            {
                                if(navigableVerticesMap.find(index) == navigableVerticesMap.end())
                                {
                                    
                                    navigableVerticesMap[index] = {k, new_x2, new_y2, new_z2};
        
                                    navigableVerticesMapInteger[navigableVerticesMap[index].key] = {k, new_x2, new_y2, new_z2};
        
                                    adjacency_list[current].push_back(navigableVerticesMap[index].key);
                                    
                                    k++;
                                    
                                  
                                }
                                else
                                {
                                                            
                                    adjacency_list[current].push_back(navigableVerticesMap[index].key);
        
                                }   
                            }
                            
                        }
                        
                        



                        new_x2 = roundToMultiple(navigableVerticesMapInteger[current].x - distanceToObstacle_, distanceToObstacle_, decimals);
                        new_y2 = roundToMultiple(navigableVerticesMapInteger[current].y + (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_z2 = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index = std::make_tuple(static_cast<float>(new_x2), static_cast<float>(new_y2), static_cast<float>(new_z2));

                        
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
                    

                        if(obstaclesVertices.find(index) != obstaclesVertices.end() || obstaclesVertices.find(index1) != obstaclesVertices.end() || obstaclesVertices.find(index2) != obstaclesVertices.end() || obstaclesVertices.find(index3) != obstaclesVertices.end())
                        {
                            pode6 = false;
                        }

                        if(obstaclesVertices.find(index) == obstaclesVertices.end() && obstaclesVertices.find(index1) == obstaclesVertices.end() && obstaclesVertices.find(index2) == obstaclesVertices.end() && obstaclesVertices.find(index3) == obstaclesVertices.end())
                        {       
                            if(pode6 == true)
                            {
                                if(navigableVerticesMap.find(index) == navigableVerticesMap.end())
                                {
                                    
                                    navigableVerticesMap[index] = {k, new_x2, new_y2, new_z2};
        
                                    navigableVerticesMapInteger[navigableVerticesMap[index].key] = {k, new_x2, new_y2, new_z2};
        
                                    adjacency_list[current].push_back(navigableVerticesMap[index].key);
                                    
                                    k++;
                                    
                                  
                                }
                                else
                                {
                                                            
                                    adjacency_list[current].push_back(navigableVerticesMap[index].key);
        
                                }     
                            }
                        }



                        new_x2 = roundToMultiple(navigableVerticesMapInteger[current].x + distanceToObstacle_, distanceToObstacle_, decimals);
                        new_y2 = roundToMultiple(navigableVerticesMapInteger[current].y - (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_z2 = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index = std::make_tuple(static_cast<float>(new_x2), static_cast<float>(new_y2), static_cast<float>(new_z2));

                        
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
                    
                        if(obstaclesVertices.find(index) != obstaclesVertices.end() || obstaclesVertices.find(index1) != obstaclesVertices.end() || obstaclesVertices.find(index2) != obstaclesVertices.end() || obstaclesVertices.find(index3) != obstaclesVertices.end())
                        {
                            pode7 = false;
                        }


                        if(obstaclesVertices.find(index) == obstaclesVertices.end() && obstaclesVertices.find(index1) == obstaclesVertices.end() && obstaclesVertices.find(index2) == obstaclesVertices.end() && obstaclesVertices.find(index3) == obstaclesVertices.end())
                        {
                            if(pode7 == true)
                            {
                                if(navigableVerticesMap.find(index) == navigableVerticesMap.end())
                                {
                                    
                                    navigableVerticesMap[index] = {k, new_x2, new_y2, new_z2};
        
                                    navigableVerticesMapInteger[navigableVerticesMap[index].key] = {k, new_x2, new_y2, new_z2};
        
                                    adjacency_list[current].push_back(navigableVerticesMap[index].key);
                                    
                                    k++;
                                    
                                  
                                }
                                else
                                {
                                                            
                                    adjacency_list[current].push_back(navigableVerticesMap[index].key);
        
                                }   
                            }
                            
                        }
                        
                        



                        new_x2 = roundToMultiple(navigableVerticesMapInteger[current].x - distanceToObstacle_, distanceToObstacle_, decimals);
                        new_y2 = roundToMultiple(navigableVerticesMapInteger[current].y - (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_z2 = roundToMultipleFromBase(navigableVerticesMapInteger[current].z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        index = std::make_tuple(static_cast<float>(new_x2), static_cast<float>(new_y2), static_cast<float>(new_z2));

                        
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
                    

                        if(obstaclesVertices.find(index) != obstaclesVertices.end() || obstaclesVertices.find(index1) != obstaclesVertices.end() || obstaclesVertices.find(index2) != obstaclesVertices.end() || obstaclesVertices.find(index3) != obstaclesVertices.end())
                        {
                            pode8 = false;
                        }

                        if(obstaclesVertices.find(index) == obstaclesVertices.end() && obstaclesVertices.find(index1) == obstaclesVertices.end() && obstaclesVertices.find(index2) == obstaclesVertices.end() && obstaclesVertices.find(index3) == obstaclesVertices.end())
                        {       
                            if(pode8 == true)
                            {
                                if(navigableVerticesMap.find(index) == navigableVerticesMap.end())
                                {
                                    
                                    navigableVerticesMap[index] = {k, new_x2, new_y2, new_z2};
        
                                    navigableVerticesMapInteger[navigableVerticesMap[index].key] = {k, new_x2, new_y2, new_z2};
        
                                    adjacency_list[current].push_back(navigableVerticesMap[index].key);
                                    
                                    k++;
                                    
                                  
                                }
                                else
                                {
                                                            
                                    adjacency_list[current].push_back(navigableVerticesMap[index].key);
        
                                }   
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
        
            float dx = pose_x_ - static_cast<float>(verticesDestino_[i_].x);
            float dy = pose_y_ - static_cast<float>(verticesDestino_[i_].y);
            float dz = pose_z_ - static_cast<float>(verticesDestino_[i_].z);

            float distanciaAteODestino = sqrt(dx * dx + dy * dy + dz * dz);

            if(distanciaAteODestino <= distanceToObstacle_)
            {
                i_ = i_ + 1;
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
           
            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<float> duration = end_time - start_time_;  
            adjacency_list.clear();
            navigableVerticesMap.clear();    
            navigableVerticesMapInteger.clear();
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

            obstaclesVertices.insert(index);
            
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
   
        
        
        if (new_distanceToObstacle != distanceToObstacle_) 
        {
            distanceToObstacle_ = new_distanceToObstacle;
            resolution_ = 1;
            std::cout << "\n" << std::endl;
            RCLCPP_INFO(this->get_logger(), "Updated DistanceToObstacle: %.2f", distanceToObstacle_);
            RCLCPP_INFO(this->get_logger(), "Resolution set to 1.");          
        }

        if(new_diagonalEdges != diagonalEdges_)
        {
            diagonalEdges_ = new_diagonalEdges;

            std::cout << "\n" << std::endl;

            RCLCPP_INFO(this->get_logger(), "Updated diagonalEdges: %d", diagonalEdges_);
        }
       
        

      
    }
    
   
public:
    AStar()
     : Node("a_star")
    {
    
     
        this->declare_parameter<double>("distanceToObstacle", 0.05);
        this->declare_parameter<int>("diagonalEdges", 1);


        // Initialize parameters 
        distanceToObstacle_ =  static_cast<float>(this->get_parameter("distanceToObstacle").get_parameter_value().get<double>());
        diagonalEdges_ = this->get_parameter("diagonalEdges").get_parameter_value().get<int>();


        RCLCPP_INFO(this->get_logger(), "Updated DistanceToObstacle: %f", distanceToObstacle_);
        RCLCPP_INFO(this->get_logger(), "Updated diagonalEdges: %d", diagonalEdges_);

        parameterTimer = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&AStar::check_parameters, this));

        parameterTimer = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&AStar::check_parameters, this));

        decimals = countDecimals(distanceToObstacle_);
       
 
        subscription_navigable_removed_vertices = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/obstacles_vertices", 10, std::bind(&AStar::callback_removed_navigable_vertices, this, std::placeholders::_1));

        publisher_nav_path_ = this->create_publisher<nav_msgs::msg::Path>("visualize_path", 10);
        timer_visualize_path_ = this->create_wall_timer(100ms, std::bind(&AStar::publisher_dijkstra_path, this));

        publisher_path_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/path", 10);
        timer_path_ = this->create_wall_timer(1ms, std::bind(&AStar::publisher_dijkstra, this));
        

        subscription_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/rtabmap/odom", 10, std::bind(&AStar::callback_odom, this, std::placeholders::_1));

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