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


class VisualizeAStar : public rclcpp::Node {

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

    struct TupleCompare {
        bool operator()(const std::pair<float, std::tuple<float, float, float>>& a, 
                        const std::pair<float, std::tuple<float, float, float>>& b) const {
            return a.first > b.first;
        }
    };

 
    //Publishers.
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_path_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_created_vertices;
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
    bool activate_only_with_obstacles = false;
    int diagonalEdges_;
    float pose_x_ = 0.0, pose_y_ = 0.0, pose_z_ = 0.0;
    float distanceToObstacle_;
    int decimals = 0, time_between_points = 1;

    std::tuple<float, float, float> globalGoalIndex;
    std::tuple<float, float, float> globalIndex;

    std::vector<std::tuple<float, float, float>> destinationEdges;
    std::vector<VertexDijkstra> verticesDestino_;
    std::vector<VertexDijkstra> verticesDijkstra;
    std::vector<Edge> shortestPathEdges;
    

    std::unordered_set<std::tuple<float, float,float>> createdVertices;
    std::unordered_map<int, std::vector<int>> adjacency_list;
    std::unordered_set<std::tuple<float, float, float>> obstaclesVertices;
    std::unordered_map<int, Vertex> navigableVerticesMapInteger;

    inline float roundToMultiple(float value, float multiple, int decimals) {
        if (multiple == 0.0) return value; // Evita divisão por zero
        
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
    
    std::vector<std::tuple<float, float, float>> runAStar(float start[3], float goal[3]) 
    {
        destinationEdges.clear();
        
        
        struct Node {
            std::tuple<float, float, float> parent;
            float g_score = std::numeric_limits<float>::infinity();
            float f_score = std::numeric_limits<float>::infinity();
            bool closed = false;
        };
        
        
        std::unordered_map<std::tuple<float, float, float>, Node> nodes;
        
        std::unordered_map<std::tuple<float, float, float>, std::vector<std::tuple<float, float, float>>> adjacency_list_tuples;
        
        auto offsets1 = getOffsets(distanceToObstacle_);
        
        std::tuple<float, float, float> start_tuple = std::make_tuple(start[0], start[1], start[2]);
        std::tuple<float, float, float> goal_tuple = std::make_tuple(goal[0], goal[1], goal[2]);
        
        float new_x = 0.0, new_y = 0.0, new_z = 0.0;
        bool findNavigableVertice = false;
        
        // Find navigable vertices near start
        for(int i = 1; i <= 2; i++)
        {
            for (int a = 0; a < 26; a++) 
            {
                new_x = roundToMultiple(std::get<0>(start_tuple) + (offsets1[a][0] * i), distanceToObstacle_, decimals);
                new_y = roundToMultiple(std::get<1>(start_tuple) + (offsets1[a][1] * i), distanceToObstacle_, decimals);
                new_z = roundToMultiple(std::get<2>(start_tuple) + (offsets1[a][2] * i), distanceToObstacle_, decimals);
   
                auto neighbor_tuple = std::make_tuple(static_cast<float>(new_x), 
                    static_cast<float>(new_y), 
                    static_cast<float>(new_z));
                
                if (obstaclesVertices.find(neighbor_tuple) == obstaclesVertices.end())
                { 
                    adjacency_list_tuples[start_tuple].push_back(neighbor_tuple);
                    findNavigableVertice = true;
                }
            }
        }
        
        if(findNavigableVertice == false) 
        {
            RCLCPP_WARN(this->get_logger(), "The robot is too far of the navigable area.");
            return {};
        }
        
        // Find navigable vertices near goal
        bool findNavigableGoalVertice = false;
        
        for(int i = 1; i <= 2; i++)
        {
            for (int a = 0; a < 26; a++) 
            {
                new_x = roundToMultiple(std::get<0>(goal_tuple) + (offsets1[a][0] * i), distanceToObstacle_, decimals);
                new_y = roundToMultiple(std::get<1>(goal_tuple) + (offsets1[a][1] * i), distanceToObstacle_, decimals);
                new_z = roundToMultiple(std::get<2>(goal_tuple) + (offsets1[a][2] * i), distanceToObstacle_, decimals);
                
                auto neighbor_tuple = std::make_tuple(static_cast<float>(new_x), 
                    static_cast<float>(new_y), 
                    static_cast<float>(new_z));
                
                if (obstaclesVertices.find(neighbor_tuple) == obstaclesVertices.end())
                { 
                    adjacency_list_tuples[neighbor_tuple].push_back(goal_tuple);
                    findNavigableGoalVertice = true;
                }
            }
        }
        
        if(findNavigableGoalVertice == false)
        {
            RCLCPP_WARN(this->get_logger(), "Destination is too far of the navigable area. Increase navigable area.");
            return {};
        }
        
        auto heuristic = [](const std::tuple<float, float, float>& a, const std::tuple<float, float, float>& b) {
            float x1 = std::get<0>(a);
            float y1 = std::get<1>(a);
            float z1 = std::get<2>(a);
            
            float x2 = std::get<0>(b);
            float y2 = std::get<1>(b);
            float z2 = std::get<2>(b);
            
            return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2) + std::pow(z2 - z1, 2));
        };
        
        nodes[start_tuple].g_score = 0;
        nodes[start_tuple].f_score = heuristic(start_tuple, goal_tuple);
        
        struct TupleCompare {
            bool operator()(const std::pair<float, std::tuple<float, float, float>>& a, 
                            const std::pair<float, std::tuple<float, float, float>>& b) const {
                return a.first > b.first;
            }
        };
        
        std::priority_queue<
            std::pair<float, std::tuple<float, float, float>>,
            std::vector<std::pair<float, std::tuple<float, float, float>>>,
            TupleCompare
        > open_set;
        
        open_set.push({nodes[start_tuple].f_score, start_tuple});
        
        while (!open_set.empty()) 
        {
            auto current_pair = open_set.top();
            open_set.pop();
            auto current = current_pair.second;
            
            if (nodes[current].closed)
                continue;
                
            if (current_pair.first > nodes[current].f_score)
                continue;
                
            nodes[current].closed = true;

            
            createdVertices.insert(current);

            publish_created_vertices();

            std::this_thread::sleep_for(std::chrono::milliseconds(time_between_points));
        


            
            if (current != start_tuple && current != goal_tuple)
            {
                for (int a = 0; a < 26; a++) 
                {
                    new_x = roundToMultiple(std::get<0>(current) + offsets1[a][0], distanceToObstacle_, decimals);
                    new_y = roundToMultiple(std::get<1>(current) + offsets1[a][1], distanceToObstacle_, decimals);
                    new_z = roundToMultiple(std::get<2>(current) + offsets1[a][2], distanceToObstacle_, decimals);

                    
                    auto neighbor_tuple = std::make_tuple(static_cast<float>(new_x), 
                        static_cast<float>(new_y), 
                        static_cast<float>(new_z));
                    
                    if (obstaclesVertices.find(neighbor_tuple) == obstaclesVertices.end())
                    {
                        adjacency_list_tuples[current].push_back(neighbor_tuple);
                    }
                }





                int i = 2;
                float new_x2 = 0.0, new_y2 = 0.0, new_z2 = 0.0;
                bool pode1 = true, pode2 = true, pode3 = true, pode4 = true, pode5 = true, pode6 = true, pode7 = true, pode8 = true;

                while(i <= diagonalEdges_)
                {
                   
                    if(i > 1 || i < -1)
                    {
                   
                        new_x2 = roundToMultiple(std::get<0>(current) + (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_y2 = roundToMultiple(std::get<1>(current) + distanceToObstacle_, distanceToObstacle_, decimals);
                        new_z2 = roundToMultiple(std::get<2>(current), distanceToObstacle_, decimals);
                        
                        auto index = std::make_tuple(static_cast<float>(new_x2), static_cast<float>(new_y2), static_cast<float>(new_z2));

                        new_x = roundToMultiple(std::get<0>(current) + (distanceToObstacle_* (i - 1)), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(std::get<1>(current) + distanceToObstacle_, distanceToObstacle_, decimals);
                        new_z = roundToMultiple(std::get<2>(current), distanceToObstacle_, decimals);
                        
                        auto index1 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));
                        
                        new_x = roundToMultiple(std::get<0>(current) + (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(std::get<1>(current), distanceToObstacle_, decimals);
                        
                        
                        auto index2 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        new_x = roundToMultiple(std::get<0>(current) + (distanceToObstacle_ * (i - 1)), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(std::get<1>(current), distanceToObstacle_, decimals);
                        
                        
                        auto index3 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));
                       
                        //Esse lixo de código é para não verificar todos os vértices toda vez
                        if(pode1 == true) 
                        {
                            if(obstaclesVertices.find(index) != obstaclesVertices.end() || obstaclesVertices.find(index1) != obstaclesVertices.end() || obstaclesVertices.find(index2) != obstaclesVertices.end() || obstaclesVertices.find(index3) != obstaclesVertices.end())
                            {
                                pode1 = false;
                            }
                            
                            if(pode1 == true)
                            {
                                adjacency_list_tuples[current].push_back(index);
                            }
                            
                        }
                      
                        
                        

                        new_x2 = roundToMultiple(std::get<0>(current) + (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_y2 = roundToMultiple(std::get<1>(current) - distanceToObstacle_, distanceToObstacle_, decimals);
                        
                        
                        index = std::make_tuple(static_cast<float>(new_x2), static_cast<float>(new_y2), static_cast<float>(new_z2));

                        new_x = roundToMultiple(std::get<0>(current) + (distanceToObstacle_* (i - 1)), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(std::get<1>(current) - distanceToObstacle_, distanceToObstacle_, decimals);
                        
                        
                        index1 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));
                        
                        new_x = roundToMultiple(std::get<0>(current) + (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(std::get<1>(current), distanceToObstacle_, decimals);
                        
                        
                        index2 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        new_x = roundToMultiple(std::get<0>(current) + (distanceToObstacle_ * (i - 1)), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(std::get<1>(current), distanceToObstacle_, decimals);
                        
                        
                        index3 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));
                       

                        if(pode2 == true) 
                        {
                            if(obstaclesVertices.find(index) != obstaclesVertices.end() || obstaclesVertices.find(index1) != obstaclesVertices.end() || obstaclesVertices.find(index2) != obstaclesVertices.end() || obstaclesVertices.find(index3) != obstaclesVertices.end())
                            {
                                pode2 = false;
                            }
                            
                            if(pode2 == true)
                            {
                                adjacency_list_tuples[current].push_back(index);
                            }
                        }


                        

                        new_x2 = roundToMultiple(std::get<0>(current) - (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_y2 = roundToMultiple(std::get<1>(current) + distanceToObstacle_, distanceToObstacle_, decimals);
                        
                        
                        index = std::make_tuple(static_cast<float>(new_x2), static_cast<float>(new_y2), static_cast<float>(new_z2));

                        new_x = roundToMultiple(std::get<0>(current) + (distanceToObstacle_* (i - 1)), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(std::get<1>(current) + distanceToObstacle_, distanceToObstacle_, decimals);
                        
                        
                        index1 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));
                        
                        new_x = roundToMultiple(std::get<0>(current) + (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(std::get<1>(current), distanceToObstacle_, decimals);
                        
                        
                        index2 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        new_x = roundToMultiple(std::get<0>(current) + (distanceToObstacle_ * (i - 1)), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(std::get<1>(current), distanceToObstacle_, decimals);
                        
                        
                        index3 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));
                       
                        if(pode3 == true) 
                        {
                            if(obstaclesVertices.find(index) != obstaclesVertices.end() || obstaclesVertices.find(index1) != obstaclesVertices.end() || obstaclesVertices.find(index2) != obstaclesVertices.end() || obstaclesVertices.find(index3) != obstaclesVertices.end())
                            {
                                pode3 = false;
                            }
                            
                            if(pode3 == true)
                            {
                                adjacency_list_tuples[current].push_back(index);
                            }
                        }
                        

                        new_x2 = roundToMultiple(std::get<0>(current) - (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_y2 = roundToMultiple(std::get<1>(current) - distanceToObstacle_, distanceToObstacle_, decimals);
                        
                        
                        index = std::make_tuple(static_cast<float>(new_x2), static_cast<float>(new_y2), static_cast<float>(new_z2));

                        new_x = roundToMultiple(std::get<0>(current) - (distanceToObstacle_* (i - 1)), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(std::get<1>(current) - distanceToObstacle_, distanceToObstacle_, decimals);
                        
                        
                        index1 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));
                        
                        new_x = roundToMultiple(std::get<0>(current) - (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(std::get<1>(current), distanceToObstacle_, decimals);
                        
                        
                        index2 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        new_x = roundToMultiple(std::get<0>(current) - (distanceToObstacle_ * (i - 1)), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(std::get<1>(current), distanceToObstacle_, decimals);
                        
                        
                        index3 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));
                       
                        if(pode4 == true) 
                        {
                            if(obstaclesVertices.find(index) != obstaclesVertices.end() || obstaclesVertices.find(index1) != obstaclesVertices.end() || obstaclesVertices.find(index2) != obstaclesVertices.end() || obstaclesVertices.find(index3) != obstaclesVertices.end())
                            {
                                pode4 = false;
                            }
                            
                            if(pode4 == true)
                            {
                                adjacency_list_tuples[current].push_back(index);
                            }
                        }


    

                        new_x2 = roundToMultiple(std::get<0>(current) + distanceToObstacle_, distanceToObstacle_, decimals);
                        new_y2 = roundToMultiple(std::get<1>(current) + (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        
                        
                        index = std::make_tuple(static_cast<float>(new_x2), static_cast<float>(new_y2), static_cast<float>(new_z2));

                        
                        new_x = roundToMultiple(std::get<0>(current) + distanceToObstacle_, distanceToObstacle_, decimals);
                        new_y = roundToMultiple(std::get<1>(current) + (distanceToObstacle_ * (i - 1) ), distanceToObstacle_, decimals);
                        
                        
                        index1 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        new_x = roundToMultiple(std::get<0>(current), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(std::get<1>(current) + (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        
                        
                        index2 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        new_x = roundToMultiple(std::get<0>(current), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(std::get<1>(current) + (distanceToObstacle_ * (i - 1)), distanceToObstacle_, decimals);
                        
                        
                        index3 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));
                    

                        if(pode5 == true) 
                        {
                            if(obstaclesVertices.find(index) != obstaclesVertices.end() || obstaclesVertices.find(index1) != obstaclesVertices.end() || obstaclesVertices.find(index2) != obstaclesVertices.end() || obstaclesVertices.find(index3) != obstaclesVertices.end())
                            {
                                pode5 = false;
                            }
                            
                            if(pode5 == true)
                            {
                                adjacency_list_tuples[current].push_back(index);
                            }
                        }
                        


                        new_x2 = roundToMultiple(std::get<0>(current) - distanceToObstacle_, distanceToObstacle_, decimals);
                        new_y2 = roundToMultiple(std::get<1>(current) + (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        
                        
                        index = std::make_tuple(static_cast<float>(new_x2), static_cast<float>(new_y2), static_cast<float>(new_z2));

                        
                        new_x = roundToMultiple(std::get<0>(current) - distanceToObstacle_, distanceToObstacle_, decimals);
                        new_y = roundToMultiple(std::get<1>(current) + (distanceToObstacle_ * (i - 1) ), distanceToObstacle_, decimals);
                        
                        
                        index1 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        new_x = roundToMultiple(std::get<0>(current), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(std::get<1>(current) + (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        
                        
                        index2 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        new_x = roundToMultiple(std::get<0>(current), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(std::get<1>(current) + (distanceToObstacle_ * (i - 1)), distanceToObstacle_, decimals);
                        
                        
                        index3 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));
                    

                        if(pode6 == true) 
                        {
                            if(obstaclesVertices.find(index) != obstaclesVertices.end() || obstaclesVertices.find(index1) != obstaclesVertices.end() || obstaclesVertices.find(index2) != obstaclesVertices.end() || obstaclesVertices.find(index3) != obstaclesVertices.end())
                            {
                                pode6 = false;
                            }
                            
                            if(pode6 == true)
                            {
                                adjacency_list_tuples[current].push_back(index);
                            }
                        }



                        new_x2 = roundToMultiple(std::get<0>(current) + distanceToObstacle_, distanceToObstacle_, decimals);
                        new_y2 = roundToMultiple(std::get<1>(current) - (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        
                        
                        index = std::make_tuple(static_cast<float>(new_x2), static_cast<float>(new_y2), static_cast<float>(new_z2));

                        
                        new_x = roundToMultiple(std::get<0>(current) + distanceToObstacle_, distanceToObstacle_, decimals);
                        new_y = roundToMultiple(std::get<1>(current) - (distanceToObstacle_ * (i - 1) ), distanceToObstacle_, decimals);
                        
                        
                        index1 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        new_x = roundToMultiple(std::get<0>(current), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(std::get<1>(current) - (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        
                        
                        index2 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        new_x = roundToMultiple(std::get<0>(current), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(std::get<1>(current) - (distanceToObstacle_ * (i - 1)), distanceToObstacle_, decimals);
                        
                        
                        index3 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));
                    
                        
                        if(pode7 == true) 
                        {
                            if(obstaclesVertices.find(index) != obstaclesVertices.end() || obstaclesVertices.find(index1) != obstaclesVertices.end() || obstaclesVertices.find(index2) != obstaclesVertices.end() || obstaclesVertices.find(index3) != obstaclesVertices.end())
                            {
                                pode7 = false;
                            }
                            
                            if(pode7 == true)
                            {
                                adjacency_list_tuples[current].push_back(index);
                            }
                        }
                        
    

                        new_x2 = roundToMultiple(std::get<0>(current) - distanceToObstacle_, distanceToObstacle_, decimals);
                        new_y2 = roundToMultiple(std::get<1>(current) - (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        
                        
                        index = std::make_tuple(static_cast<float>(new_x2), static_cast<float>(new_y2), static_cast<float>(new_z2));

                        
                        new_x = roundToMultiple(std::get<0>(current) - distanceToObstacle_, distanceToObstacle_, decimals);
                        new_y = roundToMultiple(std::get<1>(current) - (distanceToObstacle_ * (i - 1) ), distanceToObstacle_, decimals);
                        
                        
                        index1 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        new_x = roundToMultiple(std::get<0>(current), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(std::get<1>(current) - (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        
                        
                        index2 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        new_x = roundToMultiple(std::get<0>(current), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(std::get<1>(current) - (distanceToObstacle_ * (i - 1)), distanceToObstacle_, decimals);
                        
                        
                        index3 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));
                    

                        if(pode8 == true) 
                        {
                            if(obstaclesVertices.find(index) != obstaclesVertices.end() || obstaclesVertices.find(index1) != obstaclesVertices.end() || obstaclesVertices.find(index2) != obstaclesVertices.end() || obstaclesVertices.find(index3) != obstaclesVertices.end())
                            {
                                pode8 = false;
                            }
                            
                            if(pode8 == true)
                            {
                                adjacency_list_tuples[current].push_back(index);
                            }
                        }
                        

                    }
              
                    i++;
                }
    


            }
            
            if (current == goal_tuple) 
            {
                std::vector<std::tuple<float, float, float>> path;
                auto current_vertex = current;
                
                path.insert(path.begin(), current_vertex);
                
                while (nodes.find(current_vertex) != nodes.end() && 
                    current_vertex != start_tuple) {
                    current_vertex = nodes[current_vertex].parent;
                    path.insert(path.begin(), current_vertex);
                }
                
                return path;
            }
            
           
            for (const auto& neighbor : adjacency_list_tuples[current])
            {
                if (nodes.find(neighbor) != nodes.end() && nodes[neighbor].closed)
                    continue;
                
                float tentative_g_score = nodes[current].g_score + heuristic(current, neighbor);
                
                if (nodes.find(neighbor) == nodes.end() || tentative_g_score < nodes[neighbor].g_score) 
                {
                    nodes[neighbor].parent = current;
                    nodes[neighbor].g_score = tentative_g_score;
                    nodes[neighbor].f_score = tentative_g_score + heuristic(neighbor, goal_tuple);
                    open_set.push({nodes[neighbor].f_score, neighbor});
                }
            }
            
            adjacency_list_tuples.erase(current);
        }
        
        RCLCPP_WARN(this->get_logger(), "It is not possible to reach the destination.");
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
            
        
        return path;
    }

    void storeEdgesInPath(const std::vector<std::tuple<float, float, float>>& path) 
    {
        shortestPathEdges.clear();
        verticesDijkstra.clear();
    
        if (path.empty()) {
            return;
        }
    
        // Processar as arestas do caminho
        for (size_t i = 0; i < path.size() - 1; i++) 
        {
            int u = i;  // Usando o índice para representar os vértices
            int v = i + 1;
            shortestPathEdges.push_back({u, v});
        }
    
        // Processar os vértices do caminho
        for (size_t i = 0; i < path.size(); i++) 
        {
            VertexDijkstra vertex;
            
            // Acessando os elementos da tupla
            vertex.x = std::get<0>(path[i]);
            vertex.y = std::get<1>(path[i]);
            vertex.z = std::get<2>(path[i]);
    
            // Cálculo da orientação entre os vértices
            if (i < path.size() - 1) 
            {
                // Obter o próximo vértice no caminho
                const std::tuple<float, float, float>& current_vertex = path[i];
                const std::tuple<float, float, float>& next_vertex = path[i + 1];
    
                float dx = std::get<0>(next_vertex) - std::get<0>(current_vertex);
                float dy = std::get<1>(next_vertex) - std::get<1>(current_vertex);
                float dz = std::get<2>(next_vertex) - std::get<2>(current_vertex);
                float distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    
                // Normalizar a direção
                if (distance > 0.0f) {
                    dx /= distance;
                    dy /= distance;
                    dz /= distance;
                }
    
                // Criar o quaternion com a direção normalizada
                Eigen::Vector3f direction(dx, dy, dz);
                Eigen::Vector3f reference(1.0f, 0.0f, 0.0f); // Vetor de referência
    
                Eigen::Quaternionf quaternion = Eigen::Quaternionf::FromTwoVectors(reference, direction);
    
                // Definir a orientação
                vertex.orientation_x = quaternion.x();
                vertex.orientation_y = quaternion.y();
                vertex.orientation_z = quaternion.z();
                vertex.orientation_w = quaternion.w();
            } 
            else 
            {
                // Para o último vértice, sem orientação
                vertex.orientation_x = 0.0;
                vertex.orientation_y = 0.0;
                vertex.orientation_z = 0.0;
                vertex.orientation_w = 1.0;
            }
    
            // Adicionar o vértice processado à lista
            verticesDijkstra.push_back(vertex);
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

    void publish_created_vertices()
    {
        sensor_msgs::msg::PointCloud2 cloud_msg_created_vertices;
        cloud_msg_created_vertices.header.stamp = this->get_clock()->now();
        cloud_msg_created_vertices.header.frame_id = "map";

        // Configuração dos campos do PointCloud2
        cloud_msg_created_vertices.height = 1;  // Ponto único em cada linha
        cloud_msg_created_vertices.width = createdVertices.size(); // Quantidade de vértices
        cloud_msg_created_vertices.is_dense = true;
        cloud_msg_created_vertices.is_bigendian = false;
        cloud_msg_created_vertices.point_step = 3 * sizeof(float); // x, y, z
        cloud_msg_created_vertices.row_step = cloud_msg_created_vertices.point_step * cloud_msg_created_vertices.width;

        // Adicionar campos de x, y, z
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg_created_vertices);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(cloud_msg_created_vertices.width);

        // Preencher os dados do PointCloud2
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg_created_vertices, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg_created_vertices, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg_created_vertices, "z");
        for (const auto& vertex : createdVertices) 
        {
           
                *iter_x = std::get<0>(vertex);
                *iter_y = std::get<1>(vertex);
                *iter_z = std::get<2>(vertex);

                ++iter_x;
                ++iter_y;
                ++iter_z;
   
            
        }


         publisher_created_vertices->publish(cloud_msg_created_vertices);
    }


    /*
    
        CALLBACKS.

    */

    bool obstaclesVerticesReceived = !activate_only_with_obstacles;

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
        
        
        if(!verticesDestino_.empty() && obstaclesVerticesReceived == true)
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

            /*

                coloquei createdVertices.clear() aqui para poder ver o caminho gerado 
                por 3 segundos depois que o A* foi executado.

            */

            verticesDijkstra.clear();
            createdVertices.clear();

            // Publicar createdVertices vazio (só para tirar nuvem de pontos do rviz)
            publish_created_vertices();

            RCLCPP_INFO(this->get_logger(), "A* will be executed in 3 seconds.");
            std::this_thread::sleep_for(std::chrono::milliseconds(3000));

              //Publicar caminho vaio para sair do rviz.
            publisher_dijkstra_path();
            auto start_time_ = std::chrono::high_resolution_clock::now();

            std::vector<std::tuple<float, float, float>> shortestPath = runAStar(array_inicial, array_final);
           
            storeEdgesInPath(shortestPath);
          
            adjacency_list.clear();
            

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
                roundToMultiple(z, distanceToObstacle_, decimals)
            );

            obstaclesVertices.insert(index);

         
        }
       
        if(!obstaclesVertices.empty())
        {
            obstaclesVerticesReceived = true; 
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
        auto new_time_between_points = this->get_parameter("time_between_points").get_parameter_value().get<int>();
        auto new_activate_only_with_obstacles = this->get_parameter("activate_only_with_obstacles").get_parameter_value().get<bool>();
        
        if (new_distanceToObstacle != distanceToObstacle_) 
        {
            distanceToObstacle_ = new_distanceToObstacle;
            std::cout << "\n" << std::endl;
            RCLCPP_INFO(this->get_logger(), "distanceToObstacle set to: %.2f", distanceToObstacle_);          
        }

        if(new_diagonalEdges != diagonalEdges_)
        {
            diagonalEdges_ = new_diagonalEdges;

            std::cout << "\n" << std::endl;

            RCLCPP_INFO(this->get_logger(), "diagonalEdges set to: %d", diagonalEdges_);
        }
       
        if(new_time_between_points != time_between_points)
        {
            time_between_points = new_time_between_points;
            RCLCPP_INFO(this->get_logger(), "time_between_points set to: %d ms", time_between_points);
        }

        if(new_activate_only_with_obstacles != activate_only_with_obstacles)
        {
            activate_only_with_obstacles = new_activate_only_with_obstacles;
            RCLCPP_INFO(this->get_logger(), "activate_only_with_obstacles set to: %s", activate_only_with_obstacles ? "true" : "false");
        }

      
    }
    
   
public:
    VisualizeAStar()
     : Node("visualize_a_star")
    {
    
     
        this->declare_parameter<double>("distanceToObstacle", 0.2);
        this->declare_parameter<int>("diagonalEdges", 3);
        this->declare_parameter<int>("time_between_points", 1);
        this->declare_parameter<bool>("activate_only_with_obstacles", false);

        distanceToObstacle_ =  static_cast<float>(this->get_parameter("distanceToObstacle").get_parameter_value().get<double>());
        diagonalEdges_ = this->get_parameter("diagonalEdges").get_parameter_value().get<int>();
        time_between_points = this->get_parameter("time_between_points").get_parameter_value().get<int>();
        activate_only_with_obstacles = this->get_parameter("activate_only_with_obstacles").get_parameter_value().get<bool>();


        RCLCPP_INFO(this->get_logger(), "distanceToObstacle is set to: %f", distanceToObstacle_);
        RCLCPP_INFO(this->get_logger(), "diagonalEdges is set to: %d", diagonalEdges_);
        RCLCPP_INFO(this->get_logger(), "time_between_points is set to: %d ms", time_between_points);
        RCLCPP_INFO(this->get_logger(), "activate_only_with_obstacles is set to: %s", activate_only_with_obstacles ? "true" : "false");
        
        
        parameterTimer = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&VisualizeAStar::check_parameters, this));

      
        decimals = countDecimals(distanceToObstacle_);
       
 
        subscription_navigable_removed_vertices = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/obstacles_vertices", 10, std::bind(&VisualizeAStar::callback_removed_navigable_vertices, this, std::placeholders::_1));

        publisher_nav_path_ = this->create_publisher<nav_msgs::msg::Path>("/visualize_path", 10);
        timer_visualize_path_ = this->create_wall_timer(100ms, std::bind(&VisualizeAStar::publisher_dijkstra_path, this));

        publisher_created_vertices = this->create_publisher<sensor_msgs::msg::PointCloud2>("/created_vertices", 10);


        publisher_path_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/path", 10);
        timer_path_ = this->create_wall_timer(1ms, std::bind(&VisualizeAStar::publisher_dijkstra, this));
        

        subscription_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/rtabmap/odom", 10, std::bind(&VisualizeAStar::callback_odom, this, std::placeholders::_1));

        subscription3_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/destinations", 10, std::bind(&VisualizeAStar::callback_destinations, this, std::placeholders::_1));


       
    }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<VisualizeAStar>());
    rclcpp::shutdown();
    return 0;
}