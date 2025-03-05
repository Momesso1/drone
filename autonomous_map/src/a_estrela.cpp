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
    int temp_ = 1, maxSize = 0, diagonalEdges_;
    float resolution_;
    float pose_x_ = 0.0, pose_y_ = 0.0, pose_z_ = 0.0;
    float distanceToObstacle_;
    float x_min_, x_max_;
    float y_min_, y_max_;
    float z_min_, z_max_;
    
    int decimals = 0;

    std::tuple<float, float, float> globalGoalIndex;
    std::tuple<float, float, float> globalIndex;

    std::vector<std::tuple<float, float, float>> destinationEdges;
    std::vector<VertexDijkstra> verticesDestino_;
    std::vector<VertexDijkstra> verticesDijkstra;
    std::vector<Edge> shortestPathEdges;
    std::vector<Edge> navigableEdges_; 

    
    std::unordered_map<int, std::vector<std::pair<int, int>>> relatedEdges;
    
    std::unordered_map<int, std::unordered_set<std::pair<int, int>, PairHash>> adjacency_list;
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

    /*
        Edges
    */

  

    std::vector<std::array<float, 3>> getOffsets(float distanceToObstacle) {
        return {
         
            // Basic unit offsets
        {-distanceToObstacle, distanceToObstacle, 0.0},
        {0.0, distanceToObstacle, 0.0},
        {distanceToObstacle, distanceToObstacle, 0.0},
          {-distanceToObstacle, 0.0, 0.0},
          {distanceToObstacle, 0.0, 0.0},
         
          {0.0, -distanceToObstacle, 0.0},
          
          {0.0, 0.0, -distanceToObstacle},
          
          {-distanceToObstacle, -distanceToObstacle, 0.0},
          {distanceToObstacle, -distanceToObstacle, 0.0},
          
          
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
          

        };
    }
    
    void remove_edges2(int u, int v)
    {
     
        for (auto it = adjacency_list[u].begin(); it != adjacency_list[u].end(); )
        {
            std::pair<int, int> current = *it;
            
            
            if (current.first == v && current.second == v)
            {
                // Remove (v, v)
                it = adjacency_list[u].erase(it);
            }
            else if (v > current.first && v < current.second)
            {
                // Se v estiver entre os limites, divida o intervalo em dois
                if (current.first <= v - 1) 
                {
                    adjacency_list[u].insert({current.first, v - 1});
                }
                if (v + 1 <= current.second) 
                {
                    adjacency_list[u].insert({v + 1, current.second});
                }

                it = adjacency_list[u].erase(it);
              
            }
            else if (v == current.first)
            {
                // Se o primeiro elemento é v, incrementa-o
                auto p = current;
                p.first += 1;
                adjacency_list[u].insert(p);
                it = adjacency_list[u].erase(it);
                
            }
            else if (current.second == v)
            {
                // Se o segundo elemento é v, decrementa-o
                auto p = current;
                p.second -= 1;
                adjacency_list[u].insert(p);
                it = adjacency_list[u].erase(it);
                
            }
            else
            {
                ++it;
            }
        }
        
        
            
    
        
        for (auto it = adjacency_list[v].begin(); it != adjacency_list[v].end(); )
        {
           // std::cout << "itV " << it->first << " " << it->second << std::endl;
            if(it->first == u && it->second == u)
            {
                // Exemplo: (1820, 1820) é removido.
                it = adjacency_list[v].erase(it);
            }
            else if (u > it->first && u < it->second)
            {
                // Exemplo: u está entre os limites do par.
                // Se o par for, por exemplo, (1818, 1824) e u == 1820,
                // Dois pares são criados: (1818, 1819) e (1821, 1824).
                

                
                auto p = *it;
                it = adjacency_list[v].erase(it);
                if (p.first <= u - 1)
                {
                    adjacency_list[v].insert({p.first, u - 1});
                    //std::cout << "p.first " <<p.first << " " << u - 1 << std::endl;
                }
                if (u + 1 <= p.second)
                {
                    adjacency_list[v].insert({u + 1, p.second});
                    //std::cout << "p.second " << u + 1 << " " << p.second << std::endl;
                }
            }
            else if (it->first == u)
            {
                // Exemplo: (1820, 1823) se torna (1821, 1823).
                auto p = *it;
                it = adjacency_list[v].erase(it);

                p.first += 1; 
                adjacency_list[v].insert(p);
            }
            else if (it->second == u)
            {
                // Exemplo: (1820, 1823) se torna (1820, 1822).
                auto p = *it;
                it = adjacency_list[v].erase(it);

                p.second -= 1; 
                adjacency_list[v].insert(p);
            }
            else
            {
                ++it;
            }
        }
    }

    void remove_edges(std::tuple<float, float, float> index)
    {
        

        for (const auto& edgePair : relatedEdges[navigableVerticesMap[index].key])
        {
            
            int u = edgePair.first;
            int v = edgePair.second;
            
            if(navigableVerticesMapInteger.find(v) != navigableVerticesMapInteger.end() && navigableVerticesMapInteger.find(u) != navigableVerticesMapInteger.end())
            {
                int i_1 = 0, i_2 = 0;
                bool negative_x = false, negative_y = false;

                float new_x = 0.0, new_y = 0.0;

                float temp = navigableVerticesMapInteger[v].x;
                float temp1 = navigableVerticesMapInteger[v].y;
                float temp2 = navigableVerticesMapInteger[u].x;
                float temp3 = navigableVerticesMapInteger[u].y;


                float x = std::abs(navigableVerticesMapInteger[v].x) - std::abs(navigableVerticesMapInteger[u].x);
                float y = std::abs(navigableVerticesMapInteger[v].y) - std::abs(navigableVerticesMapInteger[u].y);
                float dx = navigableVerticesMapInteger[v].x - navigableVerticesMapInteger[u].x;
                float dy = navigableVerticesMapInteger[v].y - navigableVerticesMapInteger[u].y;

                
                bool left = (dx < 0); 
                bool down = (dy < 0); 

                if(down)
                {
                    negative_y = true;
                }

                if(left)
                {
                    negative_x = true;
                }
                                

                if(x != 0)
                {
                    i_1 = std::abs(x) / distanceToObstacle_;
                   
                }
    
                if(y != 0)
                {
                    i_2 = std::abs(y) / distanceToObstacle_;
                }

               
                
    
                if(negative_x == false)
                {
                    new_x = distanceToObstacle_;
                }
                else
                {
                    new_x = -distanceToObstacle_; 
                }
    
                if(negative_y == false)
                {
                    new_y = distanceToObstacle_;
                }
                else
                {
                    new_y = -distanceToObstacle_;
                }

                int i = -1;

                if(i_1 >= i_2)
                {
                    i = i_1 - 1;
                }
                else
                {
                    i = i_2 - 1;
                }
                
                
                
                if(i > diagonalEdges_)
                {
                    std::cout << "\n" << std::endl;
                    std::cout << temp << " " << temp1 << " " << temp2 << " " << temp3 << std::endl;
                    std::cout << navigableVerticesMapInteger[v].x << " " << navigableVerticesMapInteger[v].y << std::endl;
                    std::cout << navigableVerticesMapInteger[u].x << " " << navigableVerticesMapInteger[u].y << std::endl;
                    std::cout << x << " " << y << std::endl;
                    std::cout << i_1 << " " << i_2 << " " << i << std::endl;
                }

            
                
                i = -1;
                while(true)
                {
                    if(i_1 >= i_2)
                    {
                        auto index1 = std::make_tuple(static_cast<float>(navigableVerticesMapInteger[v].x + (new_x * i)), 
                                static_cast<float>(navigableVerticesMapInteger[v].y), 
                                static_cast<float>(navigableVerticesMapInteger[v].z));
                        
                        
                        
                        if(navigableVerticesMap.find(index1) != navigableVerticesMap.end())
                        {
                            remove_edges2(u, navigableVerticesMap[index1].key);
                        }
                        

                        
                    }
                    else
                    {
                        auto index2 = std::make_tuple(static_cast<float>(navigableVerticesMapInteger[v].x), 
                                static_cast<float>(navigableVerticesMapInteger[v].y + (new_y * i)), 
                                static_cast<float>(navigableVerticesMapInteger[v].z));
    
                        if(navigableVerticesMap.find(index2) != navigableVerticesMap.end())
                        {
                            remove_edges2(u, navigableVerticesMap[index2].key);
                        }
                    }

                    if(i == diagonalEdges_ + 1)
                    {
                        break;
                    }

                    i++;
                   
                }     

            }
           
        } 
    }

    std::vector<std::array<float, 3>> getOffsets2(float distanceToObstacle) {
        return {
           
            {distanceToObstacle, distanceToObstacle, 0.0}, 
            {distanceToObstacle, 0.0, 0.0},

            {distanceToObstacle, -distanceToObstacle, 0.0},
            {distanceToObstacle, 0.0, 0.0},

            {-distanceToObstacle, -distanceToObstacle, 0.0},
            {-distanceToObstacle, 0.0, 0.0},

            {-distanceToObstacle, distanceToObstacle, 0.0},
            {-distanceToObstacle, 0.0, 0.0},



        
            
            {-distanceToObstacle, -distanceToObstacle, 0.0},
            {0.0, -distanceToObstacle, 0.0},

            {distanceToObstacle,  -distanceToObstacle, 0.0},
            {0.0, -distanceToObstacle , 0.0},






            {distanceToObstacle, distanceToObstacle, 0.0},
            {0.0, distanceToObstacle, 0.0},

            {-distanceToObstacle, distanceToObstacle, 0.0},
            {0.0, distanceToObstacle, 0.0},

        };
      }

      void saveGraphToBinaryFile(const std::string& filename) 
      {
          std::ofstream out(filename, std::ios::binary);
          if (!out) {
              std::cerr << "Erro ao abrir o arquivo para escrita binária: " << filename << std::endl;
              return;
          }
      
          // Salvar os vértices
          size_t numVertices = navigableVerticesMapInteger.size();
          out.write(reinterpret_cast<const char*>(&numVertices), sizeof(numVertices));
      
          for (const auto& pair : navigableVerticesMapInteger) {
              const Vertex& v = pair.second;
              out.write(reinterpret_cast<const char*>(&v.key), sizeof(v.key));
              out.write(reinterpret_cast<const char*>(&v.x), sizeof(v.x));
              out.write(reinterpret_cast<const char*>(&v.y), sizeof(v.y));
              out.write(reinterpret_cast<const char*>(&v.z), sizeof(v.z));
          }
      
          // Salvar a lista de adjacências
          size_t numAdjEntries = adjacency_list.size();
          out.write(reinterpret_cast<const char*>(&numAdjEntries), sizeof(numAdjEntries));
      
          for (const auto& entry : adjacency_list) {
              int origem = entry.first;
              out.write(reinterpret_cast<const char*>(&origem), sizeof(origem));
      
              size_t numEdges = entry.second.size();
              out.write(reinterpret_cast<const char*>(&numEdges), sizeof(numEdges));
      
              for (const auto& destino : entry.second) {
                  out.write(reinterpret_cast<const char*>(&destino.first), sizeof(destino.first));
                  out.write(reinterpret_cast<const char*>(&destino.second), sizeof(destino.second));  // Caso haja peso
              }
          }
      
          // Salvar relatedEdges
          size_t numRelatedEdges = relatedEdges.size();
          out.write(reinterpret_cast<const char*>(&numRelatedEdges), sizeof(numRelatedEdges));
      
          for (const auto& pair : relatedEdges) {
              int key = pair.first;
              out.write(reinterpret_cast<const char*>(&key), sizeof(key));
      
              size_t vecSize = pair.second.size();
              out.write(reinterpret_cast<const char*>(&vecSize), sizeof(vecSize));
      
              // Agora, iteramos sobre o vetor de inteiros
              for (const auto& value : pair.second) {
                  out.write(reinterpret_cast<const char*>(&value), sizeof(value));
              }
          }
      
          out.close();
          std::cout << "Grafo salvo em formato binário em: " << filename << std::endl;
      }
      
      
      
      
      void loadGraphFromBinaryFile(const std::string& filename) {
          std::ifstream in(filename, std::ios::binary);
          if (!in) {
              std::cerr << "Erro ao abrir o arquivo para leitura binária: " << filename << std::endl;
              return;
          }
      
          // Limpar as estruturas
          navigableVerticesMap.clear();
          navigableVerticesMapInteger.clear();
          adjacency_list.clear();
          relatedEdges.clear();
      
          // Carregar os vértices
          size_t numVertices = 0;
          in.read(reinterpret_cast<char*>(&numVertices), sizeof(numVertices));
      
          for (size_t i = 0; i < numVertices; ++i) {
              Vertex v;
              in.read(reinterpret_cast<char*>(&v.key), sizeof(v.key));
              in.read(reinterpret_cast<char*>(&v.x), sizeof(v.x));
              in.read(reinterpret_cast<char*>(&v.y), sizeof(v.y));
              in.read(reinterpret_cast<char*>(&v.z), sizeof(v.z));
      
              navigableVerticesMapInteger[v.key] = v;
              auto index = std::make_tuple(v.x, v.y, v.z);
              navigableVerticesMap[index] = v;
          }
      
          // Carregar a lista de adjacências
          size_t numAdjEntries = 0;
          in.read(reinterpret_cast<char*>(&numAdjEntries), sizeof(numAdjEntries));
      
          for (size_t i = 0; i < numAdjEntries; ++i) {
              int origem = 0;
              in.read(reinterpret_cast<char*>(&origem), sizeof(origem));
      
              size_t numEdges = 0;
              in.read(reinterpret_cast<char*>(&numEdges), sizeof(numEdges));
      
              std::unordered_set<std::pair<int, int>, PairHash> edges;
              for (size_t j = 0; j < numEdges; ++j) {
                  int destino = 0;
                  int peso = 0;  // Supondo que exista um peso na aresta
                  in.read(reinterpret_cast<char*>(&destino), sizeof(destino));
                  in.read(reinterpret_cast<char*>(&peso), sizeof(peso));
                  edges.insert({destino, peso});
              }
      
              adjacency_list[origem] = edges;
          }
      
          // Carregar relatedEdges
          // Carregar relatedEdges
        size_t numRelatedEdges = 0;
        in.read(reinterpret_cast<char*>(&numRelatedEdges), sizeof(numRelatedEdges));

        for (size_t i = 0; i < numRelatedEdges; ++i) {
            int key = 0;
            in.read(reinterpret_cast<char*>(&key), sizeof(key));

            size_t vecSize = 0;
            in.read(reinterpret_cast<char*>(&vecSize), sizeof(vecSize));

            std::vector<std::pair<int, int>> vec;
            for (size_t j = 0; j < vecSize; ++j) {
                int first = 0, second = 0;
                in.read(reinterpret_cast<char*>(&first), sizeof(first));
                in.read(reinterpret_cast<char*>(&second), sizeof(second));
                vec.emplace_back(first, second);
            }
            relatedEdges[key] = vec;  // Agora armazenamos pares corretamente
        }

      
          in.close();
          std::cout << "Grafo carregado a partir do arquivo binário: " << filename << std::endl;
      }
      
    
   
    
    void createGraph() 
    {
        auto start_time_ = std::chrono::high_resolution_clock::now();
        navigableVerticesMap.clear();
        navigableEdges_.clear();
        int id_counter = 0;
        navigableVerticesMapInteger.clear();

        std::ostringstream oss;
        oss << "graph_"
            << std::fixed << std::setprecision(2)
            << x_min_ << "_" << x_max_ << "_"
            << y_min_ << "_" << y_max_ << "_"
            << z_min_ << "_" << z_max_ << "_"
            << distanceToObstacle_<< "_" << diagonalEdges_ << ".bin";

        std::string file_name = oss.str();
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("autonomous_map");
        std::string file_path = "/home/momesso/autonomous/src/autonomous_map/config/" + file_name;


        if(std::filesystem::exists(file_path))
        {
            
            loadGraphFromBinaryFile(file_path);
            RCLCPP_INFO(this->get_logger(), "Grafo do arquivo tem %zu vertices e %zu arestas", navigableVerticesMapInteger.size(), adjacency_list.size());
            auto end_time2 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<float> duration2 = end_time2 - start_time_;
            std::cout << "Vertices created in: " << duration2.count() << " seconds" << std::endl;
        }
        else
        {
            float new_x, new_y, new_z = 0.0;
            
            for (float z = z_min_; z <= z_max_; z += distanceToObstacle_) {
                for (float y = y_min_; y <= y_max_; y += distanceToObstacle_) {
                    for (float x = x_min_; x <= x_max_; x += distanceToObstacle_) {
                    
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
                        }
                    }
                }
            }
            auto end_time1 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<float> duration1 = end_time1 - start_time_;
            std::cout << "Vertices created in: " << duration1.count() << " seconds" << std::endl;
           
        
            maxSize = id_counter;

           
            /*
                Se não tiver criando as arestas corretamente, o erro pode ser aqui.
            */
            auto offsets1 = getOffsets(distanceToObstacle_);
            auto offsets2 = getOffsets2(distanceToObstacle_);

          
        
            for (auto it = navigableVerticesMap.begin(); it != navigableVerticesMap.end(); ++it) 
            {
                std::vector<int> edges;
                std::unordered_set<int> repeatedEdges;
                
            
                for (int a = 0; a < 16; a = a + 2) 
                {
                    auto index3 = std::make_tuple(0.0, 0.0, 0.0);
                    auto index4 = std::make_tuple(0.0, 0.0, 0.0);
                    for(int i = 1; i < diagonalEdges_; i++)
                    {
                        if(a <= 7)
                        {
                            
                            new_x = roundToMultiple(it->second.x + (offsets2[a][0] * i), distanceToObstacle_, decimals);
                            new_y = roundToMultiple(it->second.y + offsets2[a][1], distanceToObstacle_, decimals);
                            new_z = roundToMultipleFromBase(it->second.z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                            
                            auto index = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                            new_x = roundToMultiple(it->second.x + (offsets2[a + 1][0] * i), distanceToObstacle_, decimals);
                            new_y = roundToMultiple(it->second.y + offsets2[a + 1][1], distanceToObstacle_, decimals);
                            new_z = roundToMultipleFromBase(it->second.z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                            
                            auto index1 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));
                            
                            new_x = roundToMultiple(it->second.x + (offsets2[a + 1][0] * (i - 1)), distanceToObstacle_, decimals);
                            new_y = roundToMultiple(it->second.y + offsets2[a + 1][1], distanceToObstacle_, decimals);
                            new_z = roundToMultipleFromBase(it->second.z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                            
                            auto index2 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                            if(navigableVerticesMap.find(index) != navigableVerticesMap.end() && navigableVerticesMap.find(index1) != navigableVerticesMap.end() && navigableVerticesMap.find(index2) != navigableVerticesMap.end() && i != 1)
                            {
                               
                                int keyEdge = navigableVerticesMap[index].key;
                                if (repeatedEdges.find(keyEdge) == repeatedEdges.end())
                                {
                                    edges.push_back(keyEdge);
                                    repeatedEdges.insert(keyEdge);

                                    relatedEdges[navigableVerticesMap[index1].key].push_back({it->second.key, navigableVerticesMap[index].key});
                                    relatedEdges[navigableVerticesMap[index2].key].push_back({it->second.key, navigableVerticesMap[index].key});
                                    relatedEdges[navigableVerticesMap[index3].key].push_back({it->second.key, navigableVerticesMap[index].key});
                                }
                                    
                            }

                            index3 = index;
                               
                        }
                        else
                        {

                            
                            new_x = roundToMultiple(it->second.x + offsets2[a][0], distanceToObstacle_, decimals);
                            new_y = roundToMultiple(it->second.y + (offsets2[a][1] * i ), distanceToObstacle_, decimals);
                            new_z = roundToMultipleFromBase(it->second.z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                            
                            auto index = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                           
                            new_x = roundToMultiple(it->second.x + offsets2[a + 1][0], distanceToObstacle_, decimals);
                            new_y = roundToMultiple(it->second.y + (offsets2[a + 1][1] * i ), distanceToObstacle_, decimals);
                            new_z = roundToMultipleFromBase(it->second.z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                            
                            auto index1 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                            new_x = roundToMultiple(it->second.x + offsets2[a + 1][0], distanceToObstacle_, decimals);
                            new_y = roundToMultiple(it->second.y + (offsets2[a + 1][1] * (i - 1)), distanceToObstacle_, decimals);
                            new_z = roundToMultipleFromBase(it->second.z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                            
                            auto index2 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));
                      
                            if(navigableVerticesMap.find(index) != navigableVerticesMap.end() && navigableVerticesMap.find(index1) != navigableVerticesMap.end() && navigableVerticesMap.find(index2) != navigableVerticesMap.end() && i != 1)
                            {
                                int keyEdge = navigableVerticesMap[index].key;
                                if (repeatedEdges.find(keyEdge) == repeatedEdges.end())
                                {
                                    edges.push_back(keyEdge);
                                    repeatedEdges.insert(keyEdge);

                                    relatedEdges[navigableVerticesMap[index1].key].push_back({it->second.key, navigableVerticesMap[index].key});
                                    relatedEdges[navigableVerticesMap[index2].key].push_back({it->second.key, navigableVerticesMap[index].key});
                                    relatedEdges[navigableVerticesMap[index4].key].push_back({it->second.key, navigableVerticesMap[index].key});
                                }
          
                            }

                            index4 = index;
                             
                        }
                        
                        
                    }
                }

                for (int a = 0; a < 26; a++) 
                {
                   
                    new_x = roundToMultiple(it->second.x + offsets1[a][0], distanceToObstacle_, decimals);
                    new_y = roundToMultiple(it->second.y + offsets1[a][1], distanceToObstacle_, decimals);
                    new_z = roundToMultipleFromBase(it->second.z + offsets1[a][2], roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                    auto index = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));
                                                
                    if(navigableVerticesMap.find(index) != navigableVerticesMap.end())
                    {
                        int keyEdge = navigableVerticesMap[index].key;
                        if (repeatedEdges.find(keyEdge) == repeatedEdges.end())
                        {
                            edges.push_back(navigableVerticesMap[index].key);
                        }                 
                    }       

                }
                std::stable_sort(edges.begin(), edges.end());
                
                int min = edges[0], max = edges[0]; 
            
                for (size_t i = 1; i <= edges.size(); i++) 
                { 
                    if (i < edges.size() && edges[i] == edges[i - 1] + 1) 
                    {
                        max = edges[i]; 
                    }
                    else 
                    {
                        
                       
                        adjacency_list[it->second.key].insert({min, max});
                     
        
                        if (i < edges.size()) 
                        {
                            min = max = edges[i];
                        }
                    }
                }
            

            }

            
            

            RCLCPP_INFO(this->get_logger(), "Graph created with %zu vertices AND %zu edges", navigableVerticesMapInteger.size(), adjacency_list.size());
            RCLCPP_INFO(this->get_logger(), "NAVIGABLEVERTICESMAP: %zu ", navigableVerticesMap.size());
        
       

            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<float> duration = end_time - start_time_;
            std::cout << "Graph created in: " << duration.count() << " seconds" << std::endl;
            
            
            //saveGraphToBinaryFile(file_path);
        }

       
        std::cout << relatedEdges.size() << std::endl;
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
                    
                    edges.push_back(navigableVerticesMap[index1].key);
                    
                    findNavigableVertice = true;
                } 
            }

            if(findNavigableVertice == true)
            {

                int min = edges[0], max = edges[0]; 
            
                for (size_t i = 1; i <= edges.size(); i++) 
                { 
                    if (i < edges.size() && edges[i] == edges[i - 1] + 1) 
                    {
                        max = edges[i]; 
                    }
                    else 
                    {
                
                        adjacency_list[navigableVerticesMap[index].key].insert({min, max});
                     
                        if (i < edges.size()) 
                        {
                            min = max = edges[i];
                        }
                    }
                }

                break;
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

                    adjacency_list[navigableVerticesMap[index2].key].insert({navigableVerticesMap[goalIndex].key, navigableVerticesMap[goalIndex].key});
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
            
            
            if (closed_set.find(current) != closed_set.end())
                continue;
                
          
            if (current_pair.first > f_score[current])
                continue;
                
           
            closed_set.insert(current);
            
            if (current == end_index) 
            {
               
                return reconstructPath(came_from, current);
            }
            
            // Para cada intervalo de vizinhos do nó atual
            for (const auto& range : adjacency_list[current]) 
            {
                size_t start = range.first;
                size_t end = range.second;
                
                // Mesma lógica anterior - percorre cada nó no intervalo
                for (size_t neighbor = start; neighbor <= end; neighbor++) 
                {
                    if (closed_set.find(neighbor) != closed_set.end())
                        continue;
                    

                    
                 
                    // Resto do código (mesma lógica)
                    float tentative_g_score = g_score[current] + heuristic(current, neighbor);
                    if (g_score.find(neighbor) == g_score.end() || tentative_g_score < g_score[neighbor]) 
                    {
                        came_from[neighbor] = current;
                        g_score[neighbor] = tentative_g_score;
                        f_score[neighbor] = tentative_g_score + heuristic(neighbor, end_index);
                        open_set.push({f_score[neighbor], neighbor});
                    }
                }
            }
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
                adjacency_list.erase(navigableVerticesMap[globalGoalIndex].key);
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

            adjacency_list.erase(navigableVerticesMap[globalIndex].key);            
            navigableVerticesMapInteger.erase(navigableVerticesMap[globalIndex].key);
            navigableVerticesMap.erase(globalIndex);

           
            for(const auto& tuple : destinationEdges)
            {
               
                /*
                    Removendo o vértice de destino da lista de adjacência das tuplas em destinationEdges.
                    Isso ocorre porque o vértice de destino pode ser trocado e se ele for trocado, usará
                    a mesma key do vértice de destino anterior, então fiz isso para não causar conflito.
                */
               
                adjacency_list[navigableVerticesMap[tuple].key].erase({navigableVerticesMap[globalGoalIndex].key, navigableVerticesMap[globalGoalIndex].key});
            }

            navigableVerticesMapInteger.erase(navigableVerticesMap[globalGoalIndex].key);
            
            navigableVerticesMap.erase(globalGoalIndex);
            
           
            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<float> duration = end_time - start_time_;        
            
            RCLCPP_INFO(this->get_logger(), "A* execution time: %.10f", duration.count());
        }
    }

    void callback_removed_navigable_vertices(const subdrone_interfaces::msg::PassarArrayVertices::SharedPtr msg)
    {
        
        
        for (const auto& vertex : msg->data) 
        {   
            auto index = std::make_tuple(roundToMultiple(vertex.x, distanceToObstacle_, decimals), roundToMultiple(vertex.y, distanceToObstacle_, decimals), roundToMultipleFromBase(vertex.z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals));
            if(navigableVerticesMap.find(index) != navigableVerticesMap.end())
            {

                if(relatedEdges.find(navigableVerticesMap[index].key) != relatedEdges.end())
                {
                    remove_edges(index);
                }
                
                    
                
                adjacency_list.erase(navigableVerticesMap[index].key);
                relatedEdges.erase(navigableVerticesMap[index].key);

                //navigableVerticesMapInteger.erase(navigableVerticesMap[index].key);
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
        auto new_resolution = static_cast<float>(this->get_parameter("resolution").get_parameter_value().get<int>());
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
        else if(new_resolution != temp_)
        {   
            temp_ = new_resolution;

            std::cout << "\n" << std::endl;

            if(new_resolution <= -1)
            {
                resolution_ =  std::abs(1.0  / new_resolution);
                RCLCPP_INFO(this->get_logger(), "Updated resolution: %lf", new_resolution);
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
     : Node("a_star")
    {
    
     
        this->declare_parameter<double>("distanceToObstacle", 0.05);
        this->declare_parameter<int>("diagonalEdges", 1);
        this->declare_parameter<int>("resolution", 1);
        this->declare_parameter<double>("x_min", -10.0);
        this->declare_parameter<double>("x_max", 10.0);
        this->declare_parameter<double>("y_min", -10.0);
        this->declare_parameter<double>("y_max", 10.0);
        this->declare_parameter<double>("z_min", 0.2);
        this->declare_parameter<double>("z_max", 0.2);

        // Initialize parameters 
        distanceToObstacle_ =  static_cast<float>(this->get_parameter("distanceToObstacle").get_parameter_value().get<double>());
        diagonalEdges_ = this->get_parameter("diagonalEdges").get_parameter_value().get<int>();
        resolution_ = static_cast<float>(this->get_parameter("resolution").get_parameter_value().get<int>());
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