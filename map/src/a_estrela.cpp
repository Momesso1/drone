#include <string>
#include <random>
#include <algorithm>
#include <geometry_msgs/msg/point.hpp>
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <chrono>
#include <functional>
#include <memory>
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
#include <fstream>
#include <cstdint>

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
    

    struct TupleHash 
    {
        size_t operator()(const std::tuple<float, float, float>& t) const {
            return std::hash<float>()(std::get<0>(t)) ^ 
                (std::hash<float>()(std::get<1>(t)) << 1) ^ 
                (std::hash<float>()(std::get<2>(t)) << 2);
        }
    };


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

    struct PairTupleHash {
        std::size_t operator()(const std::pair<std::tuple<float, float, float>, 
                            std::tuple<float, float, float>>& p) const {
            auto hash1 = std::hash<float>{}(std::get<0>(p.first)) ^ 
                        (std::hash<float>{}(std::get<1>(p.first)) << 1) ^ 
                        (std::hash<float>{}(std::get<2>(p.first)) << 2);
            
            auto hash2 = std::hash<float>{}(std::get<0>(p.second)) ^ 
                        (std::hash<float>{}(std::get<1>(p.second)) << 1) ^ 
                        (std::hash<float>{}(std::get<2>(p.second)) << 2);
            
            return hash1 ^ (hash2 << 1);
        }
    };

    struct PairHashTuple {
        template <typename T>
        std::size_t hash_tuple(const T& t) const {
            std::size_t seed = 0;
            auto hash_combine = [&seed](auto& v) {
                seed ^= std::hash<std::decay_t<decltype(v)>>{}(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            };
            std::apply([&](auto&&... args) { (hash_combine(args), ...); }, t);
            return seed;
        }
    
        std::size_t operator()(const std::pair<std::tuple<float, float, float>, std::tuple<float, float, float>>& p) const {
            std::size_t h1 = hash_tuple(p.first);
            std::size_t h2 = hash_tuple(p.second);
            return h1 ^ (h2 << 1); // Combinação dos hashes
        }
    };

    struct PairTupleEqual {
        bool operator()(const std::pair<std::tuple<float, float, float>, 
                    std::tuple<float, float, float>>& p1,
                    const std::pair<std::tuple<float, float, float>, 
                    std::tuple<float, float, float>>& p2) const {
            return p1.first == p2.first && p1.second == p2.second;
        }
    };

   
    struct FileHeader {
        uint32_t hashTableSize; 
        uint32_t numEntries;     
        uint32_t dataStartOffset; 
    };

   
    struct HashEntry {
        bool occupied;  
        float key_first_x, key_first_y, key_first_z;
        float key_second_x, key_second_y, key_second_z;
        uint32_t dataOffset; 
        uint32_t dataSize;   
    };


    using Point3D = std::tuple<float, float, float>;
    using KeyPair = std::pair<Point3D, Point3D>;
    using PointVector = std::vector<Point3D>;
    using ComplexMap = std::unordered_map<KeyPair, PointVector, PairTupleHash, PairTupleEqual>;


    uint32_t hashAddress(const KeyPair& key, uint32_t tableSize) 
    {
        PairTupleHash hasher;
        return hasher(key) % tableSize;
    }

    uint32_t recommendedHashTableSize(uint32_t numItems) 
    {
        static const uint32_t primes[] = 
        {
            53, 97, 193, 389, 769, 1543, 3079, 6151, 12289, 24593, 49157, 
            98317, 196613, 393241, 786433, 1572869, 3145739, 6291469
        };
        
        uint32_t target = numItems * 3 / 2;
        
        for (uint32_t prime : primes) {
            if (prime > target) {
                return prime;
            }
        }
        
        return primes[sizeof(primes) / sizeof(primes[0]) - 1];
    }


    

 
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
    std::unordered_set<std::tuple<float, float, float>> obstaclesVertices;
    std::unordered_map<int, Vertex> navigableVerticesMapInteger;


   
    bool saveMapToBinaryFile(const std::unordered_map<std::pair<std::tuple<float, float, float>, std::tuple<float, float, float>>, std::vector<std::tuple<float, float, float>>,  PairHashTuple> map, const std::string& filename) 
    {
        std::fstream file(filename, std::ios::in | std::ios::out | std::ios::binary | std::ios::trunc);
        if (!file) {
            std::cerr << "Erro ao abrir o arquivo para escrita: " << filename << std::endl;
            return false;
        }
        
        uint32_t hashTableSize = recommendedHashTableSize(map.size());
        
        FileHeader header;
        header.hashTableSize = hashTableSize;
        header.numEntries = map.size();
        header.dataStartOffset = sizeof(FileHeader) + hashTableSize * sizeof(HashEntry);
        
        file.write(reinterpret_cast<char*>(&header), sizeof(FileHeader));
        
        HashEntry emptyEntry = {false, 0, 0, 0, 0, 0, 0, 0, 0};
        for (uint32_t i = 0; i < hashTableSize; ++i) {
            file.write(reinterpret_cast<const char*>(&emptyEntry), sizeof(HashEntry));
        }
        
        uint32_t currentDataOffset = header.dataStartOffset;
        
        for (const auto& pair : map) {
            uint32_t address = hashAddress(pair.first, hashTableSize);
            uint32_t originalAddress = address;
            
            HashEntry entry;
            bool positionFound = false;
            
            while (!positionFound) {
                file.seekg(sizeof(FileHeader) + address * sizeof(HashEntry));
                file.read(reinterpret_cast<char*>(&entry), sizeof(HashEntry));
                
                if (!entry.occupied) {
                    positionFound = true;
                } else {
                    address = (address + 1) % hashTableSize;
                    if (address == originalAddress) {
                        std::cerr << "Erro: Tabela hash no arquivo está cheia" << std::endl;
                        file.close();
                        return false;
                    }
                }
            }
            
            entry.occupied = true;
            auto [key_first_x, key_first_y, key_first_z] = pair.first.first;
            auto [key_second_x, key_second_y, key_second_z] = pair.first.second;
            
            entry.key_first_x = key_first_x;
            entry.key_first_y = key_first_y;
            entry.key_first_z = key_first_z;
            entry.key_second_x = key_second_x;
            entry.key_second_y = key_second_y;
            entry.key_second_z = key_second_z;
            entry.dataOffset = currentDataOffset;
            
            file.seekp(currentDataOffset);
            
            uint32_t vectorSize = pair.second.size();
            file.write(reinterpret_cast<char*>(&vectorSize), sizeof(uint32_t));
            
            for (const auto& point : pair.second) {
                float x, y, z;
                std::tie(x, y, z) = point;
                file.write(reinterpret_cast<const char*>(&x), sizeof(float));
                file.write(reinterpret_cast<const char*>(&y), sizeof(float));
                file.write(reinterpret_cast<const char*>(&z), sizeof(float));
            }
            
            entry.dataSize = sizeof(uint32_t) + vectorSize * (3 * sizeof(float));
            currentDataOffset += entry.dataSize;
            
            file.seekp(sizeof(FileHeader) + address * sizeof(HashEntry));
            file.write(reinterpret_cast<const char*>(&entry), sizeof(HashEntry));
        }
        
        file.close();
        return true;
    }

    PointVector loadVectorByKey(const std::string& filename, const KeyPair& key) {
        std::ifstream file(filename, std::ios::binary);
        if (!file) {
            std::cerr << "Erro ao abrir o arquivo para leitura: " << filename << std::endl;
            return {};
        }
        
        FileHeader header;
        file.read(reinterpret_cast<char*>(&header), sizeof(FileHeader));
        
        uint32_t address = hashAddress(key, header.hashTableSize);
        uint32_t originalAddress = address;
        
        while (true) {
            file.seekg(sizeof(FileHeader) + address * sizeof(HashEntry));
            
            HashEntry entry;
            file.read(reinterpret_cast<char*>(&entry), sizeof(HashEntry));
            
            if (!entry.occupied) {
                file.close();
                return {};
            }
            
            Point3D first(entry.key_first_x, entry.key_first_y, entry.key_first_z);
            Point3D second(entry.key_second_x, entry.key_second_y, entry.key_second_z);
            KeyPair fileKey(first, second);
            
            if (fileKey == key) {
                file.seekg(entry.dataOffset);
                
                uint32_t vectorSize;
                file.read(reinterpret_cast<char*>(&vectorSize), sizeof(uint32_t));
                
                PointVector result;
                result.reserve(vectorSize);  
                
                for (uint32_t i = 0; i < vectorSize; ++i) {
                    float x, y, z;
                    file.read(reinterpret_cast<char*>(&x), sizeof(float));
                    file.read(reinterpret_cast<char*>(&y), sizeof(float));
                    file.read(reinterpret_cast<char*>(&z), sizeof(float));
                    
                    result.emplace_back(x, y, z);
                }
                
                file.close();
                return result;
            }
            
            address = (address + 1) % header.hashTableSize;
            
            if (address == originalAddress) {
                break;
            }
        }
        
        file.close();
        return {}; 
    }

    void viewHashTable(const std::string& filename) 
    {
        std::ifstream file(filename, std::ios::binary);
        if (!file) {
            std::cerr << "Erro ao abrir o arquivo: " << filename << std::endl;
            return;
        }
        
        FileHeader header;
        file.read(reinterpret_cast<char*>(&header), sizeof(FileHeader));
        
        std::cout << "Tamanho da tabela hash: " << header.hashTableSize << std::endl;
        std::cout << "Número de entradas: " << header.numEntries << std::endl;
        std::cout << "Início dos dados: " << header.dataStartOffset << std::endl;
        
        for (uint32_t i = 0; i < header.hashTableSize; ++i) {
            HashEntry entry;
            file.read(reinterpret_cast<char*>(&entry), sizeof(HashEntry));
            
            if (entry.occupied) {
                std::cout << "Posição " << i << ": ";
                std::cout << "Chave: ((" 
                        << entry.key_first_x << ", " << entry.key_first_y << ", " << entry.key_first_z << "), ("
                        << entry.key_second_x << ", " << entry.key_second_y << ", " << entry.key_second_z << ")) ";
                std::cout << "Dados em: " << entry.dataOffset << " (tamanho: " << entry.dataSize << ")" << std::endl;
            }
        }
        
        file.close();
    }








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

    bool indexExistsInFile(const std::string& filename, const std::tuple<float, float, float>& index) 
    {
        std::ifstream file(filename, std::ios::binary);
        if (!file.is_open()) return false;
        
        size_t tableSize;
        file.read(reinterpret_cast<char*>(&tableSize), sizeof(size_t));
        
        size_t hash = TupleHash()(index) % tableSize;
        
        size_t bytePos = hash / 8;
        uint8_t bitPos = hash % 8;
        
        file.seekg(sizeof(size_t) + bytePos, std::ios::beg);
        
        uint8_t byte;
        file.read(reinterpret_cast<char*>(&byte), 1);
        
        bool exists = (byte & (1 << bitPos)) != 0;
        
        file.close();
        return exists;
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
        
        const std::string filename = "/home/momesso/autonomous/src/map/config/obstacles.bin";


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
        for(int i = 1; i <= 3; i++)
        {
            for (int a = 0; a < 26; a++) 
            {
                new_x = roundToMultiple(std::get<0>(start_tuple) + (offsets1[a][0] * i), distanceToObstacle_, decimals);
                new_y = roundToMultiple(std::get<1>(start_tuple) + (offsets1[a][1] * i), distanceToObstacle_, decimals);
                new_z = roundToMultipleFromBase(std::get<2>(start_tuple) + (offsets1[a][2] * i), 
                    roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                
                auto neighbor_tuple = std::make_tuple(static_cast<float>(new_x), 
                    static_cast<float>(new_y), 
                    static_cast<float>(new_z));
                
                if(!indexExistsInFile(filename, neighbor_tuple))
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
                new_z = roundToMultipleFromBase(std::get<2>(goal_tuple) + (offsets1[a][2] * i),
                    roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                
                auto neighbor_tuple1 = std::make_tuple(static_cast<float>(new_x), 
                    static_cast<float>(new_y), 
                    static_cast<float>(new_z));
                
                if(!indexExistsInFile(filename, neighbor_tuple1))
                { 
                    adjacency_list_tuples[neighbor_tuple1].push_back(goal_tuple);
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
            
            if (current != start_tuple && current != goal_tuple)
            {
                for (int a = 0; a < 26; a++) 
                {
                    new_x = roundToMultiple(std::get<0>(current) + offsets1[a][0], distanceToObstacle_, decimals);
                    new_y = roundToMultiple(std::get<1>(current) + offsets1[a][1], distanceToObstacle_, decimals);
                    new_z = roundToMultipleFromBase(std::get<2>(current) + offsets1[a][2],
                        roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                    
                    auto neighbor_tuple = std::make_tuple(static_cast<float>(new_x), 
                        static_cast<float>(new_y), 
                        static_cast<float>(new_z));
                    if(!indexExistsInFile(filename, neighbor_tuple))
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
                        new_z2 = roundToMultipleFromBase(std::get<2>(current), roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        auto index = std::make_tuple(static_cast<float>(new_x2), static_cast<float>(new_y2), static_cast<float>(new_z2));

                        new_x = roundToMultiple(std::get<0>(current) + (distanceToObstacle_* (i - 1)), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(std::get<1>(current) + distanceToObstacle_, distanceToObstacle_, decimals);
                        new_z = roundToMultipleFromBase(std::get<2>(current), roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                        
                        auto index1 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));
                        
                        new_x = roundToMultiple(std::get<0>(current) + (distanceToObstacle_ * i), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(std::get<1>(current), distanceToObstacle_, decimals);
                        
                        
                        auto index2 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));

                        new_x = roundToMultiple(std::get<0>(current) + (distanceToObstacle_ * (i - 1)), distanceToObstacle_, decimals);
                        new_y = roundToMultiple(std::get<1>(current), distanceToObstacle_, decimals);
                        
                        
                        auto index3 = std::make_tuple(static_cast<float>(new_x), static_cast<float>(new_y), static_cast<float>(new_z));
                       
                        if(pode1 == true) 
                        {
                            if(indexExistsInFile(filename, index) || indexExistsInFile(filename, index1)  || indexExistsInFile(filename, index2)|| indexExistsInFile(filename, index3))
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
                            if(indexExistsInFile(filename, index) || indexExistsInFile(filename, index1)  || indexExistsInFile(filename, index2)|| indexExistsInFile(filename, index3))
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
                            if(indexExistsInFile(filename, index) || indexExistsInFile(filename, index1)  || indexExistsInFile(filename, index2)|| indexExistsInFile(filename, index3))
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
                            if(indexExistsInFile(filename, index) || indexExistsInFile(filename, index1)  || indexExistsInFile(filename, index2)|| indexExistsInFile(filename, index3))
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
                            if(indexExistsInFile(filename, index) || indexExistsInFile(filename, index1)  || indexExistsInFile(filename, index2)|| indexExistsInFile(filename, index3))
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
                            if(indexExistsInFile(filename, index) || indexExistsInFile(filename, index1)  || indexExistsInFile(filename, index2)|| indexExistsInFile(filename, index3))
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
                            if(indexExistsInFile(filename, index) || indexExistsInFile(filename, index1)  || indexExistsInFile(filename, index2)|| indexExistsInFile(filename, index3))
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
                            if(indexExistsInFile(filename, index) || indexExistsInFile(filename, index1)  || indexExistsInFile(filename, index2)|| indexExistsInFile(filename, index3))
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

  

    void storeEdgesInPath(const std::vector<std::tuple<float, float, float>>& path) 
    {
        shortestPathEdges.clear();
        verticesDijkstra.clear();
        const std::string filename = "/home/momesso/autonomous/src/map/config/savedPaths.bin";
        
        
    
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

        std::vector<std::tuple<float, float, float>> reversedPath = path;
        std::reverse(reversedPath.begin(), reversedPath.end());
       
        std::pair<std::tuple<float, float, float>, std::tuple<float, float, float>> pair1 = std::make_pair(path[0], reversedPath[0]);
        std::pair<std::tuple<float, float, float>, std::tuple<float, float, float>> pair2 = std::make_pair(reversedPath[0], path[0]);
    
        

       
        // Processar os vértices do caminho
        for (size_t i = 0; i < path.size(); i++) 
        {
            
            VertexDijkstra vertex;
            std::unordered_map<std::pair<std::tuple<float, float, float>, std::tuple<float, float, float>>, std::vector<std::tuple<float, float, float>>,  PairHashTuple> startToEnd;

           

            startToEnd[pair1].push_back(path[i]);
            startToEnd[pair2].push_back(reversedPath[i]);

            saveMapToBinaryFile(startToEnd, filename);
            std::cout << "path: " <<  path[i] << std::endl;
            std::cout << "reversed path: " <<  reversedPath[i] << std::endl;

            
            
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
            std::vector<std::tuple<float, float, float>> shortestPath = runAStar(array_inicial, array_final);
           
            storeEdgesInPath(shortestPath);
           
            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<float> duration = end_time - start_time_;  

            adjacency_list.clear();

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
    
     
        this->declare_parameter<double>("distanceToObstacle", 0.2);
        this->declare_parameter<int>("diagonalEdges", 3);


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