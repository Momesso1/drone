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


class VerifyPath : public rclcpp::Node {

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
    int diagonalEdges_;
    float pose_x_ = 0.0, pose_y_ = 0.0, pose_z_ = 0.0;
    float distanceToObstacle_;
    float z_min_;
 
    int decimals = 0;

    std::tuple<float, float, float> globalGoalIndex;
    std::tuple<float, float, float> globalIndex;

    std::vector<std::tuple<float, float, float>> destinationEdges;
    std::vector<VertexDijkstra> destinationVertices;
    std::vector<VertexDijkstra> verticesDijkstra;
    std::vector<Edge> shortestPathEdges;
    

    std::unordered_map<int, std::vector<int>> adjacency_list;
    std::unordered_set<std::tuple<float, float, float>> obstaclesVertices;
    std::unordered_map<int, Vertex> navigableVerticesMapInteger;


    
    bool loadDataFromBinaryFile(
        const std::string& filename, 
        const KeyPair& key, 
        std::vector<std::tuple<float, float, float>>& resultVector) 
    {
        std::ifstream file(filename, std::ios::binary);
        if (!file.is_open()) {
            std::cerr << "Erro ao abrir o arquivo para leitura: " << filename << std::endl;
            return false;
        }
    
        // Ler o cabeçalho do arquivo
        FileHeader header;
        file.read(reinterpret_cast<char*>(&header), sizeof(FileHeader));
        if (file.fail()) {
            std::cerr << "Erro ao ler o cabeçalho do arquivo" << std::endl;
            return false;
        }
    
        // Calcular o endereço hash da chave
        uint32_t originalAddress = hashAddress(key, header.hashTableSize);
        uint32_t address = originalAddress;
        int probeCount = 0;
        HashEntry entry;
        
        while (probeCount < header.hashTableSize) {
            // Posicionar o ponteiro na entrada da tabela hash
            file.seekg(sizeof(FileHeader) + address * sizeof(HashEntry), std::ios::beg);
            if (file.fail()) {
                std::cerr << "Erro ao buscar entrada na tabela hash" << std::endl;
                return false;
            }
    
            // Ler a entrada da tabela hash
            file.read(reinterpret_cast<char*>(&entry), sizeof(HashEntry));
            if (file.fail()) {
                std::cerr << "Erro ao ler a entrada no endereço " << address << std::endl;
                return false;
            }
    
            // Se a entrada estiver vazia, significa que a chave não foi encontrada
            if (!entry.occupied) {
                std::cerr << "Chave não encontrada na tabela hash" << std::endl;
                return false;
            }
    
            // Verificar se a chave lida é a mesma que estamos procurando
            KeyPair storedKey = {
                {entry.key_first_x, entry.key_first_y, entry.key_first_z}, 
                {entry.key_second_x, entry.key_second_y, entry.key_second_z}
            };
    
            if (storedKey == key) {
                // Chave encontrada, vamos buscar os dados
                file.seekg(entry.dataOffset, std::ios::beg);
                if (file.fail()) {
                    std::cerr << "Erro ao posicionar no offset de dados" << std::endl;
                    return false;
                }
    
                // Ler o tamanho do vetor
                uint32_t vectorSize;
                file.read(reinterpret_cast<char*>(&vectorSize), sizeof(uint32_t));
                if (file.fail()) {
                    std::cerr << "Erro ao ler o tamanho do vetor" << std::endl;
                    return false;
                }
    
                // Ler os pontos do vetor
                resultVector.resize(vectorSize);
                for (uint32_t i = 0; i < vectorSize; ++i) {
                    float x, y, z;
                    file.read(reinterpret_cast<char*>(&x), sizeof(float));
                    file.read(reinterpret_cast<char*>(&y), sizeof(float));
                    file.read(reinterpret_cast<char*>(&z), sizeof(float));
                    
                    if (file.fail()) {
                        std::cerr << "Erro ao ler ponto do vetor" << std::endl;
                        return false;
                    }
                    resultVector[i] = std::make_tuple(x, y, z);
                }
    
                return true;
            }
    
            // Se houve colisão, aplicar a sondagem quadrática para encontrar a chave correta
            probeCount++;
            address = (originalAddress + probeCount * probeCount) % header.hashTableSize;
        }
    
        std::cerr << "Erro: Chave não encontrada após sondagem quadrática" << std::endl;
        return false;
    }






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



    /*
    
        CALLBACKS.

    */

    void callback_destinations(const geometry_msgs::msg::PoseArray::SharedPtr msg) 
    {
        destinationVertices.clear();
        for (const auto& pose_in : msg->poses) {
            VertexDijkstra destino;

            destino.x = pose_in.position.x;
            destino.y = pose_in.position.y;
            destino.z = pose_in.position.z;

            destino.orientation_x = pose_in.orientation.x;
            destino.orientation_y = pose_in.orientation.y;
            destino.orientation_z = pose_in.orientation.z;
            destino.orientation_w = pose_in.orientation.w;

            destinationVertices.push_back(destino);
        }
        
         
        if(!destinationVertices.empty())
        {
        
            float dx = pose_x_ - static_cast<float>(destinationVertices[i_].x);
            float dy = pose_y_ - static_cast<float>(destinationVertices[i_].y);
            float dz = pose_z_ - static_cast<float>(destinationVertices[i_].z);

            float distanciaAteODestino = sqrt(dx * dx + dy * dy + dz * dz);

            if(distanciaAteODestino <= distanceToObstacle_)
            {
                i_ = i_ + 1;
            }

            float roundedPoseX = roundToMultiple(pose_x_, distanceToObstacle_, decimals);
            float roundedPoseY = roundToMultiple(pose_y_, distanceToObstacle_, decimals);
            float roundedPoseZ = roundToMultiple(pose_z_, distanceToObstacle_, decimals);

            float roundedDestinationX = roundToMultiple(static_cast<float>(destinationVertices[i_].x), distanceToObstacle_, decimals);
            float roundedDestinationY = roundToMultiple(static_cast<float>(destinationVertices[i_].y), distanceToObstacle_, decimals);
            float roundedDestinationZ = roundToMultiple(static_cast<float>(destinationVertices[i_].z), distanceToObstacle_, decimals);
         
       
           
            if(i_ == destinationVertices.size())
            {
                i_ = 0;
            }
            
            auto start_time_ = std::chrono::high_resolution_clock::now();

            const std::string filename = "/home/momesso/autonomous/src/map/config/savedPaths2.bin";

            std::tuple<float, float, float> pose_tuple = {roundedPoseX, roundedPoseY, roundedPoseZ};
            std::tuple<float, float, float> destination_tuple = {roundedDestinationX, roundedDestinationY, roundedDestinationZ};

            std::pair<std::tuple<float, float, float>, std::tuple<float, float, float>> key = {pose_tuple, destination_tuple};

            std::cout << pose_tuple << std::endl;
            std::cout << destination_tuple << std::endl;

            std::vector<std::tuple<float, float, float>> result;
            
            bool found = loadDataFromBinaryFile(filename, key, result);        
            // Verificar o resultado
            if (!result.empty()) 
            {
                std::cout << "Vetor encontrado para a chave especificada. Contém " << result.size() << " pontos:" << std::endl;
                
              

                storeEdgesInPath(result);
            } 
            else 
            {
                std::cout << "Nenhum vetor encontrado para a chave especificada." << std::endl;
            }


            bool isValid = verifyFileContent(filename);

            if (isValid) {
               std::cout << "Verificação do arquivo concluída com sucesso!" << std::endl;
            } else {
                std::cout << "Falha na verificação do arquivo." << std::endl;
            }
           
           
            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<float> duration = end_time - start_time_;  


            RCLCPP_INFO(this->get_logger(), "A* execution time: %.10f", duration.count());
       
        }
    }

    




    bool verifyFileContent(const std::string& filename) {
        std::ifstream file(filename, std::ios::binary);
        if (!file.is_open()) {
            std::cerr << "Erro ao abrir o arquivo para verificação: " << filename << std::endl;
            return false;
        }
        
        // Verificar o tamanho do arquivo
        file.seekg(0, std::ios::end);
        std::streampos fileSize = file.tellg();
        file.seekg(0, std::ios::beg);
        
        std::cout << "Tamanho do arquivo: " << fileSize << " bytes" << std::endl;
        
        if (fileSize == 0) {
            std::cerr << "O arquivo está vazio" << std::endl;
            file.close();
            return false;
        }
        
        // Tentar ler o cabeçalho
        FileHeader header;
        file.read(reinterpret_cast<char*>(&header), sizeof(FileHeader));
        
        if (file.fail()) {
            std::cerr << "Erro ao ler o cabeçalho do arquivo" << std::endl;
            file.close();
            return false;
        }
        
        std::cout << "Informações do cabeçalho:" << std::endl;
        std::cout << "  Tamanho da tabela hash: " << header.hashTableSize << std::endl;
        std::cout << "  Número de entradas: " << header.numEntries << std::endl;
        std::cout << "  Offset inicial de dados: " << header.dataStartOffset << std::endl;
        
        // Verificar se os valores do cabeçalho são razoáveis
        if (header.hashTableSize == 0 || header.numEntries == 0 || 
            header.dataStartOffset < sizeof(FileHeader) ||
            header.dataStartOffset > fileSize) {
            std::cerr << "Cabeçalho contém valores inválidos" << std::endl;
            file.close();
            return false;
        }
        
        // Contar entradas ocupadas
        int occupiedEntries = 0;
        for (uint32_t i = 0; i < header.hashTableSize; ++i) {
            HashEntry entry;
            file.seekg(sizeof(FileHeader) + i * sizeof(HashEntry));
            file.read(reinterpret_cast<char*>(&entry), sizeof(HashEntry));
            
            if (file.fail()) {
                std::cerr << "Erro ao ler a entrada #" << i << std::endl;
                file.close();
                return false;
            }
            
            if (entry.occupied) {
                occupiedEntries++;
                
                // Verificar se o offset de dados é válido
                if (entry.dataOffset < header.dataStartOffset || entry.dataOffset >= fileSize) {
                    std::cerr << "Entrada #" << i << " tem offset de dados inválido: " << entry.dataOffset << std::endl;
                    continue;
                }
                
                // Verificar se podemos ler o tamanho do vetor
                file.seekg(entry.dataOffset);
                uint32_t vectorSize;
                file.read(reinterpret_cast<char*>(&vectorSize), sizeof(uint32_t));
                
                if (file.fail()) {
                    std::cerr << "Erro ao ler o tamanho do vetor para a entrada #" << i << std::endl;
                    continue;
                }
                if(entry.key_first_x == 0 && entry.key_first_y == 0 && entry.key_first_z == 0 && entry.key_second_x == 0 && entry.key_second_y == 0.25 && entry.key_second_z == 0.75)
                {
                    for(int i = 0; i < 15; i++)
                    {
                        std::cout << "Entrada #" << i << ":" << std::endl;
                        std::cout << "  Chave: (" 
                                  << entry.key_first_x << "," << entry.key_first_y << "," << entry.key_first_z << ") -> ("
                                  << entry.key_second_x << "," << entry.key_second_y << "," << entry.key_second_z << ")" << std::endl;
                        std::cout << "  Offset de dados: " << entry.dataOffset << std::endl;
                        std::cout << "  Tamanho de dados: " << entry.dataSize << std::endl;
                        std::cout << "  Pontos no vetor: " << vectorSize << std::endl;
                    }
                }
                
               
                
                // Verificar se o tamanho declarado é consistente
                uint32_t expectedSize = sizeof(uint32_t) + vectorSize * (3 * sizeof(float));
                if (entry.dataSize != expectedSize) {
                    std::cerr << "  Aviso: Tamanho declarado (" << entry.dataSize 
                              << ") não corresponde ao calculado (" << expectedSize << ")" << std::endl;
                }
                
                // Verificar se podemos ler alguns pontos
                // if (vectorSize > 0) {
                //     float x, y, z;
                //     file.read(reinterpret_cast<char*>(&x), sizeof(float));
                //     file.read(reinterpret_cast<char*>(&y), sizeof(float));
                //     file.read(reinterpret_cast<char*>(&z), sizeof(float));
                    
                //     if (!file.fail()) {
                //         std::cout << "  Primeiro ponto: (" << x << "," << y << "," << z << ")" << std::endl;
                //     } else {
                //         std::cerr << "  Erro ao ler o primeiro ponto" << std::endl;
                //     }
                // }
                
                // std::cout << "  ------------------------------" << std::endl;
            }
        }
        
        // std::cout << "Total de entradas ocupadas: " << occupiedEntries << " (esperado: " << header.numEntries << ")" << std::endl;
        
        file.close();
        return true;
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
    VerifyPath()
     : Node("a_star")
    {
    
     
        this->declare_parameter<double>("distanceToObstacle", 0.25);
        this->declare_parameter<int>("diagonalEdges", 3);


        // Initialize parameters 
        distanceToObstacle_ =  static_cast<float>(this->get_parameter("distanceToObstacle").get_parameter_value().get<double>());
        diagonalEdges_ = this->get_parameter("diagonalEdges").get_parameter_value().get<int>();


        RCLCPP_INFO(this->get_logger(), "Updated DistanceToObstacle: %f", distanceToObstacle_);
        RCLCPP_INFO(this->get_logger(), "Updated diagonalEdges: %d", diagonalEdges_);

        parameterTimer = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&VerifyPath::check_parameters, this));

        parameterTimer = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&VerifyPath::check_parameters, this));

        decimals = countDecimals(distanceToObstacle_);
       
 
        publisher_nav_path_ = this->create_publisher<nav_msgs::msg::Path>("visualize_path", 10);
        timer_visualize_path_ = this->create_wall_timer(100ms, std::bind(&VerifyPath::publisher_dijkstra_path, this));

        publisher_path_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/path", 10);
        timer_path_ = this->create_wall_timer(1ms, std::bind(&VerifyPath::publisher_dijkstra, this));
        

        subscription_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/rtabmap/odom", 10, std::bind(&VerifyPath::callback_odom, this, std::placeholders::_1));

        subscription3_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/destinations", 10, std::bind(&VerifyPath::callback_destinations, this, std::placeholders::_1));


       
    }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<VerifyPath>());
    rclcpp::shutdown();
    return 0;
}