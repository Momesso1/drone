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
#include <omp.h>
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

    struct PairHash1 {
        std::size_t operator()(const std::pair<std::tuple<float, float, float>, std::tuple<float, float, float>>& p) const {
            std::size_t h1 = std::hash<float>{}(std::get<0>(p.first)) ^ 
                            (std::hash<float>{}(std::get<1>(p.first)) << 1) ^
                            (std::hash<float>{}(std::get<2>(p.first)) << 2);
            
            std::size_t h2 = std::hash<float>{}(std::get<0>(p.second)) ^
                            (std::hash<float>{}(std::get<1>(p.second)) << 1) ^
                            (std::hash<float>{}(std::get<2>(p.second)) << 2);
                            
            return h1 ^ (h2 << 1);
        }
    };

    struct PairEqual {
        bool operator()(const std::pair<std::tuple<float, float, float>, std::tuple<float, float, float>>& p1,
                        const std::pair<std::tuple<float, float, float>, std::tuple<float, float, float>>& p2) const {
            return std::get<0>(p1.first) == std::get<0>(p2.first) &&
                   std::get<1>(p1.first) == std::get<1>(p2.first) &&
                   std::get<2>(p1.first) == std::get<2>(p2.first) &&
                   std::get<0>(p1.second) == std::get<0>(p2.second) &&
                   std::get<1>(p1.second) == std::get<1>(p2.second) &&
                   std::get<2>(p1.second) == std::get<2>(p2.second);
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
    std::unordered_map<std::pair<std::tuple<float, float, float>, std::tuple<float, float, float>>, std::vector<std::tuple<float, float, float>>,  PairHashTuple> startToEnd;


    bool saveMapToBinaryFile(
        const std::unordered_map<
            std::pair<std::tuple<float, float, float>, std::tuple<float, float, float>>,
            std::vector<std::tuple<float, float, float>>,
            PairHash1, PairEqual>& map, 
        const std::string& filename)
    {
        // Verificar se o mapa está vazio
        if (map.empty()) {
            std::cerr << "Erro: O mapa está vazio" << std::endl;
            return false;
        }

        // Abrir o arquivo com verificação explícita
        std::fstream file(filename, std::ios::in | std::ios::out | std::ios::binary | std::ios::trunc);
        if (!file.is_open()) {
            std::cerr << "Erro ao abrir o arquivo para escrita: " << filename << std::endl;
            return false;
        }

        // Verificar permissões de escrita
        file << "test";
        if (file.fail()) {
            std::cerr << "Erro: Não foi possível escrever no arquivo. Verifique as permissões." << std::endl;
            file.close();
            return false;
        }
        
        // Reiniciar o arquivo
        file.close();
        file.open(filename, std::ios::in | std::ios::out | std::ios::binary | std::ios::trunc);
        if (!file.is_open()) {
            std::cerr << "Erro ao reabrir o arquivo: " << filename << std::endl;
            return false;
        }
        
        // Calcular o tamanho da tabela hash
        uint32_t hashTableSize = recommendedHashTableSize(map.size() * 2); // Dobrar o tamanho para evitar colisões
        // std::cout << "Tamanho da tabela hash: " << hashTableSize << std::endl;
        // std::cout << "Número de entradas no mapa: " << map.size() << std::endl;
        
        // Escrever o cabeçalho do arquivo
        FileHeader header;
        header.hashTableSize = hashTableSize;
        header.numEntries = map.size();
        header.dataStartOffset = sizeof(FileHeader) + hashTableSize * sizeof(HashEntry);
        file.write(reinterpret_cast<char*>(&header), sizeof(FileHeader));
        
        if (file.fail()) {
            std::cerr << "Erro ao escrever o cabeçalho" << std::endl;
            file.close();
            return false;
        }
        
        // Inicializar a tabela hash com entradas vazias
        HashEntry emptyEntry = {false, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0};
        for (uint32_t i = 0; i < hashTableSize; ++i) {
            file.write(reinterpret_cast<const char*>(&emptyEntry), sizeof(HashEntry));
            if (file.fail()) {
                std::cerr << "Erro ao escrever a entrada vazia #" << i << std::endl;
                file.close();
                return false;
            }
        }
        
        // Verificar a posição atual do arquivo
        std::streampos expectedOffset = sizeof(FileHeader) + hashTableSize * sizeof(HashEntry);
        if (file.tellp() != expectedOffset) {
            std::cerr << "Erro: Offset inesperado após escrever entradas vazias. Esperado: " 
                    << expectedOffset << ", Atual: " << file.tellp() << std::endl;
            file.close();
            return false;
        }
        
        // Posição atual no arquivo onde os dados começarão a ser escritos
        uint32_t currentDataOffset = header.dataStartOffset;
        // std::cout << "Offset inicial de dados: " << currentDataOffset << std::endl;
        
        // Variável para contar colisões
        int collisionCount = 0;
        
        // Cria um arquivo de texto adicional para armazenar as chaves formatadas com espaço após as vírgulas
        std::ofstream formattedKeysFile(filename + ".txt");
        if (!formattedKeysFile.is_open()) {
            // std::cerr << "Aviso: Não foi possível criar o arquivo de texto para chaves formatadas" << std::endl;
            // Continuar mesmo assim, pois este é apenas um arquivo auxiliar
        }
        
        // Processar todas as entradas do mapa
        int entryCount = 0;
        for (const auto& pair : map) 
        {
            entryCount++;
            // std::cout << "Processando entrada #" << entryCount << "/" << map.size() << std::endl;
            
            // Calcular o endereço hash para esta chave
            // Dentro do loop que processa cada entrada do map:
            uint32_t originalAddress = hashAddress(pair.first, hashTableSize);
            uint32_t address = originalAddress;
            bool positionFound = false;
            int probeCount = 0;
            HashEntry entry;

            while (!positionFound && probeCount < hashTableSize) {
                // Posicionar o ponteiro para ler a entrada na posição calculada
                file.seekg(sizeof(FileHeader) + address * sizeof(HashEntry), std::ios::beg);
                if (file.fail()) {
                    std::cerr << "Erro ao posicionar o ponteiro para leitura no endereço " << address << std::endl;
                    file.close();
                    formattedKeysFile.close();
                    return false;
                }
                
                file.read(reinterpret_cast<char*>(&entry), sizeof(HashEntry));
                if (file.fail()) {
                    std::cerr << "Erro ao ler a entrada no endereço " << address << std::endl;
                    file.close();
                    formattedKeysFile.close();
                    return false;
                }
                
                if (!entry.occupied) {
                    positionFound = true;
                    // std::cout << "Posição livre encontrada no endereço " << address << std::endl;
                    break;
                } else {
                    collisionCount++;
                    probeCount++;
                    // Sondagem quadrática: o deslocamento é o quadrado do número de tentativas
                    address = (originalAddress + probeCount * probeCount) % hashTableSize;
                    // std::cout << "Colisão detectada. Tentativa " << probeCount 
                    //           << ", novo endereço: " << address << std::endl;
                }
            }

            if (!positionFound) {
                std::cerr << "Erro: Não foi possível encontrar uma posição livre após " 
                        << probeCount << " tentativas" << std::endl;
                file.close();
                formattedKeysFile.close();
                return false;
            }
                        
            // Preencher os dados da entrada
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
            
            // Exibir os dados no console com espaços após as vírgulas
            // std::cout << "Dados da chave: (" 
            //         << key_first_x << ", " << key_first_y << ", " << key_first_z << ") -> ("
            //         << key_second_x << ", " << key_second_y << ", " << key_second_z << ")" << std::endl;
            // std::cout << "Offset de dados: " << currentDataOffset << std::endl;
            
            // Salvar os dados formatados no arquivo de texto auxiliar
            if (formattedKeysFile.is_open()) {
                formattedKeysFile << "Índice: " << address << ", Chave: (" 
                    << key_first_x << ", " << key_first_y << ", " << key_first_z << ") -> ("
                    << key_second_x << ", " << key_second_y << ", " << key_second_z << ")" << std::endl;
                
                formattedKeysFile << "  Offset: " << currentDataOffset 
                    << ", Pontos: " << pair.second.size() << std::endl;
                
                // Também podemos salvar os pontos no arquivo de texto
                int pointIndex = 0;
                for (const auto& point : pair.second) {
                    float x, y, z;
                    std::tie(x, y, z) = point;
                    formattedKeysFile << "    Ponto " << pointIndex++ << ": (" 
                        << x << ", " << y << ", " << z << ")" << std::endl;
                }
                // formattedKeysFile << "  ------------------------------" << std::endl;
            }
            
            // Posicionar o ponteiro de escrita no offset de dados correto
            file.seekp(currentDataOffset, std::ios::beg);
            if (file.fail()) {
                // std::cerr << "Erro ao posicionar o ponteiro para escrita no offset " << currentDataOffset << std::endl;
                file.close();
                formattedKeysFile.close();
                return false;
            }
            
            // Escrever o tamanho do vetor
            uint32_t vectorSize = pair.second.size();
            file.write(reinterpret_cast<char*>(&vectorSize), sizeof(uint32_t));
            if (file.fail()) {
                // std::cerr << "Erro ao escrever o tamanho do vetor: " << vectorSize << std::endl;
                file.close();
                formattedKeysFile.close();
                return false;
            }
            
            // std::cout << "Tamanho do vetor: " << vectorSize << " pontos" << std::endl;
            
            // Escrever cada ponto no vetor
            for (const auto& point : pair.second) {
                float x, y, z;
                std::tie(x, y, z) = point;
                file.write(reinterpret_cast<const char*>(&x), sizeof(float));
                file.write(reinterpret_cast<const char*>(&y), sizeof(float));
                file.write(reinterpret_cast<const char*>(&z), sizeof(float));
                
                if (file.fail()) {
                    // std::cerr << "Erro ao escrever o ponto (" << x << ", " << y << ", " << z << ")" << std::endl;
                    file.close();
                    formattedKeysFile.close();
                    return false;
                }
            }
            
            // Calcular o tamanho total dos dados para esta entrada
            entry.dataSize = sizeof(uint32_t) + vectorSize * (3 * sizeof(float));
            currentDataOffset += entry.dataSize;
            
            // std::cout << "Tamanho dos dados: " << entry.dataSize << " bytes" << std::endl;
            // std::cout << "Próximo offset de dados: " << currentDataOffset << std::endl;
            
            // Escrever a entrada atualizada na tabela hash
            file.seekp(sizeof(FileHeader) + address * sizeof(HashEntry), std::ios::beg);
            if (file.fail()) {
                // std::cerr << "Erro ao posicionar o ponteiro para escrita na tabela hash, endereço " << address << std::endl;
                file.close();
                formattedKeysFile.close();
                return false;
            }
            
            file.write(reinterpret_cast<const char*>(&entry), sizeof(HashEntry));
            if (file.fail()) {
                // std::cerr << "Erro ao escrever a entrada na tabela hash" << std::endl;
                file.close();
                formattedKeysFile.close();
                return false;
            }
            
            // std::cout << "Entrada #" << entryCount << " gravada com sucesso" << std::endl;
            // std::cout << "------------------------------" << std::endl;
        }
        
        // Fechar o arquivo de texto formatado
        if (formattedKeysFile.is_open()) {
            formattedKeysFile.close();
        }
        
        // Forçar a gravação em disco
        file.flush();
        
        // Verificar o tamanho final do arquivo
        file.seekp(0, std::ios::end);
        std::streampos fileSize = file.tellp();
        // std::cout << "Tamanho final do arquivo: " << fileSize << " bytes" << std::endl;
        
        if (fileSize == 0 || fileSize < currentDataOffset) {
            // std::cerr << "Erro: Tamanho do arquivo inconsistente" << std::endl;
            file.close();
            return false;
        }
        
        file.close();
        
        // Verificar se o arquivo existe e tem o tamanho esperado
        std::ifstream checkFile(filename, std::ios::binary);
        if (!checkFile.is_open()) {
            // std::cerr << "Erro: Não foi possível abrir o arquivo para verificação" << std::endl;
            return false;
        }
        
        checkFile.seekg(0, std::ios::end);
        std::streampos checkSize = checkFile.tellg();
        checkFile.close();
        
        // std::cout << "Verificação final do arquivo: " << checkSize << " bytes" << std::endl;
        
        if (checkSize == 0) {
            std::cerr << "Erro: O arquivo está vazio após a gravação" << std::endl;
            return false;
        }
        
        std::cout << "Arquivo salvo com sucesso: " << filename << std::endl;
        std::cout << "Entradas gravadas: " << entryCount << " de " << map.size() << std::endl;
        std::cout << "Arquivo de texto com chaves formatadas: " << filename << ".txt" << std::endl;
        
        return true;
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
   
    
    void trainAStar() 
    {
        const std::string filename = "/home/momesso/autonomous/src/map/config/savedPaths2.bin";
        std::cout << "Iniciando otimização do A*." << std::endl;
    
        // VERIFICAÇÃO DO AMBIENTE OPENMP
        #ifdef _OPENMP
            std::cout << "OpenMP está habilitado na compilação." << std::endl;
        #else
            std::cout << "OpenMP NÃO está habilitado na compilação!" << std::endl;
        #endif
        
        // Força explicitamente todas as 24 threads
        omp_set_num_threads(24);
        
        // VERIFICA SE O AMBIENTE CONSEGUE USAR MAIS DE UMA THREAD
        int max_threads = omp_get_max_threads();
        std::cout << "Threads máximas disponíveis: " << max_threads << std::endl;
        
        if (max_threads <= 1) {
            std::cout << "AVISO: Seu ambiente permite apenas 1 thread!" << std::endl;
            std::cout << "Verifique se a variável OMP_NUM_THREADS está definida corretamente." << std::endl;
        }
        
        // Testa se realmente consegue usar múltiplas threads
        int thread_count = 0;
        #pragma omp parallel
        {
            #pragma omp atomic
            thread_count++;
        }
        std::cout << "Threads realmente ativadas: " << thread_count << std::endl;
        
        // Se não conseguiu usar todas as threads, vamos forçar manualmente
        if (thread_count < max_threads) {
            std::cout << "FORÇANDO paralelização manual com threads" << std::endl;
        }
    
        // 1. Pré-calcule todos os pontos válidos (não-obstáculo)
        std::vector<std::tuple<float, float, float>> validPoints;
        
        auto start_phase1 = std::chrono::high_resolution_clock::now();
        
        // Coletando pontos válidos
        for (float z = z_min_; z <= z_max_; z += distanceToObstacle_) {
            for (float y = y_min_; y <= y_max_; y += distanceToObstacle_) {
                for (float x = x_min_; x <= x_max_; x += distanceToObstacle_) {
                    float new_x = roundToMultiple(x, distanceToObstacle_, decimals);
                    float new_y = roundToMultiple(y, distanceToObstacle_, decimals);
                    float new_z = roundToMultipleFromBase(z, roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
    
                    // Normaliza valores zero, se necessário
                    if(new_x == -0.0f)
                        new_x = 0.0f;
                    if(new_y == -0.0f)
                        new_y = 0.0f;
                    if(new_z == -0.0f)
                        new_z = 0.0f;
    
                    auto pt = std::make_tuple(new_x, new_y, new_z);
                    if (obstaclesVertices.find(pt) == obstaclesVertices.end()) {
                        validPoints.push_back(pt);
                    }
                }
            }
        }
        
        auto end_phase1 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration_phase1 = end_phase1 - start_phase1;
        std::cout << "Fase 1 concluída em: " << duration_phase1.count() << " segundos" << std::endl;
        std::cout << "Total de pontos válidos: " << validPoints.size() << std::endl;
    
        // Declaração do mapa global para armazenar os caminhos
        std::unordered_map<
            std::pair<std::tuple<float, float, float>, std::tuple<float, float, float>>,
            std::vector<std::tuple<float, float, float>>,
            PairHash1, PairEqual> startToEnd;
    
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Definir número de threads (usar quantidade de threads disponíveis)
        int num_threads = std::max(24, max_threads);
        std::cout << "Iniciando processamento com " << num_threads << " threads" << std::endl;
        
        // NOVA ABORDAGEM: Fila de trabalho compartilhada
        // Criar uma fila de tarefas
        std::queue<size_t> task_queue;
        std::mutex queue_mutex;
        std::mutex map_mutex;
        std::mutex cout_mutex;
        std::atomic<bool> processing_complete(false);
        std::atomic<int> total_edges_found(0);
        
        // Preencher a fila com os índices dos pontos a serem processados
        for (size_t i = 0; i < validPoints.size(); i++) {
            task_queue.push(i);
        }
        
        // Função que será executada por cada thread
        auto worker_function = [&](int thread_id) {
            auto thread_start = std::chrono::high_resolution_clock::now();
            int local_edges_found = 0;
            int points_processed = 0;
            
            // Mapa local para esta thread
            std::unordered_map<
                std::pair<std::tuple<float, float, float>, std::tuple<float, float, float>>,
                std::vector<std::tuple<float, float, float>>,
                PairHash1, PairEqual> local_map;
            
            // Log de início
            {
                std::lock_guard<std::mutex> lock(cout_mutex);
                std::cout << "Thread " << thread_id << " iniciada" << std::endl;
            }
            
            while (!processing_complete) {
                // Obter próxima tarefa da fila
                size_t point_index;
                bool has_task = false;
                
                {
                    std::lock_guard<std::mutex> lock(queue_mutex);
                    if (!task_queue.empty()) {
                        point_index = task_queue.front();
                        task_queue.pop();
                        has_task = true;
                    }
                }
                
                // Se não há mais tarefas, sair do loop
                if (!has_task) {
                    break;
                }
                
                // Log para monitoramento (reduzido para evitar excesso de logs)
                if (points_processed % 50 == 0) {
                    std::lock_guard<std::mutex> lock(cout_mutex);
                    std::cout << "Thread " << thread_id << " processando ponto " << point_index << std::endl;
                }
                
                int edge_count = 0;
                
                // Comparar com todos os outros pontos (após o índice atual)
                for (size_t j = point_index + 1; j < validPoints.size(); j++) {
                    float array_inicial[3] = { 
                        std::get<0>(validPoints[point_index]), 
                        std::get<1>(validPoints[point_index]), 
                        std::get<2>(validPoints[point_index]) 
                    };
                    
                    float array_final[3] = { 
                        std::get<0>(validPoints[j]), 
                        std::get<1>(validPoints[j]), 
                        std::get<2>(validPoints[j]) 
                    };
    
                    std::vector<std::tuple<float, float, float>> shortestPath = runAStar(array_inicial, array_final);
                    
                    // Se um caminho válido foi encontrado, armazena no mapa local
                    if (!shortestPath.empty()) {
                        auto edge = std::make_pair(validPoints[point_index], validPoints[j]);
                        local_map[edge] = shortestPath;
                        edge_count++;
                    }
                }
                
                // Incrementar contadores
                local_edges_found += edge_count;
                points_processed++;
                
                // Log do progresso (a cada 10 pontos para reduzir número de logs)
                if (points_processed % 10 == 0) {
                    std::lock_guard<std::mutex> lock(cout_mutex);
                    std::cout << "Thread " << thread_id << ", processou ponto " << point_index 
                            << ": Arestas encontradas neste ponto: " << edge_count 
                            << ", Total local: " << local_edges_found << std::endl;
                }
                
                // Mesclar periodicamente para liberar memória
                if (local_map.size() > 1000) {
                    std::lock_guard<std::mutex> lock(map_mutex);
                    for (const auto& edge : local_map) {
                        startToEnd[edge.first] = edge.second;
                    }
                    local_map.clear();
                }
            }
            
            // Mesclar resultados finais
            {
                std::lock_guard<std::mutex> lock(map_mutex);
                for (const auto& edge : local_map) {
                    startToEnd[edge.first] = edge.second;
                }
                total_edges_found += local_edges_found;
            }
            
            // Log de finalização
            auto thread_end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> thread_duration = thread_end - thread_start;
            
            {
                std::lock_guard<std::mutex> lock(cout_mutex);
                std::cout << "Thread " << thread_id << " finalizada em " << thread_duration.count() 
                        << " segundos, processou " << points_processed << " pontos, "
                        << "encontrou " << local_edges_found << " arestas." << std::endl;
            }
        };
        
        // Criar e iniciar as threads
        std::vector<std::thread> threads;
        for (int t = 0; t < num_threads; t++) {
            threads.push_back(std::thread(worker_function, t));
        }
        
        // Aguardar até que a fila esteja vazia
        while (true) {
            {
                std::lock_guard<std::mutex> lock(queue_mutex);
                if (task_queue.empty()) {
                    processing_complete = true;
                    break;
                }
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            // Status do processamento
            std::lock_guard<std::mutex> lock(cout_mutex);
            std::cout << "Tarefas restantes na fila: ";
            {
                std::lock_guard<std::mutex> qlock(queue_mutex);
                std::cout << task_queue.size() << std::endl;
            }
        }
        
        // Aguardar todas as threads finalizarem
        std::cout << "Aguardando conclusão de " << threads.size() << " threads..." << std::endl;
        for (auto& thread : threads) {
            thread.join();
        }
        std::cout << "Todas as threads concluídas." << std::endl;
        
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end_time - start_time;
        std::cout << "Processamento concluído em: " << duration.count() << " segundos" << std::endl;
        std::cout << "Total de caminhos encontrados: " << startToEnd.size() << std::endl;
        std::cout << "Total de arestas encontradas (contador atômico): " << total_edges_found << std::endl;
        
        // Salvar os caminhos em um arquivo
        saveMapToBinaryFile(startToEnd, filename);
        std::cout << "Caminhos salvos em: " << filename << std::endl;
    }

        

    std::vector<std::tuple<float, float, float>> runAStar(float start[3], float goal[3]) 
    {
        
        
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
        
        for(int i = 1; i <= 2; i++)
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
        
        bool findNavigableGoalVertice = false;
        
        for(int i = 1; i <= 2; i++)
        {
            for (int a = 0; a < 26; a++) 
            {
                new_x = roundToMultiple(std::get<0>(goal_tuple) + (offsets1[a][0] * i), distanceToObstacle_, decimals);
                new_y = roundToMultiple(std::get<1>(goal_tuple) + (offsets1[a][1] * i), distanceToObstacle_, decimals);
                new_z = roundToMultipleFromBase(std::get<2>(goal_tuple) + (offsets1[a][2] * i),
                    roundToMultiple(z_min_, distanceToObstacle_, decimals), distanceToObstacle_, decimals);
                
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

        std::pair<std::tuple<float, float, float>, std::tuple<float, float, float>> pair1 = std::make_pair(path[0], path[path.size() - 1]);
    
        
       
        // Processar os vértices do caminho
        for (size_t i = 0; i < path.size(); i++) 
        {
            
            VertexDijkstra vertex;

           

            startToEnd[pair1].push_back(path[i]);

            
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
        }

        if(new_diagonalEdges != diagonalEdges_)
        {
            diagonalEdges_ = new_diagonalEdges;

            std::cout << "\n" << std::endl;

            RCLCPP_INFO(this->get_logger(), "Updated diagonalEdges: %d", diagonalEdges_);
        }

        if (new_x_min != x_min_) 
        {
            std::cout << "\n" << std::endl;
            x_min_ = new_x_min;
            RCLCPP_INFO(this->get_logger(), "Updated x_min: %.2f", x_min_);
            trainAStar();
        }
        if (new_x_max != x_max_) 
        {
            std::cout << "\n" << std::endl;
            x_max_ = new_x_max;
            RCLCPP_INFO(this->get_logger(), "Updated x_max: %.2f", x_max_);
            trainAStar();
        }
        if (new_y_min != y_min_) 
        {
            std::cout << "\n" << std::endl;
            y_min_ = new_y_min;
            RCLCPP_INFO(this->get_logger(), "Updated y_min: %.2f", y_min_);
            trainAStar();
        }
        if (new_y_max != y_max_) 
        {
            std::cout << "\n" << std::endl;
           y_max_ = new_y_max;
            RCLCPP_INFO(this->get_logger(), "Updated y_max: %.2f", y_max_);
            trainAStar();
        }        
        if (new_z_min != z_min_) 
        {
            std::cout << "\n" << std::endl;
            z_min_ = new_z_min;
            RCLCPP_INFO(this->get_logger(), "Updated z_min: %.2f", z_min_);
            trainAStar();
        }
        if (new_z_max != z_max_) 
        {
            std::cout << "\n" << std::endl;
            z_max_ = new_z_max;
            RCLCPP_INFO(this->get_logger(), "Updated z_max: %.2f", z_max_);
            trainAStar();
        }
       
        

      
    }
    
   
public:
    AStar()
     : Node("a_star")
    {
    
     
        this->declare_parameter<double>("distanceToObstacle", 0.2);
        this->declare_parameter<int>("diagonalEdges", 3);
        this->declare_parameter<double>("x_min", -1.0);
        this->declare_parameter<double>("x_max", 1.0);
        this->declare_parameter<double>("y_min", -1.0);
        this->declare_parameter<double>("y_max", 1.0);
        this->declare_parameter<double>("z_min", 0.0);
        this->declare_parameter<double>("z_max", 0.4);


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
        
       
 
        publisher_nav_path_ = this->create_publisher<nav_msgs::msg::Path>("visualize_path", 10);
        timer_visualize_path_ = this->create_wall_timer(100ms, std::bind(&AStar::publisher_dijkstra_path, this));

        publisher_path_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/path", 10);
        timer_path_ = this->create_wall_timer(1ms, std::bind(&AStar::publisher_dijkstra, this));
        

        subscription_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/rtabmap/odom", 10, std::bind(&AStar::callback_odom, this, std::placeholders::_1));

        
        // const std::string filename = "/home/momesso/autonomous/src/map/config/obstacles.bin";

       

        // loadVerticesFromFile(filename);
        trainAStar();
    }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<AStar>());
    rclcpp::shutdown();
    return 0;
}