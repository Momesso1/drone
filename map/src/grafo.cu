#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/pair.h>
#include <thrust/tuple.h>
#include <thrust/execution_policy.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <chrono>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <random>
#include <algorithm>
#include <memory>
#include <map>
#include <stack>
#include <optional>
#include <climits>
#include <iomanip>
#include <thread>
#include <queue>
#include <cmath>
#include <cstring>
#include <utility> 
// Using dirent.h instead of filesystem for better compatibility
#include <dirent.h>
#include <cstdint>
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



struct __align__(16) Point3D {
    float x, y, z;
    
    // Constructor
    __host__ __device__ Point3D(float x_val = 0, float y_val = 0, float z_val = 0) 
        : x(x_val), y(y_val), z(z_val) {}
    
    // Comparison operators
    __host__ __device__ bool operator==(const Point3D& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
    
    __host__ __device__ bool operator!=(const Point3D& other) const {
        return !(*this == other);
    }
    
    // Indexing operator
    __host__ __device__ float& operator[](int idx) {
        if (idx == 0) return x;
        if (idx == 1) return y;
        if (idx == 2) return z;
        return x; // Return x as a fallback
    }
    
    // Const indexing operator
    __host__ __device__ const float& operator[](int idx) const {
        if (idx == 0) return x;
        if (idx == 1) return y;
        if (idx == 2) return z;
        return x; // Return x as a fallback
    }
};

// Hash function for Point3D - simplified without relying on std::hash
struct Point3DHash {
    size_t operator()(const Point3D& p) const {
        // Manual hash combination to avoid std::hash
        size_t h1 = static_cast<size_t>(p.x * 73856093);
        size_t h2 = static_cast<size_t>(p.y * 19349663);
        size_t h3 = static_cast<size_t>(p.z * 83492791);
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

// Simplified std::hash specialization for Point3D
namespace std {
    template <>
    struct hash<Point3D> {
        std::size_t operator()(const Point3D& p) const {
            // Manual hash combination 
            size_t h1 = static_cast<size_t>(p.x * 73856093);
            size_t h2 = static_cast<size_t>(p.y * 19349663);
            size_t h3 = static_cast<size_t>(p.z * 83492791);
            return ((h1 * 31) + h2) * 31 + h3;
        }
    };
}

// Rounding functions for CUDA
__host__ __device__ inline float roundToMultiple(float value, float multiple, int decimals) 
{
    if (multiple == 0.0) return value; // Evita divisão por zero
    
    float result = std::round(value / multiple) * multiple;
    float factor = std::pow(10.0, decimals);
    result = std::round(result * factor) / factor;
    
    return result;
}
__host__ __device__ inline float roundToMultipleFromBase(float value, float base, float multiple, int decimals) 
{
    if (multiple == 0.0) return value; 
    
    float result = base + std::round((value - base) / multiple) * multiple;
    float factor = std::pow(10.0, decimals);
    result = std::round(result * factor) / factor;
    
    return result;
}



__device__ size_t atomicAdd(size_t* address, size_t val) {
    return (size_t)atomicAdd((unsigned long long*)address, (unsigned long long)val);
}

__global__ void generateValidPointsKernel(
    Point3D* validPoints, size_t* validPointCount,
    float x_min, float x_max, float y_min, float y_max, float z_min, float z_max,
    float distanceToObstacle, int decimals,
    Point3D* obstacles, size_t numObstacles, size_t maxPoints) {
    
    size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
    size_t stride = blockDim.x * gridDim.x;
    
    // Print debug info from first thread only
    if (idx == 0) {
        printf("Debug: x_min=%f, x_max=%f, distanceToObstacle=%f, decimals=%d\n",
               x_min, x_max, distanceToObstacle, decimals);
    }
    
    // Calculate the number of points in each dimension
    size_t total_z = ceilf((z_max - z_min) / distanceToObstacle) + 1;
    size_t total_y = ceilf((y_max - y_min) / distanceToObstacle) + 1;
    size_t total_x = ceilf((x_max - x_min) / distanceToObstacle) + 1;
    size_t total_points = total_z * total_y * total_x;
    
    if (idx == 0) {
        printf("Debug: total points to process: %zu\n", total_points);
    }
    
    for (size_t i = idx; i < total_points; i += stride) {
        // Convert linear index to 3D coordinates
        size_t z_idx = i / (total_x * total_y);
        size_t remainder = i % (total_x * total_y);
        size_t y_idx = remainder / total_x;
        size_t x_idx = remainder % total_x;
        
        // Calculate actual coordinates
        float z = z_min + z_idx * distanceToObstacle;
        float y = y_min + y_idx * distanceToObstacle;
        float x = x_min + x_idx * distanceToObstacle;
        
        // Debug printing for the first few points
        if (i < 5) {
            printf("Thread %zu processing point %zu: raw (%f, %f, %f)\n",
                   idx, i, x, y, z);
        }
        
        // Apply rounding to get consistent coordinates
        float new_x = roundToMultiple(x, distanceToObstacle, decimals);
        float new_y = roundToMultiple(y, distanceToObstacle, decimals);
        float new_z = roundToMultipleFromBase(z, roundToMultiple(z_min, distanceToObstacle, decimals),
                                             distanceToObstacle, decimals);
        
        // Debug printing after rounding
        if (i < 5) {
            printf("Thread %zu point %zu: rounded (%f, %f, %f)\n",
                   idx, i, new_x, new_y, new_z);
        }
        
        // Check if point is an obstacle
        bool isObstacle = false;
        for (size_t j = 0; j < numObstacles; j++) {  // Alterado obstacleCount para numObstacles
            if (new_x == obstacles[j].x && new_y == obstacles[j].y && new_z == obstacles[j].z) {
                isObstacle = true;
                break;
            }
        }
        
        // Only add non-obstacle points
        if (!isObstacle) {
            Point3D pt(new_x, new_y, new_z);
            size_t index = atomicAdd(validPointCount, 1);
            if (index < maxPoints) {
                validPoints[index] = pt;
                if (index < 5) {
                    printf("Added valid point %zu: (%f, %f, %f)\n",
                           index, pt.x, pt.y, pt.z);
                }
            }
        }
    }
}



// Device structure for A* algorithm
struct DeviceNode {
    Point3D point;
    Point3D parent;
    float g_score;
    float f_score;
    bool closed;
    
    __device__ DeviceNode() : 
        g_score(INFINITY), f_score(INFINITY), closed(false) {}
};

// Device function to get neighbor offsets
__device__ void getNeighborOffsets(float* offsets, int* offsetCount, float distance) {
    // Basic 26-direction neighborhood
    float directions[26][3] = {
        {0.0f, 0.0f, -distance},
        {-distance, 0.0f, 0.0f},
        {distance, 0.0f, 0.0f},
        {0.0f, -distance, 0.0f},
        {0.0f, distance, 0.0f},
        {0.0f, 0.0f, distance},
        {-distance, -distance, 0.0f},
        {-distance, distance, 0.0f},
        {distance, -distance, 0.0f},
        {distance, distance, 0.0f},
        {-distance, 0.0f, -distance},
        {distance, 0.0f, -distance},
        {0.0f, -distance, -distance},
        {0.0f, distance, -distance},
        {-distance, -distance, -distance},
        {-distance, distance, -distance},
        {distance, -distance, -distance},
        {distance, distance, -distance},
        {-distance, 0.0f, distance},
        {distance, 0.0f, distance},
        {0.0f, -distance, distance},
        {0.0f, distance, distance},
        {-distance, -distance, distance},
        {-distance, distance, distance},
        {distance, -distance, distance},
        {distance, distance, distance}
    };
    
    for (int i = 0; i < 26; i++) {
        offsets[i*3] = directions[i][0];
        offsets[i*3+1] = directions[i][1];
        offsets[i*3+2] = directions[i][2];
    }
    
    *offsetCount = 26;
}

__host__ __device__ float heuristic(const Point3D& a, const Point3D& b) {
    return sqrtf(powf(b.x - a.x, 2) + powf(b.y - a.y, 2) + powf(b.z - a.z, 2));
}


// Kernel for parallel A* algorithm
__global__ void parallelAStarKernel(
    Point3D* d_validPoints, size_t validPointCount,
    Point3D* d_obstacles, size_t obstacleCount,
    float distanceToObstacle, int decimals, float z_min,
    Point3D* d_paths, size_t* d_pathLengths, size_t maxPathLength,
    bool* d_pathSuccess, size_t startOffset, size_t batchSize) {
    
    size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= batchSize)
        return;
    
    size_t pathIdx = startOffset + idx;
    if (pathIdx >= (validPointCount * (validPointCount - 1)) / 2)
        return;
    
    // Convert linear index to pair indices (i,j) where j > i
    size_t i = 0, j = 0;
    size_t k = validPointCount - 1;
    size_t cumSum = 0;
    
    while (cumSum + k < pathIdx) {
        cumSum += k;
        i++;
        k--;
    }
    
    j = i + 1 + (pathIdx - cumSum);
    
    if (i >= validPointCount || j >= validPointCount || i == j) {
        // Invalid indices - shouldn't happen if math is correct
        d_pathSuccess[idx] = false;
        d_pathLengths[idx] = 0;
        return;
    }
    
    Point3D start = d_validPoints[i];
    Point3D goal = d_validPoints[j];
    
    // Initialize open and closed sets
    DeviceNode nodes[2048]; // Fixed size for simplicity - adjust based on max expected nodes
    size_t nodeCount = 0;
    
    // Add start node
    nodes[0].point = start;
    nodes[0].parent = start; // Parent of start is itself
    nodes[0].g_score = 0.0f;
    nodes[0].f_score = heuristic(start, goal);
    nodes[0].closed = false;
    nodeCount = 1;
    
    // Priority queue for open set (implemented as array)
    size_t openSet[1024]; // Max open set size
    size_t openCount = 1;
    openSet[0] = 0; // Index of start node
    
    // Get neighbor offsets
    float offsets[26 * 3]; // Store x,y,z offsets for 26 neighbors
    int offsetCount = 0;
    getNeighborOffsets(offsets, &offsetCount, distanceToObstacle);
    
    bool pathFound = false;
    size_t maxIterations = 500; // Prevent infinite loops
    size_t iterations = 0;
    
    while (openCount > 0 && iterations < maxIterations) {
        iterations++;
        
        // Find node with lowest f_score in open set
        size_t currentNodeIdx = openSet[0];
        float lowestF = nodes[currentNodeIdx].f_score;
        size_t lowestIdx = 0;
        
        for (size_t k = 1; k < openCount; k++) {
            if (nodes[openSet[k]].f_score < lowestF) {
                lowestF = nodes[openSet[k]].f_score;
                currentNodeIdx = openSet[k];
                lowestIdx = k;
            }
        }
        
        // Remove current from open set by swapping with last element
        openSet[lowestIdx] = openSet[openCount - 1];
        openCount--;
        
        DeviceNode& current = nodes[currentNodeIdx];
        current.closed = true;
        
        // Check if we reached the goal
        if (current.point.x == goal.x && current.point.y == goal.y && current.point.z == goal.z) {
            pathFound = true;
            break;
        }
        
        // Process neighbors
        for (size_t k = 0; k < offsetCount; k++) {
            // Calculate neighbor position
            float nx = roundToMultiple(current.point.x + offsets[k*3], distanceToObstacle, decimals);
            float ny = roundToMultiple(current.point.y + offsets[k*3+1], distanceToObstacle, decimals);
            float nz = roundToMultipleFromBase(current.point.z + offsets[k*3+2], 
                                         roundToMultiple(z_min, distanceToObstacle, decimals),
                                         distanceToObstacle, decimals);
            
            Point3D neighbor(nx, ny, nz);
            
            // Check if neighbor is an obstacle
            bool isObstacle = false;
            for (size_t o = 0; o < obstacleCount; o++) {
                if (nx == d_obstacles[o].x && ny == d_obstacles[o].y && nz == d_obstacles[o].z) {
                    isObstacle = true;
                    break;
                }
            }
            
            if (isObstacle)
                continue;
            
            // Skip if already in closed set
            bool inClosed = false;
            int neighborIdx = -1;
            
            for (size_t n = 0; n < nodeCount; n++) {
                if (nodes[n].point.x == nx && nodes[n].point.y == ny && nodes[n].point.z == nz) {
                    if (nodes[n].closed) {
                        inClosed = true;
                    }
                    neighborIdx = n;
                    break;
                }
            }
            
            if (inClosed)
                continue;
            
            // Calculate g score
            float tentative_g = current.g_score + heuristic(current.point, neighbor);
            
            // If neighbor not in open set, add it
            if (neighborIdx == -1) {
                if (nodeCount < 2048) { // Prevent buffer overflow
                    neighborIdx = nodeCount;
                    nodes[neighborIdx].point = neighbor;
                    nodes[neighborIdx].parent = current.point;
                    nodes[neighborIdx].g_score = tentative_g;
                    nodes[neighborIdx].f_score = tentative_g + heuristic(neighbor, goal);
                    nodes[neighborIdx].closed = false;
                    
                    nodeCount++;
                    
                    if (openCount < 1024) {
                        openSet[openCount] = neighborIdx;
                        openCount++;
                    }
                }
            }
            // If neighbor is in open set, check if this path is better
            else if (tentative_g < nodes[neighborIdx].g_score) {
                nodes[neighborIdx].parent = current.point;
                nodes[neighborIdx].g_score = tentative_g;
                nodes[neighborIdx].f_score = tentative_g + heuristic(neighbor, goal);
            }
        }
    }
    
    // Reconstruct path if found
    if (pathFound) {
        // Start with goal and trace back to start
        Point3D traceback[1024]; // Temp array for path
        size_t traceLength = 0;
        
        Point3D current = goal;
        traceback[traceLength++] = current;
        
        // Find parent of goal
        
        for (size_t n = 0; n < nodeCount; n++) {
            if (nodes[n].point.x == goal.x && nodes[n].point.y == goal.y && nodes[n].point.z == goal.z) {
                current = nodes[n].parent;
                break;
            }
        }
        
        // Trace parents until we reach start
        while (!(current.x == start.x && current.y == start.y && current.z == start.z) && traceLength < 1024) {
            traceback[traceLength++] = current;
            
            bool foundParent = false;
            for (size_t n = 0; n < nodeCount; n++) {
                if (nodes[n].point.x == current.x && nodes[n].point.y == current.y && nodes[n].point.z == current.z) {
                    current = nodes[n].parent;
                    foundParent = true;
                    break;
                }
            }
            
            if (!foundParent) break;
        }
        
        // Add start point
        traceback[traceLength++] = start;
        
        // Reverse the path (start to goal)
        if (traceLength <= maxPathLength) {
            for (size_t p = 0; p < traceLength; p++) {
                d_paths[idx * maxPathLength + p] = traceback[traceLength - 1 - p];
            }
            d_pathLengths[idx] = traceLength;
            d_pathSuccess[idx] = true;
        } else {
            // Path too long for buffer
            d_pathLengths[idx] = 0;
            d_pathSuccess[idx] = false;
        }
    } else {
        // No path found
        d_pathLengths[idx] = 0;
        d_pathSuccess[idx] = false;
    }
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

// Define a simple tuple class to avoid std::tuple
struct Point3DTuple {
    float x, y, z;
    
    Point3DTuple(float x_val = 0, float y_val = 0, float z_val = 0) 
        : x(x_val), y(y_val), z(z_val) {}
    
    bool operator==(const Point3DTuple& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

// Define a hash function for the Point3DTuple
struct Point3DTupleHash {
    size_t operator()(const Point3DTuple& p) const {
        size_t h1 = static_cast<size_t>(p.x * 73856093);
        size_t h2 = static_cast<size_t>(p.y * 19349663);
        size_t h3 = static_cast<size_t>(p.z * 83492791);
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

// Define a pair of points
struct PointPair {
    Point3DTuple first;
    Point3DTuple second;
    
    PointPair(const Point3DTuple& p1, const Point3DTuple& p2) 
        : first(p1), second(p2) {}
    
    bool operator==(const PointPair& other) const {
        return first == other.first && second == other.second;
    }
};

// Define a hash function for the PointPair
struct PointPairHash {
    size_t operator()(const PointPair& pair) const {
        Point3DTupleHash hasher;
        size_t h1 = hasher(pair.first);
        size_t h2 = hasher(pair.second);
        return h1 ^ (h2 << 1);
    }
};

// Use Point3DTuple and related structures instead of std::tuple
typedef std::unordered_map<PointPair, std::vector<Point3DTuple>, PointPairHash> PathMap;
PathMap startToEnd;

// Update the storeEdgesInPath function to use our custom structures
void storeEdgesInPath(const std::vector<Point3D>& path) {
    if (path.empty()) {
        return;
    }
    
    // Convert Point3D to tuple
    auto pointToTuple = [](const Point3D& p) -> Point3DTuple {
        // Round to decimal places to avoid floating point comparison issues
        auto round = [](float val, int decimals) -> float {
            float factor = powf(10.0f, decimals);
            return roundf(val * factor) / factor;
        };
        
        return Point3DTuple(
            round(p.x, 4), 
            round(p.y, 4), 
            round(p.z, 4)
        );
    };
    
    // Create key pair from first and last point
    PointPair pair(pointToTuple(path.front()), pointToTuple(path.back()));
    PointPair pair2(pointToTuple(path.back()), pointToTuple(path.front()));

    // Convert the entire path to tuples for storage
    std::vector<Point3DTuple> tuplePath;
    std::vector<Point3DTuple> tuplePath2;

    for (const auto& point : path) {
        tuplePath.push_back(pointToTuple(point));
    }

    //inverta path aqui

    // Store the path (replace any existing path between these points)
    startToEnd[pair] = tuplePath;
    
    // Debug output to verify paths are being stored
    static int pathCounter = 0;
    if (++pathCounter % 1000 == 0) {
        std::cout << "Stored " << pathCounter << " paths. Map size: " << startToEnd.size() << std::endl;
    }
}


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

// Modified to match the existing PointPair structure instead of using std::tuple
uint32_t hashAddress(const PointPair& key, uint32_t tableSize) 
{
    PointPairHash hasher;
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
bool saveMapToBinaryFile(const std::string& filename)
{
    // Verificar se o mapa está vazio
    if (startToEnd.empty()) {
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
    uint32_t hashTableSize = recommendedHashTableSize(startToEnd.size() * 2); // Dobrar o tamanho para evitar colisões
    
    // Escrever o cabeçalho do arquivo
    FileHeader header;
    header.hashTableSize = hashTableSize;
    header.numEntries = startToEnd.size();
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
    
    // Variável para contar colisões
    int collisionCount = 0;
    
    // Cria um arquivo de texto adicional para armazenar as chaves formatadas com espaço após as vírgulas
    std::ofstream formattedKeysFile(filename + ".txt");
    if (!formattedKeysFile.is_open()) {
        // Continuar mesmo assim, pois este é apenas um arquivo auxiliar
    }
    
    // Processar todas as entradas do mapa
    int entryCount = 0;
    for (const auto& pair : startToEnd) 
    {
        entryCount++;
        
        // Calcular o endereço hash para esta chave
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
                break;
            } else {
                collisionCount++;
                probeCount++;
                // Sondagem quadrática: o deslocamento é o quadrado do número de tentativas
                address = (originalAddress + probeCount * probeCount) % hashTableSize;
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
        // FIX: Access Point3DTuple directly instead of using std::tie
        entry.key_first_x = pair.first.first.x;
        entry.key_first_y = pair.first.first.y;
        entry.key_first_z = pair.first.first.z;
        entry.key_second_x = pair.first.second.x;
        entry.key_second_y = pair.first.second.y;
        entry.key_second_z = pair.first.second.z;
        entry.dataOffset = currentDataOffset;
        
        // Salvar os dados formatados no arquivo de texto auxiliar
        if (formattedKeysFile.is_open()) {
            formattedKeysFile << "Índice: " << address << ", Chave: (" 
                << pair.first.first.x << ", " << pair.first.first.y << ", " << pair.first.first.z << ") -> ("
                << pair.first.second.x << ", " << pair.first.second.y << ", " << pair.first.second.z << ")" << std::endl;
            
            formattedKeysFile << "  Offset: " << currentDataOffset 
                << ", Pontos: " << pair.second.size() << std::endl;
            
            // Também podemos salvar os pontos no arquivo de texto
            int pointIndex = 0;
            for (const auto& point : pair.second) {
                formattedKeysFile << "    Ponto " << pointIndex++ << ": (" 
                    << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
            }
        }
        
        // Posicionar o ponteiro de escrita no offset de dados correto
        file.seekp(currentDataOffset, std::ios::beg);
        if (file.fail()) {
            file.close();
            formattedKeysFile.close();
            return false;
        }
        
        // Escrever o tamanho do vetor
        uint32_t vectorSize = pair.second.size();
        file.write(reinterpret_cast<char*>(&vectorSize), sizeof(uint32_t));
        if (file.fail()) {
            file.close();
            formattedKeysFile.close();
            return false;
        }
        
        // Escrever cada ponto no vetor
        for (const auto& point : pair.second) {
            // FIX: Access Point3DTuple fields directly
            float x = point.x;
            float y = point.y;
            float z = point.z;
            
            file.write(reinterpret_cast<const char*>(&x), sizeof(float));
            file.write(reinterpret_cast<const char*>(&y), sizeof(float));
            file.write(reinterpret_cast<const char*>(&z), sizeof(float));
            
            if (file.fail()) {
                file.close();
                formattedKeysFile.close();
                return false;
            }
        }
        
        // Calcular o tamanho total dos dados para esta entrada
        entry.dataSize = sizeof(uint32_t) + vectorSize * (3 * sizeof(float));
        currentDataOffset += entry.dataSize;
        
        // Escrever a entrada atualizada na tabela hash
        file.seekp(sizeof(FileHeader) + address * sizeof(HashEntry), std::ios::beg);
        if (file.fail()) {
            file.close();
            formattedKeysFile.close();
            return false;
        }
        
        file.write(reinterpret_cast<const char*>(&entry), sizeof(HashEntry));
        if (file.fail()) {
            file.close();
            formattedKeysFile.close();
            return false;
        }
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
    
    if (fileSize == 0 || fileSize < currentDataOffset) {
        file.close();
        return false;
    }
    
    file.close();
    
    // Verificar se o arquivo existe e tem o tamanho esperado
    std::ifstream checkFile(filename, std::ios::binary);
    if (!checkFile.is_open()) {
        return false;
    }
    
    checkFile.seekg(0, std::ios::end);
    std::streampos checkSize = checkFile.tellg();
    checkFile.close();
    
    if (checkSize == 0) {
        std::cerr << "Erro: O arquivo está vazio após a gravação" << std::endl;
        return false;
    }
    
    std::cout << "Arquivo salvo com sucesso: " << filename << std::endl;
    std::cout << "Entradas gravadas: " << entryCount << " de " << startToEnd.size() << std::endl;
    std::cout << "Arquivo de texto com chaves formatadas: " << filename << ".txt" << std::endl;
    
    return true;
}


// Main function optimized for CUDA with memory fixes
// Main function optimized for CUDA with memory fixes
void trainAStarCUDA() {
    const std::string filename = "/home/momesso/autonomous/src/map/config/savedPaths2.bin";  // Caminho relativo para salvar os dados
    std::cout << "Starting A* optimization with CUDA." << std::endl;
    
    // Parâmetros do espaço de busca
    float x_min_ = -3.0f, x_max_ = 3.0f;
    float y_min_ = -3.0f, y_max_ = 0.0f;
    float z_min_ = 0.0f,    z_max_ = 3.0f;
    float distanceToObstacle_ = 0.3f;
    int decimals = 1;
    
    // Obstáculos (exemplo)
    std::unordered_set<Point3D, Point3DHash> obstaclesVertices;
   
    
    // Converte obstáculos para array
    std::vector<Point3D> obstaclesArray(obstaclesVertices.begin(), obstaclesVertices.end());
    
    // Configuração CUDA
    size_t threadBlockSize = 1024;
    size_t numBlocks = (x_max_ - x_min_) * (y_max_ - y_min_) * (z_max_ - z_min_) / 
                    (distanceToObstacle_ * distanceToObstacle_ * distanceToObstacle_ * threadBlockSize) + 1;
    numBlocks = std::min(numBlocks, static_cast<size_t>(65535));
    
    // Cálculo dos pontos no espaço
    size_t numStepsX = static_cast<size_t>(std::floor((x_max_ - x_min_) / distanceToObstacle_)) + 2;
    size_t numStepsY = static_cast<size_t>(std::floor((y_max_ - y_min_) / distanceToObstacle_)) + 2;
    size_t numStepsZ = static_cast<size_t>(std::floor((z_max_ - z_min_) / distanceToObstacle_)) + 2;
    size_t maxPoints = numStepsX * numStepsY * numStepsZ;
    
    std::cout << "Maximum possible points: " << maxPoints << std::endl;
    
    // === PHASE 1: Geração de pontos válidos ===
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Aloca e copia obstáculos para a GPU
    Point3D* d_obstacles;
    cudaMalloc(&d_obstacles, obstaclesArray.size() * sizeof(Point3D));
    cudaMemcpy(d_obstacles, obstaclesArray.data(), obstaclesArray.size() * sizeof(Point3D), cudaMemcpyHostToDevice);
    
    // Aloca memória para pontos válidos
    Point3D* d_validPoints;
    size_t* d_validPointCount;
    cudaMalloc(&d_validPoints, maxPoints * sizeof(Point3D));
    cudaMalloc(&d_validPointCount, sizeof(size_t));
    
    size_t initialCount = 0;
    cudaMemcpy(d_validPointCount, &initialCount, sizeof(size_t), cudaMemcpyHostToDevice);
    
    // Lança kernel para gerar pontos válidos
    generateValidPointsKernel<<<numBlocks, threadBlockSize>>>(
        d_validPoints, d_validPointCount,
        x_min_, x_max_, y_min_, y_max_, z_min_, z_max_,
        distanceToObstacle_, decimals,
        d_obstacles, obstaclesArray.size(), maxPoints);
    
    cudaError_t cudaStatus = cudaGetLastError();
    if (cudaStatus != cudaSuccess) {
        std::cerr << "generateValidPointsKernel failed: " << cudaGetErrorString(cudaStatus) << std::endl;
        cudaFree(d_validPoints);
        cudaFree(d_validPointCount);
        cudaFree(d_obstacles);
        return;
    }
    
    cudaStatus = cudaDeviceSynchronize();
    if (cudaStatus != cudaSuccess) {
        std::cerr << "cudaDeviceSynchronize failed: " << cudaGetErrorString(cudaStatus) << std::endl;
        cudaFree(d_validPoints);
        cudaFree(d_validPointCount);
        cudaFree(d_obstacles);
        return;
    }
    
    // Recupera número de pontos válidos e copia para host
    size_t validPointCount;
    cudaMemcpy(&validPointCount, d_validPointCount, sizeof(size_t), cudaMemcpyDeviceToHost);
    std::cout << "Valid points generated: " << validPointCount << std::endl;
    
    std::vector<Point3D> validPoints(validPointCount);
    cudaMemcpy(validPoints.data(), d_validPoints, validPointCount * sizeof(Point3D), cudaMemcpyDeviceToHost);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    std::cout << "Point generation time: " << duration << " ms" << std::endl;
    
    // === PHASE 2: Cálculo dos caminhos entre os pontos ===
    start_time = std::chrono::high_resolution_clock::now();
    
    size_t totalPathPairs = (validPointCount * (validPointCount - 1)) / 2;
    std::cout << "Total path pairs to calculate: " << totalPathPairs << std::endl;
    
    const size_t batchSize = 8192;
    const size_t maxPathLength = 8192;
    
    size_t processedPaths = 0; 
    size_t successfulPaths = 0;
    
    // Aloca buffers para os caminhos na GPU (uma única vez)
    Point3D* d_paths;
    size_t* d_pathLengths;
    bool* d_pathSuccess;
    
    cudaStatus = cudaMalloc(&d_paths, batchSize * maxPathLength * sizeof(Point3D));
    if (cudaStatus != cudaSuccess) {
        std::cerr << "cudaMalloc failed for paths: " << cudaGetErrorString(cudaStatus) << std::endl;
        return;
    }
    cudaStatus = cudaMalloc(&d_pathLengths, batchSize * sizeof(size_t));
    if (cudaStatus != cudaSuccess) {
        std::cerr << "cudaMalloc failed for path lengths: " << cudaGetErrorString(cudaStatus) << std::endl;
        cudaFree(d_paths);
        return;
    }
    cudaStatus = cudaMalloc(&d_pathSuccess, batchSize * sizeof(bool));
    if (cudaStatus != cudaSuccess) {
        std::cerr << "cudaMalloc failed for path success flags: " << cudaGetErrorString(cudaStatus) << std::endl;
        cudaFree(d_paths);
        cudaFree(d_pathLengths);
        return;
    }
    
    // Aloca buffers de host para os resultados dos caminhos
    std::vector<Point3D> h_paths(batchSize * maxPathLength);
    std::vector<size_t> h_pathLengths(batchSize);
    // Usamos vector<char> para contornar o problema com vector<bool>
    std::vector<char> h_pathSuccess(batchSize);
    
    // Loop de processamento dos caminhos
    while (processedPaths < totalPathPairs) {
        size_t currentBatchSize = std::min(batchSize, totalPathPairs - processedPaths);
        
        // Lança o kernel de busca de caminhos
        parallelAStarKernel<<<(currentBatchSize + threadBlockSize - 1) / threadBlockSize, threadBlockSize>>>(
            d_validPoints, validPointCount,
            d_obstacles, obstaclesArray.size(),
            distanceToObstacle_, decimals, z_min_,
            d_paths, d_pathLengths, maxPathLength,
            d_pathSuccess, processedPaths, currentBatchSize);
        
        cudaStatus = cudaGetLastError();
        if (cudaStatus != cudaSuccess) {
            std::cerr << "parallelAStarKernel failed: " << cudaGetErrorString(cudaStatus) << std::endl;
            break;
        }
        
        cudaStatus = cudaDeviceSynchronize();
        if (cudaStatus != cudaSuccess) {
            std::cerr << "cudaDeviceSynchronize failed: " << cudaGetErrorString(cudaStatus) << std::endl;
            break;
        }
        
        // Copia os resultados do dispositivo para a memória do host
        cudaMemcpy(h_pathLengths.data(), d_pathLengths, currentBatchSize * sizeof(size_t), cudaMemcpyDeviceToHost);
        cudaMemcpy(h_pathSuccess.data(), d_pathSuccess, currentBatchSize * sizeof(bool), cudaMemcpyDeviceToHost);
        cudaMemcpy(h_paths.data(), d_paths, currentBatchSize * maxPathLength * sizeof(Point3D), cudaMemcpyDeviceToHost);
        
        // Processa os resultados dos caminhos
        for (size_t i = 0; i < currentBatchSize; i++) {
            // Como usamos vector<char>, qualquer valor diferente de zero é considerado true
            if (h_pathSuccess[i]) {
                std::vector<Point3D> path;
                for (size_t j = 0; j < h_pathLengths[i]; j++) {
                    path.push_back(h_paths[i * maxPathLength + j]);
                }
                storeEdgesInPath(path);  // Armazena o caminho em startToEnd
                successfulPaths++;
            }
        }
        
        processedPaths += currentBatchSize;
        float percentage = (processedPaths * 100.0f) / totalPathPairs;
        std::cout << "Processados " << processedPaths << " de " << totalPathPairs
                  << " caminhos (" << percentage << "%). "
                  << "Bem-sucedidos: " << successfulPaths << std::endl;
    }
    
    // Libera os buffers alocados para os caminhos
    cudaFree(d_paths);
    cudaFree(d_pathLengths);
    cudaFree(d_pathSuccess);
    
    end_time = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    std::cout << "Path calculation time: " << duration << " ms" << std::endl;
    
    // Salva os caminhos em arquivo
    saveMapToBinaryFile(filename);
    
    // Clean up dos outros buffers
    cudaFree(d_validPoints);
    cudaFree(d_validPointCount);
    cudaFree(d_obstacles);
    
    float percentage = (processedPaths * 100.0f) / totalPathPairs;
    std::cout << "Processados " << processedPaths << " de " << totalPathPairs
              << " caminhos (" << percentage << "%). "
              << "Bem-sucedidos: " << successfulPaths << std::endl;
    std::cout << "A* optimization complete." << std::endl;
}

int main(int argc, char** argv) {
    // Initialize CUDA
    int deviceCount = 0;
    cudaGetDeviceCount(&deviceCount);
    
    if (deviceCount == 0) {
        std::cerr << "No CUDA devices found." << std::endl;
        return 1;
    }
    
    // Print device info
    cudaDeviceProp deviceProp;
    cudaGetDeviceProperties(&deviceProp, 0);
    std::cout << "Using CUDA device: " << deviceProp.name << std::endl;
    
    trainAStarCUDA();
    
    return 0;
}