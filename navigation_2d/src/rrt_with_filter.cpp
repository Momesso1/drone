#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vector>
#include <cmath>
#include <random>
#include <memory>
#include <algorithm>
#include <unordered_set>
#include <tuple>

// Hash function for tuples to use in unordered_set
namespace std {
    template<>
    struct hash<tuple<float, float, float>> {
        size_t operator()(const tuple<float, float, float>& t) const {
            return hash<float>()(get<0>(t)) ^ hash<float>()(get<1>(t)) ^ hash<float>()(get<2>(t));
        }
    };
}

class RRTNode 
{
    public:
        double x;
        double y;
        RRTNode* parent;

        RRTNode(double x_pos, double y_pos) : x(x_pos), y(y_pos), parent(nullptr) {}
};

class RRTNormal : public rclcpp::Node {
public:
    RRTNormal() : Node("rrt_normal_cpp_node") 
    {
        this->declare_parameter<double>("distanceToObstacle", 0.2);
        this->declare_parameter<int>("diagonalEdges", 3);
        this->declare_parameter<double>("minimumHeight", 0.01);
        this->declare_parameter<double>("maximumHeight", 0.4);
        this->declare_parameter<bool>("visualOdometry", false);
        this->declare_parameter("max_iterations", 50000);
        this->declare_parameter("goal_sample_rate", 0.1);

        distanceToObstacle_ = static_cast<float>(this->get_parameter("distanceToObstacle").get_parameter_value().get<double>());
        diagonalEdges_ = this->get_parameter("diagonalEdges").get_parameter_value().get<int>();
        minimumHeight = static_cast<float>(this->get_parameter("minimumHeight").get_parameter_value().get<double>());
        maximumHeight = static_cast<float>(this->get_parameter("maximumHeight").get_parameter_value().get<double>());
        visualOdometry = this->get_parameter("visualOdometry").get_parameter_value().get<bool>();
        max_iterations_ = this->get_parameter("max_iterations").as_int();
        goal_sample_rate_ = this->get_parameter("goal_sample_rate").as_double();


        RCLCPP_INFO(this->get_logger(), "distanceToObstacle is set to: %f", distanceToObstacle_);
        RCLCPP_INFO(this->get_logger(), "diagonalEdges is set to: %d", diagonalEdges_);
        RCLCPP_INFO(this->get_logger(), "minimumHeight is set to: %f", minimumHeight);
        RCLCPP_INFO(this->get_logger(), "maximumHeight is set to: %f", maximumHeight);
        RCLCPP_INFO(this->get_logger(), "visualOdometry set to: %s", visualOdometry ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "max_iterations is set to: %d", max_iterations_);
        RCLCPP_INFO(this->get_logger(), "goal_sample_rate is set to: %f", goal_sample_rate_);


        start_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initial_pose", 10, std::bind(&RRTNormal::startCallback, this, std::placeholders::_1));

        subscription_navigable_removed_vertices = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/obstacles_vertices", 10, std::bind(&RRTNormal::callback_removed_navigable_vertices, this, std::placeholders::_1));
            
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&RRTNormal::goalCallback, this, std::placeholders::_1));
            
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&RRTNormal::mapCallback, this, std::placeholders::_1));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/rrt_path", 10);
        tree_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/rrt_tree", 10);

        random_points_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/rrt_random_points", 10);

        decimals = countDecimals(distanceToObstacle_);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&RRTNormal::rrtPlanningCallback, this));

        RCLCPP_INFO(this->get_logger(), "RRT Normal inicializado");
    }

private:
    int max_iterations_;
    double goal_sample_rate_;
    
    size_t i_ = 0; 
    int diagonalEdges_;
    float pose_x_ = 0.0, pose_y_ = 0.0, pose_z_ = 0.0;
    float distanceToObstacle_, minimumHeight, maximumHeight;
    unsigned int width_;
    unsigned int height_;
    bool visualOdometry;

    int decimals = 0;

    geometry_msgs::msg::PoseWithCovarianceStamped start_;
    geometry_msgs::msg::PoseStamped goal_;
    nav_msgs::msg::OccupancyGrid map_;
    bool have_start_ = false;
    bool have_goal_ = false;
    bool have_map_ = false;
    std::vector<std::unique_ptr<RRTNode>> nodes_;
    std::vector<geometry_msgs::msg::Point> path_;
    std::unordered_set<std::tuple<float, float, float>> obstaclesVertices;
    std::unordered_set<std::tuple<float, float, float>> positions_prob_;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_navigable_removed_vertices;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tree_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr random_points_pub_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    std::random_device rd_;
    std::mt19937 gen_{rd_()};

    inline float roundToMultiple(float value, float multiple, int decimals) 
    {
        if (multiple == 0.0) return value; 
        
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
    
    std::vector<std::array<float, 3>> getOffsets(float distanceToObstacle) 
    {
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

    void startCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) 
    {
        start_ = *msg;
        have_start_ = true;
        RCLCPP_INFO(this->get_logger(), "Ponto de início definido: (%.2f, %.2f)", 
                   start_.pose.pose.position.x, start_.pose.pose.position.y);
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
    {
        goal_ = *msg;
        have_goal_ = true;
        RCLCPP_INFO(this->get_logger(), "Ponto de destino definido: (%.2f, %.2f)", 
                   goal_.pose.position.x, goal_.pose.position.y);
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) 
    {
        map_ = *msg;
        have_map_ = true;
        RCLCPP_INFO(this->get_logger(), "Mapa recebido: %d x %d com resolução %.2f", 
                   map_.info.width, map_.info.height, map_.info.resolution);
    }

    void rrtPlanningCallback() 
    {
        if (!have_start_ || !have_goal_ || !have_map_) {
            RCLCPP_INFO(this->get_logger(), "Aguardando todos os dados (start, goal, map)...");
            return;
        }
        auto start_time_ = std::chrono::high_resolution_clock::now();


        nodes_.clear();
        path_.clear();

        auto root = std::make_unique<RRTNode>(start_.pose.pose.position.x, start_.pose.pose.position.y);
        nodes_.push_back(std::move(root));

        visualization_msgs::msg::MarkerArray random_points_markers;
        int marker_id = 0;

        for (int i = 0; i < max_iterations_; i++) 
        {
            geometry_msgs::msg::Point rand_point = generateRandomPoint();

          
        
            
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = map_.header.frame_id;
            marker.header.stamp = this->now();
            marker.ns = "random_points";
            marker.id = marker_id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position = rand_point;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.color.a = 0.5;
            random_points_markers.markers.push_back(marker);

            if (i % 10 == 0) {
                random_points_pub_->publish(random_points_markers);
            }
            
            int nearest_index = findNearestNodeIndex(rand_point);
            const RRTNode& nearest_node = *nodes_[nearest_index];

            geometry_msgs::msg::Point new_point;
            if (!extendTowards(nearest_node, rand_point, new_point)) {
                continue;  
            }
            auto index1 = std::make_tuple(
                roundToMultiple(new_point.x, distanceToObstacle_, decimals),
                roundToMultiple(new_point.y, distanceToObstacle_, decimals),
                0.0
            );


            if(obstaclesVertices.find(index1) == obstaclesVertices.end())
            {
                auto new_node = std::make_unique<RRTNode>(
                    roundToMultiple(new_point.x, distanceToObstacle_, decimals),
                    roundToMultiple(new_point.y, distanceToObstacle_, decimals));
                
                new_node->parent = nodes_[nearest_index].get();
                nodes_.push_back(std::move(new_node));

                if (distance(nodes_.back()->x, nodes_.back()->y, 
                            goal_.pose.position.x, goal_.pose.position.y) < distanceToObstacle_) {
                    RCLCPP_INFO(this->get_logger(), "Caminho encontrado após %d iterações!", i);
                    generatePath();
                    break;
                }
            }
            
        }


        

        random_points_pub_->publish(random_points_markers);

        publishTree();

        if (!path_.empty()) 
        {
            publishPath();
        }
        else 
        {
            RCLCPP_WARN(this->get_logger(), "Caminho não encontrado dentro do limite de iterações.");
        }


        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> duration = end_time - start_time_; 

        RCLCPP_INFO(this->get_logger(), "RRT execution time: %.10f", duration.count());

    }

    geometry_msgs::msg::Point generateRandomPoint() 
    {
        geometry_msgs::msg::Point point;
        
        if (drand48() < goal_sample_rate_) 
        {
            point = goal_.pose.position;
        } 
        else 
        {
            double x_min = map_.info.origin.position.x;
            double y_min = map_.info.origin.position.y;
            double x_max = x_min + map_.info.width * map_.info.resolution;
            double y_max = y_min + map_.info.height * map_.info.resolution;
            
            std::uniform_real_distribution<> x_dist(x_min, x_max);
            std::uniform_real_distribution<> y_dist(y_min, y_max);
            
            float x = roundToMultiple(x_dist(gen_), distanceToObstacle_, decimals);
            float y = roundToMultiple(y_dist(gen_), distanceToObstacle_, decimals);
            point.x = x;
            point.y = y;
            point.z = 0.0;
        }
        
        return point;
    }

    int findNearestNodeIndex(const geometry_msgs::msg::Point& point) 
    {
        int nearest_index = 0;
        double min_dist = std::numeric_limits<double>::max();
        
        for (size_t i = 0; i < nodes_.size(); i++) 
        {
            double dist = distance(nodes_[i]->x, nodes_[i]->y, point.x, point.y);

            if (dist < min_dist) 
            {
                min_dist = dist;
                nearest_index = i;
            }
        }
        
        return nearest_index;
    }

    bool extendTowards(const RRTNode& from_node, const geometry_msgs::msg::Point& to_point, geometry_msgs::msg::Point& new_point) 
    {
        double dx = to_point.x - from_node.x;
        double dy = to_point.y - from_node.y;
        double dist = std::sqrt(dx*dx + dy*dy);
        
        if (dist < 1e-6) 
        {
            return false; 
        }
        
        new_point.x = from_node.x + (dx / dist) * distanceToObstacle_;
        new_point.y = from_node.y + (dy / dist) * distanceToObstacle_;
        new_point.z = 0.0;
        
        
        
        return true;
    }

  

    double distance(double x1, double y1, double x2, double y2) 
    {
        return std::sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
    }

    void generatePath() 
    {
        const RRTNode* node = nodes_.back().get();
        
        while (node != nullptr) 
        {
            geometry_msgs::msg::Point point;
            point.x = node->x;
            point.y = node->y;
            point.z = 0.0;
            path_.push_back(point);
            node = node->parent;
        }
        
        std::reverse(path_.begin(), path_.end());
    }

    void publishTree()
    {
        visualization_msgs::msg::MarkerArray tree_markers;
        
        visualization_msgs::msg::Marker nodes_marker;
        nodes_marker.header.frame_id = map_.header.frame_id;
        nodes_marker.header.stamp = this->now();
        nodes_marker.ns = "rrt_nodes";
        nodes_marker.id = 0;
        nodes_marker.type = visualization_msgs::msg::Marker::POINTS;
        nodes_marker.action = visualization_msgs::msg::Marker::ADD;
        nodes_marker.pose.orientation.w = 1.0;
        nodes_marker.scale.x = 0.1;
        nodes_marker.scale.y = 0.1;
        nodes_marker.color.r = 0.0;
        nodes_marker.color.g = 1.0;
        nodes_marker.color.b = 0.0;
        nodes_marker.color.a = 1.0;

        visualization_msgs::msg::Marker edges_marker;
        edges_marker.header.frame_id = map_.header.frame_id;
        edges_marker.header.stamp = this->now();
        edges_marker.ns = "rrt_edges";
        edges_marker.id = 1;
        edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        edges_marker.action = visualization_msgs::msg::Marker::ADD;
        edges_marker.pose.orientation.w = 1.0;
        edges_marker.scale.x = 0.03;
        edges_marker.color.r = 0.0;
        edges_marker.color.g = 0.8;
        edges_marker.color.b = 0.2;
        edges_marker.color.a = 0.8;
        
        for (const auto& node : nodes_) 
        {
            geometry_msgs::msg::Point p;
            p.x = roundToMultiple(node->x, distanceToObstacle_, decimals);
            p.y = roundToMultiple(node->y, distanceToObstacle_, decimals);
            p.z = 0.0;
            nodes_marker.points.push_back(p);
            
            if (node->parent != nullptr) 
            {
                geometry_msgs::msg::Point parent_p;
                parent_p.x = node->parent->x;
                parent_p.y = node->parent->y;
                parent_p.z = 0.0;
                
                edges_marker.points.push_back(parent_p);
                edges_marker.points.push_back(p);
            }
        }
        
        tree_markers.markers.push_back(nodes_marker);
        tree_markers.markers.push_back(edges_marker);
        
        tree_pub_->publish(tree_markers);
    }

    void publishPath() 
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = map_.header.frame_id;
        path_msg.header.stamp = this->now();

        
        auto start_time_ = std::chrono::high_resolution_clock::now();
        int k = 0;

        while (k < static_cast<int>(path_.size()) - 1) 
        {
            bool shortcutFound = false;
            for (int i = static_cast<int>(path_.size()) - 1; i > k; i--) 
            {
                std::tuple<float, float, float> A {
                    path_[k].x,
                    path_[k].y,
                    0.0
                };
                std::tuple<float, float, float> B {
                    path_[i].x,
                    path_[i].y,
                    0.0
                };
        
                float ax = std::get<0>(A), ay = std::get<1>(A), az = std::get<2>(A);
                float bx = std::get<0>(B), by = std::get<1>(B), bz = std::get<2>(B);
        
                float dx = bx - ax, dy = by - ay, dz = bz - az;
                float distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        
                if (distance == 0) 
                {
                    continue;
                }
        
                float ux = dx / distance;
                float uy = dy / distance;
                float uz = dz / distance;
        
                float step = distanceToObstacle_;
                float t = 0.0f;
                bool obstacleFound = false;
                auto offsets1 = getOffsets(distanceToObstacle_);
        
                while (t < distance && obstacleFound == false) 
                {
                    std::tuple<float, float, float> point;
                    std::get<0>(point) = ax + t * ux;
                    std::get<1>(point) = ay + t * uy;
                    std::get<2>(point) = az + t * uz;
        
                    double new_x = roundToMultiple(std::get<0>(point), distanceToObstacle_, decimals);
                    double new_y = roundToMultiple(std::get<1>(point), distanceToObstacle_, decimals);
                    double new_z = roundToMultiple(std::get<2>(point), distanceToObstacle_, decimals);
        
                    auto neighbor_tuple = std::make_tuple(
                        static_cast<float>(new_x), 
                        static_cast<float>(new_y), 
                        static_cast<float>(new_z)
                    );
                    
                    if (obstaclesVertices.find(neighbor_tuple) != obstaclesVertices.end()) 
                    {
                        obstacleFound = true;
                        break;
                    }

                    t += step;
                }
        
                if (obstacleFound == false) 
                {
                    path_.erase(path_.begin() + k + 1, path_.begin() + i);

                    shortcutFound = true;

                    break;  
                }
            }
        
            if (shortcutFound == true)
            {
                k++;
            } 
            else if(shortcutFound == false)
            {
                break;
            }

        }

        
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> duration = end_time - start_time_; 

        RCLCPP_INFO(this->get_logger(), "RRT filter execution time: %.10f", duration.count());

        for (const auto& point : path_) 
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position = point;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }
        
        path_pub_->publish(path_msg);
        RCLCPP_INFO(this->get_logger(), "Caminho publicado com %zu pontos", path_msg.poses.size());
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

            if(visualOdometry == true)
            {
                if(z > minimumHeight && z <= maximumHeight)
                {
                    z = 0;

                    auto index = std::make_tuple(
                        roundToMultiple(x, distanceToObstacle_, decimals),
                        roundToMultiple(y, distanceToObstacle_, decimals),
                        roundToMultiple(z, distanceToObstacle_, decimals)
                    );

                    auto index2 = std::make_tuple(
                        roundToMultiple(x, distanceToObstacle_, decimals),
                        roundToMultiple(y, distanceToObstacle_, decimals),
                        1.0f
                    );

                    positions_prob_.insert(index2);
                    obstaclesVertices.insert(index);
                }
            }
            else
            {
                z = 0;

                auto index = std::make_tuple(
                    roundToMultiple(x, distanceToObstacle_, decimals),
                    roundToMultiple(y, distanceToObstacle_, decimals),
                    0.0
                );
    
                auto index2 = std::make_tuple(
                    roundToMultiple(x, distanceToObstacle_, decimals),
                    roundToMultiple(y, distanceToObstacle_, decimals),
                    1.0f
                );
    
                positions_prob_.insert(index2);
                obstaclesVertices.insert(index);
            }
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RRTNormal>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}