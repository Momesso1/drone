#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

class SavePointCloudNode : public rclcpp::Node
{
public:
    SavePointCloudNode() : Node("save_pointcloud_node")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_map", 10, 
            std::bind(&SavePointCloudNode::pointcloud_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "SavePointCloudNode is running...");
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        std::string filename = "point_cloud.pcd";
        if (pcl::io::savePCDFileASCII(filename, cloud) == 0) {
            RCLCPP_INFO(this->get_logger(), "Saved point cloud to %s", filename.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to save point cloud.");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SavePointCloudNode>());
    rclcpp::shutdown();
    return 0;
}
