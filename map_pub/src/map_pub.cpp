#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <memory>
#include <string>

class PCDMapPublisher : public rclcpp::Node
{
public:
    PCDMapPublisher() : Node("pcd_map_publisher")
    {
        // 声明参数
        this->declare_parameter<std::string>("pcd_file_path", "/home/z/rm_sim/src/rm_simulation/FAST_LIO/PCD/test.pcd");
        this->declare_parameter<std::string>("frame_id", "map");
        this->declare_parameter<double>("publish_rate", 1.0);
        
        // 获取参数
        std::string pcd_file_path = this->get_parameter("pcd_file_path").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        double publish_rate = this->get_parameter("publish_rate").as_double();
        
        // 创建发布者
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/localizer/map_cloud", 10);
        
        // 加载PCD文件
        if (!loadPCDFile(pcd_file_path)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file: %s", pcd_file_path.c_str());
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Successfully loaded PCD file: %s", pcd_file_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Point cloud size: %ld points", cloud_->size());
        
        // 创建定时器发布点云
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate),
            std::bind(&PCDMapPublisher::publishPointCloud, this));
        
        RCLCPP_INFO(this->get_logger(), "PCD map publisher started, publishing on topic: /localizer/map_cloud");
    }

private:
    bool loadPCDFile(const std::string& file_path)
    {
        cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud_) == -1) {
            return false;
        }
        
        return true;
    }
    
    void publishPointCloud()
    {
        if (cloud_->empty()) {
            RCLCPP_WARN(this->get_logger(), "Point cloud is empty, skipping publish");
            return;
        }
        
        // 转换为ROS2消息
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud_, cloud_msg);
        cloud_msg.header.stamp = this->now();
        cloud_msg.header.frame_id = frame_id_;
        
        // 发布消息
        publisher_->publish(cloud_msg);
        
        RCLCPP_DEBUG(this->get_logger(), "Published point cloud with %ld points", cloud_->size());
    }
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    std::string frame_id_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCDMapPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}