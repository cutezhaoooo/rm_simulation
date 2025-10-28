#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class MapPublisherNode : public rclcpp::Node
{
public:
    MapPublisherNode() : Node("map_publisher")
    {
        // 参数：PCD文件路径
        this->declare_parameter<std::string>("pcd_path", "/home/z/rm_simulation/src/rm_simulation/FAST_LIO/PCD/test.pcd");
        this->get_parameter("pcd_path", pcd_path_);

        if (pcd_path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "请通过参数 'pcd_path' 指定 .pcd 文件路径");
            rclcpp::shutdown();
            return;
        }

        // 尝试加载点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path_, *cloud) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "无法加载 PCD 文件: %s", pcd_path_.c_str());
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "成功加载点云: %s, 点数: %zu", pcd_path_.c_str(), cloud->size());

        // 转换为ROS消息
        pcl::toROSMsg(*cloud, map_msg_);
        map_msg_.header.frame_id = "map";

        // 创建publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/localizer/map_cloud", 1);

        // 定时发布（例如1Hz）
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MapPublisherNode::publishMap, this)
        );
    }

private:
    void publishMap()
    {
        map_msg_.header.stamp = this->get_clock()->now();
        publisher_->publish(map_msg_);
        RCLCPP_INFO_ONCE(this->get_logger(), "开始发布地图点云...");
    }

    std::string pcd_path_;
    sensor_msgs::msg::PointCloud2 map_msg_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
