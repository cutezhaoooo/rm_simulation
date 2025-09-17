#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class PointCloudFieldChecker : public rclcpp::Node
{
public:
  PointCloudFieldChecker()
  : Node("pointcloud_field_checker")
  {
    // 创建订阅者
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar/pointcloud",  // 修改为你实际的 topic 名称
      rclcpp::QoS(1).best_effort(),  // 可根据需要改为 reliable()
      std::bind(&PointCloudFieldChecker::pointcloudCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Waiting for point cloud message on /livox/lidar/pointcloud...");
  }

private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Point Cloud Fields:");
    for (const auto& field : msg->fields) {
      RCLCPP_INFO_STREAM(this->get_logger(), 
        "  - " << field.name 
               << " (type: " << static_cast<int>(field.datatype)
               << ", offset: " << field.offset 
               << ", count: " << field.count << ")");
    }

    RCLCPP_INFO(this->get_logger(), "Height: %d", msg->height);
    RCLCPP_INFO(this->get_logger(), "Width: %d", msg->width);
    RCLCPP_INFO(this->get_logger(), "Point step: %u", msg->point_step);
    RCLCPP_INFO(this->get_logger(), "Row step: %u", msg->row_step);
    RCLCPP_INFO(this->get_logger(), "Frame ID: %s", msg->header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "Is bigendian: %s", msg->is_bigendian ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Is dense: %s", msg->is_dense ? "true" : "false");

    // 只处理一次，然后销毁订阅者（或直接关闭节点）
    this->subscription_.reset();

    // 可选：打印完后退出程序
    RCLCPP_INFO(this->get_logger(), "Shutting down after printing point cloud info.");
    // rclcpp::shutdown();
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudFieldChecker>());
  rclcpp::shutdown();
  return 0;
}