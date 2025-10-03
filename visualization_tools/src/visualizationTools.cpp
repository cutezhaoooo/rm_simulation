#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;

double overallMapVoxelSize = 0.5;
string mapFile;

pcl::PointCloud<pcl::PointXYZ>::Ptr overallMapCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr overallMapCloudDwz(new pcl::PointCloud<pcl::PointXYZ>());
pcl::VoxelGrid<pcl::PointXYZ> overallMapDwzFilter;

sensor_msgs::msg::PointCloud2 overallMap2;

// TODO目前只保留了发布overall_map的内容后续再做补充
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("overall_map_publisher");

  // 声明参数
  nh->declare_parameter<string>("mapFile", mapFile);
  nh->declare_parameter<double>("overallMapVoxelSize", overallMapVoxelSize);

  // 获取参数
  nh->get_parameter("mapFile", mapFile);
  nh->get_parameter("overallMapVoxelSize", overallMapVoxelSize);

  // 配置滤波器
  overallMapDwzFilter.setLeafSize(overallMapVoxelSize, overallMapVoxelSize, overallMapVoxelSize);

  // 读取 PLY 地图
  pcl::PLYReader ply_reader;
  if (ply_reader.read(mapFile, *overallMapCloud) == -1) {
    RCLCPP_ERROR(nh->get_logger(), "Couldn't read pointcloud.ply file: %s", mapFile.c_str());
    return -1;
  }
  RCLCPP_INFO(nh->get_logger(), "Loaded map file: %s, points: %zu", mapFile.c_str(), overallMapCloud->size());

  // 下采样
  overallMapCloudDwz->clear();
  overallMapDwzFilter.setInputCloud(overallMapCloud);
  overallMapDwzFilter.filter(*overallMapCloudDwz);

  // 转换成 ROS2 点云消息
  pcl::toROSMsg(*overallMapCloudDwz, overallMap2);
  overallMap2.header.frame_id = "camera_init";

  // 发布器
  auto pubOverallMap = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/overall_map", 1);

  rclcpp::Rate rate(1.0);  // 每秒发布一次
  while (rclcpp::ok()) {
    overallMap2.header.stamp = nh->now();
    pubOverallMap->publish(overallMap2);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
