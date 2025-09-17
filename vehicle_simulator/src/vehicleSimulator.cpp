#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.h>
#include <geometry_msgs/msg/polygon_stamped.h>
#include <sensor_msgs/msg/imu.h>

#include <gazebo_msgs/msg/model_state.hpp>
#include <gazebo_msgs/msg/entity_state.hpp>
#include <gazebo_msgs/srv/set_entity_state.hpp>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "rmw/types.h"
#include "rmw/qos_profiles.h"

rclcpp::Time odomTime;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float vehicleYawRate = 0;
float vehicleSpeed = 0;

float terrainZ = 0;
float terrainRoll = 0;
float terrainPitch = 0;

pcl::PointCloud<pcl::PointXYZI>::Ptr scanData(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZ>::Ptr scanRawData(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudIncl(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubScanPointer;

std::vector<int> scanInd;


void scanHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanIn)
{
    // NOTE这里雷达的仿真没有点云的强度

    // TODO 记得补充systemInited
    
    double scanTime = rclcpp::Time(scanIn->header.stamp).seconds();

    double odomRecTime = odomTime.seconds();
    float vehicleRecX = vehicleX;
    float vehicleRecY = vehicleY;
    float vehicleRecZ = vehicleZ;
    float terrainRecRoll = terrainRoll;
    float terrainRecPitch = terrainPitch;

    float sinTerrainRecRoll = sin(terrainRecRoll);
    float cosTerrainRecRoll = cos(terrainRecRoll);
    float sinTerrainRecPitch = sin(terrainRecPitch);
    float cosTerrainRecPitch = cos(terrainRecPitch);

    scanData->clear();    
    scanRawData->clear();
    // 先转成PointXYZ先不依赖强度信息
    pcl::fromROSMsg(*scanIn,*scanRawData);
    // 去除无效点云 返回有效点的索引
    // 输入点云 输出点云 std::vector<int>有效点在原始点云中的索引
    pcl::removeNaNFromPointCloud(*scanRawData,*scanRawData,scanInd);

    int scanDataSize = scanRawData->points.size();
    for (int i = 0; i < scanDataSize; i++)
    {
        // 保存强度信息
        pcl::PointXYZI point;


        // HACK 这里没有很理解
        float pointX1 = scanRawData->points[i].x;
        float pointY1 = scanRawData->points[i].y * cosTerrainRecRoll - scanRawData->points[i].z * sinTerrainRecRoll;
        float pointZ1 = scanRawData->points[i].y * sinTerrainRecRoll + scanRawData->points[i].z * cosTerrainRecRoll;

        float pointX2 = pointX1 * cosTerrainRecPitch + pointZ1 * sinTerrainRecPitch;
        float pointY2 = pointY1;
        float pointZ2 = -pointX1 * sinTerrainRecPitch + pointZ1 * cosTerrainRecPitch;

        // 
        // float pointX3 = pointX2 + vehicleRecX;
        // float pointY3 = pointY2 + vehicleRecY;
        // float pointZ3 = pointZ2 + vehicleRecZ;
        point.x = pointX2 + vehicleRecX;
        point.y = pointY2 + vehicleRecY;
        point.z = pointZ2 + vehicleRecZ;

        // NOTE设置默认强度值
        point.intensity = 100.0f;
        scanData->points.push_back(point);
        
    }
    
    // publish
    sensor_msgs::msg::PointCloud2 scanData2;
    pcl::toROSMsg(*scanData, scanData2);
    scanData2.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomRecTime * 1e9));
    // scanData2.header.frame_id = "map";
    // 转到map坐标系下面
    scanData2.header.frame_id = "camera_init";
    pubScanPointer->publish(scanData2);
    
}


int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    auto nh = rclcpp::Node::make_shared("vehicleSimulator");

    auto subScan = nh->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar/pointcloud",2,scanHandler);

    pubScanPointer = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/registered_scan",2);

    rclcpp::Rate rate(200);
    bool status = rclcpp::ok();
    while (status)
    {
        rclcpp::spin_some(nh);



        status = rclcpp::ok();
        rate.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}

