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
#include <pcl_ros/transforms.hpp>

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "rmw/types.h"
#include "rmw/qos_profiles.h"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

rclcpp::Time odomTime;

double sensorOffsetX = 0;
double sensorOffsetY = 0;

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

// TODO:仿真里面适配没有点云强度
pcl::PointCloud<pcl::PointXYZ>::Ptr scanRawData(new pcl::PointCloud<pcl::PointXYZ>());

pcl::PointCloud<pcl::PointXYZI>::Ptr scanData(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudIncl(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubScanPointer;

// 声明全局共享指针
std::shared_ptr<tf2_ros::Buffer> tfBuffer;
std::shared_ptr<tf2_ros::TransformListener> tfListener;

std::vector<int> scanInd;

rclcpp::Node::SharedPtr nh;

void odometryHandle(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
    // odomTime = rclcpp::Time(odom->header.stamp).seconds();
    // double roll,pitch,yaw;
    // geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
    // tf2::Matrix3x3(tf2::Quaternion(geoQuat.x,geoQuat.y,geoQuat.z,geoQuat.w)).getRPY(roll,pitch,yaw);

    // vehicleRoll = roll;
    // vehiclePitch = pitch;
    // vehicleYaw = yaw;

    // // 都是根据odom的值来更新vehicleX的位置??
    // vehicleX = odom->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
    // vehicleY = odom->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
    // // 打印 vehicleX 和 vehicleY的值
    // RCLCPP_INFO(nh->get_logger(),"vehicleX :%f , vehicleY :%f",vehicleX,vehicleY);
    // vehicleZ = odom->pose.pose.position.z;
    double vehicleX = odom->pose.pose.position.x;
    double vehicleY = odom->pose.pose.position.y;
    // double vehicleZ = odom->pose.pose.position.z;
    // RCLCPP_INFO(nh->get_logger(),"vehicleX:%f , vehicleY :%f",vehicleX,vehicleY);
}

void scanHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanIn)
{
    // 确认输入坐标系
    std::string source_frame = scanIn->header.frame_id;
    const std::string target_frame = "map";

    // 打印一次 frame id
    static bool printed = false;
    if (!printed)
    {
        RCLCPP_INFO(nh->get_logger(), "Incoming scan frame_id: %s", source_frame.c_str());
        printed = true;
    }

    // 转为 PCL 并修复强度
    pcl::fromROSMsg(*scanIn, *scanRawData);
    pcl::removeNaNFromPointCloud(*scanRawData, *scanRawData, scanInd);

    scanData->clear();
    scanData->reserve(scanRawData->size());
    for (const auto &pt : *scanRawData)
    {
        pcl::PointXYZI point;
        point.x = pt.x;
        point.y = pt.y;
        point.z = pt.z;
        point.intensity = 100.0f;
        scanData->points.push_back(point);
    }

    sensor_msgs::msg::PointCloud2 scanMsg;
    pcl::toROSMsg(*scanData, scanMsg);
    scanMsg.header = scanIn->header;

    // 取点云时间戳
    rclcpp::Time timestamp = scanIn->header.stamp;

    // ✅ 尝试等待 TF（最多 0.5 秒）
    if (!tfBuffer->canTransform(target_frame, source_frame, rclcpp::Time(0)))
    {
        RCLCPP_WARN(nh->get_logger(),
                    "❌ Unable to transform from '%s' to '%s' at time %.3f (no TF available)",
                    source_frame.c_str(), target_frame.c_str(),
                    timestamp.seconds());
        return;
    }

    try
    {
        sensor_msgs::msg::PointCloud2 scanOut;

        // ✅ 建议使用 lookupTransform + tf2::doTransform，而不是 pcl_ros::transformPointCloud
        rclcpp::Time now = nh->get_clock()->now();
        geometry_msgs::msg::TransformStamped tfStamped =
            tfBuffer->lookupTransform(target_frame, source_frame, rclcpp::Time(0));


        tf2::doTransform(scanMsg, scanOut, tfStamped);

        scanOut.header.stamp = timestamp;
        scanOut.header.frame_id = target_frame;
        pubScanPointer->publish(scanOut);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR(nh->get_logger(), "Transform failed: %s", ex.what());
    }
}



int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    nh = rclcpp::Node::make_shared("vehicleSimulator");

    // 初始化 TF Buffer 和 Listener
    tfBuffer = std::make_shared<tf2_ros::Buffer>(nh->get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

    auto subScan = nh->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar/pointcloud",2,scanHandler);

    pubScanPointer = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/registered_scan",2);

    auto subOdometry = nh->create_subscription<nav_msgs::msg::Odometry>("/Odometry",5,odometryHandle);

    // nh->declare_parameter("use_sim_time", true);
    nh->set_parameter(rclcpp::Parameter("use_sim_time", true));


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

