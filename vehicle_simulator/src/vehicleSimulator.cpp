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
    const std::string target_frame = "map";
    const std::string source_frame = "odom";  // 原始 Odometry 所在坐标系

    try {
        // 1️⃣ 查找 TF (map ← odom)
        geometry_msgs::msg::TransformStamped tfStamped =
            tfBuffer->lookupTransform(target_frame, source_frame, 
                                      odom->header.stamp, rclcpp::Duration::from_seconds(0.1));

        // 2️⃣ 将 odom.pose.pose 转换为 PoseStamped
        geometry_msgs::msg::PoseStamped odom_pose;
        odom_pose.header = odom->header;
        odom_pose.header.frame_id = source_frame;
        odom_pose.pose = odom->pose.pose;

        geometry_msgs::msg::PoseStamped map_pose;
        tf2::doTransform(odom_pose, map_pose, tfStamped);

        // 3️⃣ 构造新的 Odometry 消息（frame_id 改为 map）
        auto odom_in_map = std::make_shared<nav_msgs::msg::Odometry>();
        odom_in_map->header = odom->header;
        odom_in_map->header.frame_id = target_frame;
        odom_in_map->child_frame_id = odom->child_frame_id;  // 一般是 base_link
        odom_in_map->pose.pose = map_pose.pose;
        odom_in_map->twist = odom->twist;

        // 4️⃣ 发布（覆盖 /odom 或新话题）
        static auto pubOdomInMap = nh->create_publisher<nav_msgs::msg::Odometry>("/odom", 5);
        pubOdomInMap->publish(*odom_in_map);

        RCLCPP_INFO_ONCE(nh->get_logger(), "✅ Publishing transformed odometry in map frame.");

    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(nh->get_logger(), *nh->get_clock(), 2000, 
            "TF transform from '%s' to '%s' failed: %s",
            source_frame.c_str(), target_frame.c_str(), ex.what());
    }
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


    bool use_sim_time = nh->get_parameter("use_sim_time").as_bool();
    if (use_sim_time)
    {
        RCLCPP_WARN(nh->get_logger(), "⚠ use_sim_time = true, switching to system time!");
        nh->set_parameter(rclcpp::Parameter("use_sim_time", false));
    }


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
        // rclcpp::Time now = nh->get_clock()->now();
        // auto t = rclcpp::Clock().now();   
        // RCLCPP_INFO(nh->get_logger(), "[rclcpp::Clock().now()] sec:%lf nano:%ld", t.seconds(), t.nanoseconds());

        // std::chrono::steady_clock::time_point td = std::chrono::steady_clock::now(); 
        // std::chrono::steady_clock::duration dtn = td.time_since_epoch();
        // double secs = dtn.count() * std::chrono::steady_clock::period::num / std::chrono::steady_clock::period::den;
        // RCLCPP_INFO(nh->get_logger(), "[std::chrono::steady_clock::now()] sec:%lf", secs);
        
        // auto t2 = nh->get_clock()->now();
        // RCLCPP_INFO(nh->get_logger(), "[get_clock()->now()] sec:%lf nano:%ld", t2.seconds(), t2.nanoseconds());

        // auto t3 = nh->now();
        // RCLCPP_INFO(nh->get_logger(), "[this->now()] sec:%lf nano:%ld", t3.seconds(), t3.nanoseconds());


        geometry_msgs::msg::TransformStamped tfStamped =
            tfBuffer->lookupTransform(target_frame, source_frame, nh->get_clock()->now(), rclcpp::Duration::from_seconds(0.1));



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

    nh->set_parameter(rclcpp::Parameter("use_sim_time", false));

    // 初始化 TF Buffer 和 Listener
    tfBuffer = std::make_shared<tf2_ros::Buffer>(nh->get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

    auto subScan = nh->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar/pointcloud",2,scanHandler);

    pubScanPointer = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/registered_scan",2);

    auto subOdometry = nh->create_subscription<nav_msgs::msg::Odometry>("/Odometry",5,odometryHandle);



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

