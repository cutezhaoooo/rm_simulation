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
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <sensor_msgs/msg/imu.h>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

double vehicleLength = 0.6;
double vehicleWidth = 0.6;
double sensorOffsetX = 0;
double sensorOffsetY = 0;
bool twoWayDrive = true;
double laserVoxelSize = 0.05;
double terrainVoxelSize = 0.2;
bool useTerrainAnalysis = false;
bool checkObstacle = true;
bool checkRotObstacle = false;
double adjacentRange = 3.5;
double obstacleHeightThre = 0.2;
double groundHeightThre = 0.1;
double costHeightThre = 0.1;
double costScore = 0.02;
bool useCost = false;
const int laserCloudStackNum = 1;
int laserCloudCount = 0;
int pointPerPathThre = 2;
double minRelZ = -0.5;
double maxRelZ = 0.25;
double maxSpeed = 1.0;
double dirWeight = 0.02;
double dirThre = 90.0;
bool dirToVehicle = false;
double pathScale = 1.0;
double minPathScale = 0.75;
double pathScaleStep = 0.25;
bool pathScaleBySpeed = true;
double minPathRange = 1.0;
double pathRangeStep = 0.5;
bool pathRangeBySpeed = true;
bool pathCropByGoal = true;
bool autonomyMode = false;
double autonomySpeed = 1.0;
double joyToSpeedDelay = 2.0;
double joyToCheckObstacleDelay = 5.0;
double goalClearRange = 0.5;
double goalX = 0;
double goalY = 0;


float joySpeed = 0;
float joySpeedRaw = 0;
float joyDir = 0;

const int pathNum = 343;
const int groupNum = 7;
float gridVoxelSize = 0.02;
float searchRadius = 0.45;
float gridVoxelOffsetX = 3.2;
float gridVoxelOffsetY = 4.5;
const int gridVoxelNumX = 161;
const int gridVoxelNumY = 451;
const int gridVoxelNum = gridVoxelNumX * gridVoxelNumY;

int pathList[pathNum] = {0};
float endDirPathList[pathNum] = {0};
int clearPathList[36 * pathNum] = {0};
float pathPenaltyList[36 * pathNum] = {0};
float clearPathPerGroupScore[36 * groupNum] = {0};
std::vector<int> correspondences[gridVoxelNum];

bool newLaserCloud = false;
bool newTerrainCloud = false;

double odomTime = 0;
double joyTime = 0;

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;

pcl::VoxelGrid<pcl::PointXYZI> laserDwzFilter,terrainDwzFilter;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
// laserCloudCrop 是阈值adjacentRange内的点云
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
// laserCloudCrop保存的是点云降采样后的结果
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDWZ(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudStack[laserCloudStackNum];
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());

pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr boundaryCloud(new pcl::PointCloud<pcl::PointXYZI>());



rclcpp::Node::SharedPtr nh;

void odometryHandle(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
    // NOTE和直接读取odom的值一样的
    odomTime = rclcpp::Time(odom->header.stamp).seconds();
    double roll,pitch,yaw;
    geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
    tf2::Matrix3x3(tf2::Quaternion(geoQuat.x,geoQuat.y,geoQuat.z,geoQuat.w)).getRPY(roll,pitch,yaw);

    vehicleRoll = roll;
    vehiclePitch = pitch;
    vehicleYaw = yaw;

    // 都是根据odom的值来更新vehicleX的位置??
    vehicleX = odom->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
    vehicleY = odom->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
    // 打印 vehicleX 和 vehicleY的值
    // RCLCPP_INFO(nh->get_logger(),"vehicleX :%f , vehicleY :%f",vehicleX,vehicleY);
    vehicleZ = odom->pose.pose.position.z;

}

void laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud2)
{
    if (!useTerrainAnalysis)
    {
        // 不走地形分析
        laserCloud->clear();
        // 转成pcl格式
        pcl::fromROSMsg(*laserCloud2,*laserCloud);

        // 裁剪近距离区域
        pcl::PointXYZI point;
        laserCloudCrop->clear();
        int laserCloudSize = laserCloud->points.size();
        for (int i = 0; i < laserCloudSize; i++)
        {
            point = laserCloud->points[i];

            float pointX = point.x;
            float pointY = point.y;
            float pointZ = point.z;
            // 点云距离车辆的距离
            float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));
            // 如果距离小于
            if (dis<adjacentRange)
            {
                point.x = pointX;
                point.y = pointY;
                point.z = pointZ;
                laserCloudCrop->push_back(point);
            }
        }
        
        laserCloudDWZ->clear();
        // 降采样
        laserDwzFilter.setInputCloud(laserCloudCrop);
        // 降采样结果保存在laserCloudDwz中
        laserDwzFilter.filter(*laserCloudDWZ);

        newLaserCloud = true;
    }
}

void terrainCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrainCloud2)
{
    if (useTerrainAnalysis)
    {
        terrainCloud->clear();
        pcl::fromROSMsg(*terrainCloud2,*terrainCloud);
        
        pcl::PointXYZI point;
        // 这里要提前赋值否则会报错
        terrainCloudCrop->clear();
        int terrainCloudSize = terrainCloud->points.size();
        for (int i = 0; i < terrainCloudSize; i++)
        {
            point = terrainCloud->points[i];
            float pointX = point.x;
            float pointY = point.y;
            float pointZ = point.z;
            
            float dis = std::sqrt((pointX - vehicleX)*(pointX - vehicleX) +
                                    (pointY - vehicleY)*(pointY - vehicleY));
            if (dis < adjacentRange && (point.intensity > obstacleHeightThre || useCost))
            {
                point.x = pointX;
                point.y = pointY;
                point.z = pointZ;
                terrainCloudCrop->push_back(point);
            }
        }
        
        terrainCloudDwz->clear();
        terrainDwzFilter.setInputCloud(terrainCloudCrop);
        terrainDwzFilter.filter(*terrainCloudDwz);

        newTerrainCloud = true;
    }
}

void boundaryHandle(const geometry_msgs::msg::PolygonStamped::ConstSharedPtr boundary)
{

}

void goalHandler(const geometry_msgs::msg::PointStamped::ConstSharedPtr goal)
{
    std::string frame_id = goal->header.frame_id;
    RCLCPP_INFO(nh->get_logger(),"目标点的frame id是 %s",frame_id.c_str());
}

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);

    // 根据odom来更新vehicle的值
    nh = rclcpp::Node::make_shared("localPlanner");

    // 这里要适配fast lio2 将/odom改为/Odometry
    auto subOdometry = nh->create_subscription<nav_msgs::msg::Odometry>("/Odometry",5,odometryHandle);

    // 这里雷达的坐标系也需要转换一下
    auto subLaserCloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>("/registered_scan", 5, laserCloudHandler);

    auto pubLaserCloud = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/plannerCloud",5);
    auto pubLaserCloud2 = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/plannerCloud2",5);

    auto subTerrainCloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>("/terrain_map",5,terrainCloudHandler);

    auto subBoundary = nh->create_subscription<geometry_msgs::msg::PolygonStamped>("/navigation_boundary",5,boundaryHandle);

    // NOTE:local planner中订阅的话题是way_point
    auto subGoal = nh->create_subscription<geometry_msgs::msg::PointStamped>("/goal_pose",5,goalHandler);

    laserDwzFilter.setLeafSize(laserVoxelSize, laserVoxelSize, laserVoxelSize);
    terrainDwzFilter.setLeafSize(terrainVoxelSize, terrainVoxelSize, terrainVoxelSize);

    RCLCPP_INFO(nh->get_logger(),"Initialization complete.");

    rclcpp::Rate rate(100);
    bool status = rclcpp::ok();

    // 重置laserCloudStack
    for (int i = 0; i < laserCloudStackNum; i++)
    {
        laserCloudStack[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    }
    
    // TAG设置点云发布的频率
    // rclcpp::Rate rate(50);
    while (status)
    {
        rclcpp::spin_some(nh);
        
        // 先看看有新点云的情况 有新的newLaserCloud 或者 newTerrainCloud都会进入然后分别处理
        if (newLaserCloud || newTerrainCloud)
        {
            if (newLaserCloud )
            {
                newLaserCloud = false;
    
                // laserCloudStack是个长度为1的点云数组
                laserCloudStack[laserCloudCount]->clear();
                *laserCloudStack[laserCloudCount] = *laserCloudDWZ;
                laserCloudCount = (laserCloudCount + 1) % laserCloudStackNum;
                plannerCloud->clear();
                for (int i = 0; i < laserCloudStackNum; i++) {
                    // HACK 将点云拼接起来???
                    // 应该是要将地图逐渐拼接起来
                    *plannerCloud += *laserCloudStack[i];
                    
                }
                // 这里将点云pub出来看看
                // TAG设置header 并且需要转成ros msg
                // 其实就是降采样后的点云
                sensor_msgs::msg::PointCloud2 ros_pc2;
                pcl::toROSMsg(*plannerCloud,ros_pc2);
                ros_pc2.header.frame_id = "camera_init";
                ros_pc2.header.stamp = nh->now();
                pubLaserCloud->publish(ros_pc2);

            }

            if (newTerrainCloud)
            {
                newTerrainCloud = false;

                plannerCloud->clear();
                *plannerCloud = *terrainCloudDwz;
            }
            
            float sinVehicleYaw = sin(vehicleYaw);
            float cosVehicleYaw = cos(vehicleYaw);

            pcl::PointXYZI point;
            plannerCloudCrop->clear();
            int plannerCloudSize = plannerCloud->points.size();
            for (int i = 0; i < plannerCloudSize; i++)
            {
                point.x = plannerCloud->points[i].x;
                point.y = plannerCloud->points[i].y;
                point.z = plannerCloud->points[i].z;
                point.intensity = plannerCloud->points[i].intensity;

                float dis = sqrt(point.x * point.x + point.y * point.y);
                if (dis < adjacentRange && ((point.z > minRelZ && point.z < maxRelZ) || useTerrainAnalysis)) {
                    plannerCloudCrop->push_back(point);
                }

            }

            // 将plannerCloud显示出来看一看
            // plannerCloud
            // sensor_msgs::msg::PointCloud2 plannerCloudMsg;
            // pcl::toROSMsg(*plannerCloud,plannerCloudMsg);
            // plannerCloudMsg.header.stamp = nh->get_clock()->now();
            // plannerCloudMsg.header.frame_id = "camera_init";
            // pubLaserCloud2->publish(plannerCloudMsg);

            // int boundaryCloudSize = boundaryCloud->

            float pathRange = adjacentRange;
            if (pathRange)
            {
                pathRange = adjacentRange * joySpeed;
            }
            if (pathRange < minPathRange) 
            {
                pathRange = minPathRange;
            }
            float relativeGoalDis = adjacentRange;
            
            // TODO计算目标角度和距离
            if (autonomyMode)
            {
                
            }
            

        }
        
        rate.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}

