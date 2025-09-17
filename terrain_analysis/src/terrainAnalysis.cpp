#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

const double PI = 3.1415926;

double noDecayDis = 4.0;
double decayTime = 2.0;
double scanVoxelSize = 0.05;
float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
float vehicleXRec = 0, vehicleYRec = 0;

float sinVehicleRoll = 0, cosVehicleRoll = 0;
float sinVehiclePitch = 0, cosVehiclePitch = 0;
float sinVehicleYaw = 0, cosVehicleYaw = 0;

float terrainVoxelSize = 1.0;
int terrainVoxelShiftX = 0;
int terrainVoxelShiftY = 0;
const int terrainVoxelWidth = 21;
int terrainVoxelHalfWidth = (terrainVoxelWidth - 1) / 2;
const int terrainVoxelNum = terrainVoxelWidth * terrainVoxelWidth;
double voxelTimeUpdateThre = 2.0;



// HACK 状态机的参数??
int noDataInited = 0;

double laserCloudTime = 0;
bool newlaserCloud = false;

double systemInitTime = 0;
bool systemInited = false;

double minRelZ = -1.5;
double maxRelZ = 0.2;
double disRatioZ = 0.2;
int voxelPointUpdateThre = 100;
bool clearingCloud = false;
double clearingDis = 8.0;


pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    terrainCloudElev(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloud[terrainVoxelNum];

pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;


int terrainVoxelUpdateNum[terrainVoxelNum] = {0};
float terrainVoxelUpdateTime[terrainVoxelNum] = {0};

void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
    if (joy->buttons[5] > 0.5)
    {
        noDataInited = 0;
        clearingCloud = true;
    }
}

void clearingHandler(const std_msgs::msg::Float32::ConstSharedPtr dis)
{
    noDataInited = 0;
    clearingDis =dis->data;
    clearingCloud = true;
}


void odometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
    double roll,pitch,yaw;
    geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
    tf2::Matrix3x3(tf2::Quaternion(geoQuat.x,geoQuat.y,geoQuat.z,geoQuat.w)).getRPY(roll,pitch,yaw);

    vehicleRoll = roll;
    vehiclePitch  = pitch;
    vehicleYaw = yaw;
    vehicleX = odom->pose.pose.position.x;
    vehicleY = odom->pose.pose.position.y;
    vehicleZ = odom->pose.pose.position.z;

    sinVehicleRoll = sin(vehicleRoll);
    cosVehicleRoll = cos(vehicleRoll);
    sinVehiclePitch = sin(vehiclePitch);
    cosVehiclePitch = cos(vehiclePitch);
    sinVehicleYaw = sin(vehicleYaw);
    cosVehicleYaw = cos(vehicleYaw);

    if (noDataInited == 0) {
        // 还没有移动
        vehicleXRec = vehicleX;
        vehicleYRec = vehicleY;
        noDataInited = 1;
    }
    if (noDataInited == 1) {
        // 减去初始化时的位置
        float dis = sqrt((vehicleX - vehicleXRec) * (vehicleX - vehicleXRec) +
                        (vehicleY - vehicleYRec) * (vehicleY - vehicleYRec));
        if (dis >= noDecayDis)
            noDataInited = 2;
    }  
};

// registered laser scan callback function
void laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud2)
{
    laserCloudTime = rclcpp::Time(laserCloud2->header.stamp).seconds();
    if (!systemInited)
    {
        systemInitTime = laserCloudTime;
        systemInited = true;
    }

    laserCloud->clear();
    // 将ros的laserCloud2转成pcl的laserCloud
    pcl::fromROSMsg(*laserCloud2,*laserCloud);

    pcl::PointXYZI point;
    laserCloudCrop->clear();
    int laserCloudSize = laserCloud->points.size();
    for (int i = 0; i < laserCloudSize; i++)
    {
        point = laserCloud->points[i];

        float pointX = point.x;
        float pointY = point.y;
        float pointZ = point.z;

        float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) +
                         (pointY - vehicleY) * (pointY - vehicleY));
        if (pointZ - vehicleZ > minRelZ - disRatioZ * dis &&
            pointZ - vehicleZ < maxRelZ + disRatioZ * dis &&
            dis < terrainVoxelSize * (terrainVoxelHalfWidth + 1)) {
        point.x = pointX;
        point.y = pointY;
        point.z = pointZ;
        point.intensity = laserCloudTime - systemInitTime;
        laserCloudCrop->push_back(point);
        }
    }
    
    newlaserCloud = true;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto nh = rclcpp::Node::make_shared("terrainAnalysis");

    nh->declare_parameter<double>("scanVoxelSize",scanVoxelSize);
    nh->declare_parameter<double>("voxelPointUpdateThre",voxelPointUpdateThre);
    nh->declare_parameter<double>("voxelTimeUpdateThre",voxelTimeUpdateThre);
    nh->declare_parameter<double>("decayTime",decayTime);

    nh->get_parameter("scanVoxelSize",scanVoxelSize);
    nh->get_parameter("voxelPointUpdateThre",voxelPointUpdateThre);
    nh->get_parameter("voxelTimeUpdateThre",voxelTimeUpdateThre);
    nh->get_parameter("decayTime",decayTime);
    // 

    // 订阅odom 这里通过fast lio提供odom Odometry
    // NOTE原来的代码里面是state_estimation 
    auto subOdometry = nh->create_subscription<nav_msgs::msg::Odometry>("Odometry",5,odometryHandler);

    auto pubLaserCloud = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/terrain_map",2);

    // 对terrainVoxelCloud初始化
    for (int i = 0; i < terrainVoxelSize ;i++)
    {
        terrainVoxelCloud[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    }

    downSizeFilter.setLeafSize(scanVoxelSize,scanVoxelSize,scanVoxelSize);

    rclcpp::Rate rate(100);
    bool status = rclcpp::ok();
    while (status)
    {
        rclcpp::spin_some(nh);
        if (newlaserCloud)
        {
            newlaserCloud = false;

            // 获取当前地图中心坐标
            float terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
            float terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
            // HACK这一部分是在调整terrainVoxelShiftX吗
            // 判断是否需要向左滚动地图
            // 如果X负方向超过了一个体素格子的宽度
            while (vehicleX - terrainVoxelCenX < -terrainVoxelSize) {
                // 保存最右一列的点云指针
                for (int indY = 0; indY < terrainVoxelWidth; indY++) {
                // terrainVoxelCloud是用一维数组的方式存储 index = indX * terrainVoxelWidth + indY 
                // *terrainVoxelWidth是为了跳到第indX列   X是列 Y是行
                /*
                        indY=4   4   9   14   19   24
                        indY=3   3   8   13   18   23
                        indY=2   2   7   12   17   22
                        indY=1   1   6   11   16   21
                        indY=0   0   5   10   15   20
                                ^   ^   ^    ^    ^
                                X=0 X=1 X=2  X=3  X=4
                */
                pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
                    // (terrainVoxelWidth - 1) 就是最后一列 然后跳过每一列的terrainVoxelWidth个数据
                    // +indY就可以遍历的拿到每一个数据
                    terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) +
                                        indY];
                // 把倒数第二列的数据覆盖到最后一列 整体向右平移一列
                for (int indX = terrainVoxelWidth - 1; indX >= 1; indX--) {
                    terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                        terrainVoxelCloud[terrainVoxelWidth * (indX - 1) + indY];
                }
                // 保存到最左侧的位置
                // terrainVoxelCloudPtr原来保存的是最右侧的点云的数据 现在赋值给最左侧
                terrainVoxelCloud[indY] = terrainVoxelCloudPtr;
                // 清空里面的数据 准备接受新的点云
                terrainVoxelCloud[indY]->clear();
                }
                // 窗口中心在全局坐标向负方向移动一格格子
                terrainVoxelShiftX--;
                // 窗口中心新的X坐标
                terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
            }

            while (vehicleX - terrainVoxelCenX > terrainVoxelSize) {
                for (int indY = 0; indY < terrainVoxelWidth; indY++) {
                pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
                    terrainVoxelCloud[indY];
                for (int indX = 0; indX < terrainVoxelWidth - 1; indX++) {
                    terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                        terrainVoxelCloud[terrainVoxelWidth * (indX + 1) + indY];
                }
                terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) +
                                    indY] = terrainVoxelCloudPtr;
                terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY]
                    ->clear();
                }
                terrainVoxelShiftX++;
                terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
            }

            while (vehicleY - terrainVoxelCenY < -terrainVoxelSize) {
                for (int indX = 0; indX < terrainVoxelWidth; indX++) {
                pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
                    terrainVoxelCloud[terrainVoxelWidth * indX +
                                        (terrainVoxelWidth - 1)];
                for (int indY = terrainVoxelWidth - 1; indY >= 1; indY--) {
                    terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                        terrainVoxelCloud[terrainVoxelWidth * indX + (indY - 1)];
                }
                terrainVoxelCloud[terrainVoxelWidth * indX] = terrainVoxelCloudPtr;
                terrainVoxelCloud[terrainVoxelWidth * indX]->clear();
                }
                terrainVoxelShiftY--;
                terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
            }

            while (vehicleY - terrainVoxelCenY > terrainVoxelSize) {
                for (int indX = 0; indX < terrainVoxelWidth; indX++) {
                pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
                    terrainVoxelCloud[terrainVoxelWidth * indX];
                for (int indY = 0; indY < terrainVoxelWidth - 1; indY++) {
                    terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                        terrainVoxelCloud[terrainVoxelWidth * indX + (indY + 1)];
                }
                terrainVoxelCloud[terrainVoxelWidth * indX +
                                    (terrainVoxelWidth - 1)] = terrainVoxelCloudPtr;
                terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)]
                    ->clear();
                }
                terrainVoxelShiftY++;
                terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
            }

            // stack registered laser scans
            pcl::PointXYZI point;
            // 添加降采样后的点
            int laserCloudCropSize = laserCloudCrop->points.size();
            for (int i = 0; i < laserCloudCropSize; i++)
            {
                point = laserCloudCrop->points[i];
                // point.x - vehicleX 点相对于车的X偏移 dx
                // terrainVoxelSize 体素边长 s
                // h = s/2 半个体素 
                // dx + h 平移半个体素 把分割线从体素边界平移到格子中心 (point.x - vehicleX + terrainVoxelSize / 2)
                // (dx + h)/s 判断落到哪个格子 
                // int() 得到整数部分
                // 
                int indX = int((point.x - vehicleX + terrainVoxelSize / 2) /
                                terrainVoxelSize)+ 
                                terrainVoxelHalfWidth;
                int indY = int((point.y - vehicleY + terrainVoxelSize / 2) /
                                terrainVoxelSize)+
                                terrainVoxelHalfWidth;
                // HACK没有特别的理解
                if (point.x - vehicleX + terrainVoxelSize / 2 < 0)
                {
                    // cpp中int是向0截断 -0.6 -> 0.0
                    // -1.6 -> -1 
                    // 但是期望给到-1
                    indX--;
                }
                if (point.y - vehicleY + terrainVoxelSize / 2 < 0)
                {
                    indY--;
                }
                
                if (indX >= 0 && indX < terrainVoxelWidth && indY >=0 && indY < terrainVoxelWidth)
                {
                    // 只有在terrainVoxelWidth范围内才会被添加进来
                    terrainVoxelCloud[terrainVoxelWidth*indX + indY]->push_back(point);
                    terrainVoxelUpdateNum[terrainVoxelWidth * indX + indY]++;
                }
            }

            // 遍历每个格子查看是否需要刷新
            for (int ind = 0; ind < terrainVoxelNum; ind++)
            {
                // 点数达到阈值 时间超时 全局清理标志
                if (terrainVoxelUpdateNum[ind] >= voxelPointUpdateThre || 
                    laserCloudTime - systemInitTime - terrainVoxelUpdateTime[ind] >= voxelTimeUpdateThre ||
                    clearingCloud)
                {
                    // 对点云进行下采样
                    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr = terrainVoxelCloud[ind];
                    laserCloudDwz->clear();
                    downSizeFilter.setInputCloud(terrainVoxelCloudPtr);
                    downSizeFilter.filter(*laserCloudDwz);

                    // 清理旧点云
                    terrainVoxelCloudPtr->clear();
                    int laserCloudDwzSize = laserCloudDwz->points.size();
                    for (int i = 0; i < laserCloudDwzSize; i++)
                    {
                        point = laserCloudDwz->points[i];
                        float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                                        (point.y - vehicleY) * (point.y - vehicleY));

                        if (point.z - vehicleZ > minRelZ - disRatioZ * dis &&
                            point.z - vehicleZ < maxRelZ + disRatioZ * dis &&
                            (laserCloudTime - systemInitTime - point.intensity <
                                decayTime ||
                            dis < noDecayDis) &&
                            !(dis < clearingDis && clearingCloud)) {
                        terrainVoxelCloudPtr->push_back(point);
                        }
                    }

                    terrainVoxelUpdateNum[ind] = 0;
                    terrainVoxelUpdateTime[ind] = laserCloudTime - systemInitTime;

                }
                
            }
            
            



        }
        
        
    }
    
    

    return 0;
}

