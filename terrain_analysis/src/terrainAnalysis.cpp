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

rclcpp::Node::SharedPtr nh;

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

int minDyObsPointNum = 1;
bool considerDrop = false;


float terrainVoxelSize = 1.0;
int terrainVoxelShiftX = 0;
int terrainVoxelShiftY = 0;
const int terrainVoxelWidth = 21;
int terrainVoxelHalfWidth = (terrainVoxelWidth - 1) / 2;
const int terrainVoxelNum = terrainVoxelWidth * terrainVoxelWidth;
double voxelTimeUpdateThre = 2.0;
int minBlockPointNum = 10;  // 


bool clearDyObs = false;
bool limitGroundLift = false;
double vehicleHeight = 1.5;


// HACK 状态机的参数??
int noDataInited = 0;

double laserCloudTime = 0;
bool newlaserCloud = false;

double systemInitTime = 0;
bool systemInited = false;

double minRelZ = -1.5;
double maxRelZ = 0.2;
double disRatioZ = 0.2;
double voxelPointUpdateThre = 100.0;
bool clearingCloud = false;
double clearingDis = 8.0;

float planarVoxelSize = 0.2;
const int planarVoxelWidth = 51;
int planarVoxelHalfWidth = (planarVoxelWidth - 1)/2;
const int planarVoxelNum = planarVoxelWidth * planarVoxelWidth;
bool noDataObstacle = false;
int noDataBlockSkipNum = 0;

bool useSorting = true;

bool quantileZ = 0.25;

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
// 一维数组
float planarVoxelElev[planarVoxelNum] = {0};
int planarVoxelEdge[planarVoxelNum] = {0};
int planarVoxelDyObs[planarVoxelNum] = {0};
// planarPointElev不是一个一维数组 而是外层是一个数组 里面每个元素是std::vector<float>用于存放z值
std::vector<float> planarPointElev[planarVoxelNum];

double maxGroundLift = 0.15;

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
    // laser的frame id 
    // RCLCPP_INFO() laserCloud2->header.frame_id.c_str()
    // RCLCPP_INFO(nh->get_logger(),"雷达的frame id : %s",laserCloud2->header.frame_id.c_str());
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
        // laserCloudCrop只保存一定高度的点 
        // 后面terrainVoxelCloud的点来自于laserCloudCrop
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
    nh = rclcpp::Node::make_shared("terrainAnalysis");


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

    auto subLaserCloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>("registered_scan",5,laserCloudHandler);

    // ========================= test ========================
    
    auto pubRefreshCloud = nh->create_publisher<sensor_msgs::msg::PointCloud2>("refresh_cloud",2);
    
    // 发布机器人中心的点云
    auto pubCenterCloud = nh->create_publisher<sensor_msgs::msg::PointCloud2>("center_cloud",2);

    
    // ========================= test ========================
    
    
    // 对terrainVoxelCloud初始化
    for (int i = 0; i < terrainVoxelNum ;i++)
    {
        terrainVoxelCloud[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    }

    downSizeFilter.setLeafSize(scanVoxelSize,scanVoxelSize,scanVoxelSize);

    rclcpp::Rate rate(100);
    bool status = rclcpp::ok();
    while (status)
    {
        // RCLCPP_INFO(nh->get_logger(), "newlaserCloud value: %s", newlaserCloud ? "true" : "false");
        rclcpp::spin_some(nh);
        if (newlaserCloud)
        {
            // RCLCPP_INFO(nh->get_logger(),"new cloud");
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

                    //TAG测试 把terrainVoxelCloudPtr转成ros格式
                    // ================================   =========================================
                    // pcl::fromROSMsg
                    sensor_msgs::msg::PointCloud2 refreshCloud;
                    pcl::toROSMsg(*terrainVoxelCloudPtr,refreshCloud);
                    refreshCloud.header.stamp = nh->get_clock()->now();
                    refreshCloud.header.frame_id = "camera_init";
                    pubRefreshCloud->publish(refreshCloud);

                    terrainVoxelUpdateNum[ind] = 0;
                    terrainVoxelUpdateTime[ind] = laserCloudTime - systemInitTime;

                }
                
            }
            
            terrainCloud->clear();
            // 提取机器人中心的 11 * 11点云
            for (int indX = terrainVoxelHalfWidth - 5; indX <= terrainVoxelHalfWidth + 5; indX++)
            {
                for (int indY = terrainVoxelHalfWidth - 5; indY <= terrainVoxelHalfWidth; indY++)
                {
                    *terrainCloud += *terrainVoxelCloud[terrainVoxelWidth * indX + indY];
                }
            }
            // TAG测试
            sensor_msgs::msg::PointCloud2 centerCloud;
            pcl::toROSMsg(*terrainCloud,centerCloud);
            centerCloud.header.frame_id = "camera_init";
            centerCloud.header.stamp = nh->get_clock()->now();
            pubCenterCloud->publish(centerCloud);
            
            // estimate ground and compute elevation for each point
            for (int i = 0; i < planarVoxelNum; i++)
            {
                planarVoxelElev[i] = 0;
                planarVoxelEdge[i] = 0;
                planarVoxelDyObs[i] = 0;
                planarPointElev[i].clear();
            }
            
            // 遍历terrainCloud点云
            int terrainCloudSize = terrainCloud->points.size();
            for (int i = 0; i < terrainCloudSize; i++)
            {
                point = terrainCloud->points[i];
                
                int indX =
                    int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) +
                    planarVoxelHalfWidth;
                int indY =
                    int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) +
                    planarVoxelHalfWidth;

                if (point.x - vehicleX + planarVoxelSize/2 < 0)
                {
                    indX--;
                }
                if (point.y - vehicleY + planarVoxelSize/2 < 0)
                {
                    indY--;
                }

                // 只考虑相对车辆高度在(minRelZ,maxRelZ)范围内的点
                if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ)
                {
                    for (int dX = -1; dX <=1; dX++)
                    {
                        for (int dY = -1; dY <=1; dY++)
                        {
                            if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                                indY + dY >= 0 && indY + dY < planarVoxelWidth)
                            {
                                // 将选中点的八邻域也添加进来
                                // 采集候选的z值
                                // 这里将二维索引转成一维索引
                                planarPointElev[planarVoxelWidth * (indX + dX) + indY + dY].push_back(point.z);
                            }
                            
                        }
                        
                    }
                }

                // 是否启用障碍物清除
                if (clearDyObs)
                {
                    // TODO后续补充障碍物清除的逻辑

                    /* code */
                }
            
            }

            // TODO后续补充障碍物清除的逻辑
            if (clearDyObs)
            {
                /* code */
            }
            
            // 两种地面估计的方法
            if (useSorting) 
            {
                // 这里的planarVoxelNum就是网格的面积
                for (int i = 0; i < planarVoxelNum; i++) 
                {
                    int planarPointElevSize = planarPointElev[i].size();

                    if (planarPointElevSize > 0) 
                    {
                        // 对voxel内的点云z值进行排序
                        sort(planarPointElev[i].begin(), planarPointElev[i].end());
                        // 找到对应的分位点quantileID 
                        // 如果分位点quantileID 过低就取第一个
                        int quantileID = int(quantileZ * planarPointElevSize);
                        if (quantileID < 0)
                        {
                            quantileID = 0;
                        }
                        // 过大就取最后一个
                        else if (quantileID >= planarPointElevSize)
                        {
                            quantileID = planarPointElevSize - 1;
                        }

                        // planarPointElev[i][quantileID]体素内的某个分位数高度
                        // planarPointElev[i][0]最低的z值
                        // planarPointElev是std::vector<float> 类型
                        if (planarPointElev[i][quantileID] >
                                planarPointElev[i][0] + maxGroundLift &&
                            limitGroundLift) 
                        {
                            planarVoxelElev[i] = planarPointElev[i][0] + maxGroundLift;
                        } else {
                            planarVoxelElev[i] = planarPointElev[i][quantileID];
                        }
                    }
                }
            } else {
                // 最小值法 找到最小值z 认为最低的点就是地面
                for (int i = 0; i < planarVoxelNum; i++) {
                int planarPointElevSize = planarPointElev[i].size();
                if (planarPointElevSize > 0) {
                    float minZ = 1000.0;
                    int minID = -1;
                    for (int j = 0; j < planarPointElevSize; j++) {
                    if (planarPointElev[i][j] < minZ) {
                        minZ = planarPointElev[i][j];
                        minID = j;
                    }
                    }

                    if (minID != -1) {
                    planarVoxelElev[i] = planarPointElev[i][minID];
                    }
                }
                }
            }


            terrainCloudElev->clear();
            int terrainCloudElevSize = 0;
            for (int i = 0; i < terrainCloudSize; i++) 
            {
                point = terrainCloud->points[i];
                // 保留高度一定的点
                if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ) 
                {
                    int indX = int((point.x - vehicleX + planarVoxelSize / 2) /
                                    planarVoxelSize) +
                                planarVoxelHalfWidth;
                    int indY = int((point.y - vehicleY + planarVoxelSize / 2) /
                                    planarVoxelSize) +
                                planarVoxelHalfWidth;

                    if (point.x - vehicleX + planarVoxelSize / 2 < 0)
                        indX--;
                    if (point.y - vehicleY + planarVoxelSize / 2 < 0)
                        indY--;

                    // 有效点
                    if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
                        indY < planarVoxelWidth) 
                    {
                        if (planarVoxelDyObs[planarVoxelWidth * indX + indY] <
                                minDyObsPointNum ||
                            !clearDyObs) 
                        {
                            // 
                            float disZ = point.z - planarVoxelElev[planarVoxelWidth * indX + indY];
                            if (considerDrop)
                            {
                                disZ = fabs(disZ);
                            }
                            int planarPointElevSize = planarPointElev[planarVoxelWidth * indX + indY].size();
                            if (disZ >= 0 && disZ < vehicleHeight &&
                                planarPointElevSize >= minBlockPointNum) 
                            {
                                terrainCloudElev->push_back(point);
                                // 将intensity重写成高度
                                terrainCloudElev->points[terrainCloudElevSize].intensity = disZ;
                                terrainCloudElevSize++;
                            }
                        }
                    }
                }
            }
            
            // 将雷达看不到地方标记为障碍物
            // HACK没有细看
            if (noDataObstacle && noDataInited == 2) {
                for (int i = 0; i < planarVoxelNum; i++) {
                    int planarPointElevSize = planarPointElev[i].size();
                    if (planarPointElevSize < minBlockPointNum) {
                        planarVoxelEdge[i] = 1; // 点太少 认为格子无效
                    }
                }

                for (int noDataBlockSkipCount = 0;noDataBlockSkipCount < noDataBlockSkipNum;noDataBlockSkipCount++) {
                    for (int i = 0; i < planarVoxelNum; i++) 
                    {
                        if (planarVoxelEdge[i] >= 1) 
                        {
                            int indX = int(i / planarVoxelWidth);
                            int indY = i % planarVoxelWidth;
                            bool edgeVoxel = false;
                            for (int dX = -1; dX <= 1; dX++) {
                                for (int dY = -1; dY <= 1; dY++) 
                                {
                                    if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                                        indY + dY >= 0 && indY + dY < planarVoxelWidth) {
                                        if (planarVoxelEdge[planarVoxelWidth * (indX + dX) + indY +
                                                            dY] < planarVoxelEdge[i]) {
                                        edgeVoxel = true;
                                        }
                                    }
                                }
                            }

                            if (!edgeVoxel)
                            {
                                planarVoxelEdge[i]++;
                            }
                        }
                    }
                }

                for (int i = 0; i < planarVoxelNum; i++) {
                    if (planarVoxelEdge[i] > noDataBlockSkipNum) {
                        int indX = int(i / planarVoxelWidth);
                        int indY = i % planarVoxelWidth;

                        point.x =
                            planarVoxelSize * (indX - planarVoxelHalfWidth) + vehicleX;
                        point.y =
                            planarVoxelSize * (indY - planarVoxelHalfWidth) + vehicleY;
                        point.z = vehicleZ;
                        point.intensity = vehicleHeight;

                        point.x -= planarVoxelSize / 4.0;
                        point.y -= planarVoxelSize / 4.0;
                        terrainCloudElev->push_back(point);

                        point.x += planarVoxelSize / 2.0;
                        terrainCloudElev->push_back(point);

                        point.y += planarVoxelSize / 2.0;
                        terrainCloudElev->push_back(point);

                        point.x -= planarVoxelSize / 2.0;
                        terrainCloudElev->push_back(point);
                    }
                }
            }

            clearingCloud = false;

            // publish points with elevation
            sensor_msgs::msg::PointCloud2 terrainCloud2;
            pcl::toROSMsg(*terrainCloudElev, terrainCloud2);
            terrainCloud2.header.stamp = rclcpp::Time(static_cast<uint64_t>(laserCloudTime * 1e9));
            terrainCloud2.header.frame_id = "camera_init";
            pubLaserCloud->publish(terrainCloud2);
            

        }

        status = rclcpp::ok();
        rate.sleep();
    }
    
    
    return 0;
}

