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
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_ros/transform_listener.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

const double PI = 3.1415926;

#define PLOTPATHSET 1

std::string pathFolder;

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

bool newlaserCloud = false;


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

// 用于将 plannerCloud 转换到 body 坐标系下面 然后计算点云到车辆的距离 构造出 plannerCloudCrop
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloudBody(new pcl::PointCloud<pcl::PointXYZI>());

// startPaths数组 每个元素都是PointXYZ 标记的是路径的起点
pcl::PointCloud<pcl::PointXYZ>::Ptr startPaths[groupNum];
#if PLOTPATHSET == 1
pcl::PointCloud<pcl::PointXYZI>::Ptr paths[pathNum];
pcl::PointCloud<pcl::PointXYZI>::Ptr freePaths(new pcl::PointCloud<pcl::PointXYZI>());
#endif



static int lastSelectedGroupID = -1;  // 保存上一帧选择

float maxScore = 0;
int selectedGroupID = -1;
float scoreThreshold = maxScore * 0.9;  // 得分达到90%就算接近

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
    vehicleX = odom->pose.pose.position.x;
    vehicleY = odom->pose.pose.position.y;
    // 打印 vehicleX 和 vehicleY的值
    RCLCPP_INFO(nh->get_logger(),"vehicleX :%f , vehicleY :%f",vehicleX,vehicleY);
    vehicleZ = odom->pose.pose.position.z;

}

void laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud2)
{
    // RCLCPP_INFO(nh->get_logger(),"laserCloudHandler");
    if (!useTerrainAnalysis)
    {
        // 这个if语句基本不会进入的
        // RCLCPP_INFO(nh->get_logger(),"laserCloudHandler useTerrainAnalysis");
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

        newlaserCloud = true;
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
    // RCLCPP_INFO(nh->get_logger(),"目标点的frame id是 %s",frame_id.c_str());
    goalX = goal->point.x;
    goalY = goal->point.y;
}

int readPlyHeader(FILE *filePtr)
{
  char str[50];
  int val, pointNum;
  std::string strCur, strLast;
  while (strCur != "end_header") {
    val = fscanf(filePtr, "%s", str);
    if (val != 1) {
      RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
      exit(1);
    }

    strLast = strCur;
    strCur = std::string(str);

    if (strCur == "vertex" && strLast == "element") {
      val = fscanf(filePtr, "%d", &pointNum);
      if (val != 1) {
        RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
        exit(1);
      }
    }
  }

  return pointNum;
}

void readStartPaths()
{
  std::string fileName = pathFolder + "/startPaths.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    RCLCPP_INFO(nh->get_logger(), "Cannot read input files, exit.");
    exit(1);
  }

  int pointNum = readPlyHeader(filePtr);

  pcl::PointXYZ point;
  int val1, val2, val3, val4, groupID;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%d", &groupID);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1) {
      RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
        exit(1);
    }

    if (groupID >= 0 && groupID < groupNum) {
      startPaths[groupID]->push_back(point);
    }
  }

  fclose(filePtr);
}

#if PLOTPATHSET == 1
void readPaths()
{
  std::string fileName = pathFolder + "/paths.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    RCLCPP_INFO(nh->get_logger(), "Cannot read input files, exit.");
    exit(1);
  }

  int pointNum = readPlyHeader(filePtr);

  pcl::PointXYZI point;
  int pointSkipNum = 30;
  int pointSkipCount = 0;
  int val1, val2, val3, val4, val5, pathID;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%d", &pathID);
    val5 = fscanf(filePtr, "%f", &point.intensity);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
      RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
        exit(1);
    }

    if (pathID >= 0 && pathID < pathNum) {
      pointSkipCount++;
      if (pointSkipCount > pointSkipNum) {
        paths[pathID]->push_back(point);
        pointSkipCount = 0;
      }
    }
  }

  fclose(filePtr);
}
#endif

void readPathList()
{
  std::string fileName = pathFolder + "/pathList.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    RCLCPP_INFO(nh->get_logger(), "Cannot read input files, exit.");
    exit(1);
  }

  if (pathNum != readPlyHeader(filePtr)) {
    RCLCPP_INFO(nh->get_logger(), "Incorrect path number, exit.");
    exit(1);
  }

  int val1, val2, val3, val4, val5, pathID, groupID;
  float endX, endY, endZ;
  for (int i = 0; i < pathNum; i++) {
    val1 = fscanf(filePtr, "%f", &endX);
    val2 = fscanf(filePtr, "%f", &endY);
    val3 = fscanf(filePtr, "%f", &endZ);
    val4 = fscanf(filePtr, "%d", &pathID);
    val5 = fscanf(filePtr, "%d", &groupID);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
      RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
        exit(1);
    }

    if (pathID >= 0 && pathID < pathNum && groupID >= 0 && groupID < groupNum) {
      pathList[pathID] = groupID;
      endDirPathList[pathID] = 2.0 * atan2(endY, endX) * 180 / PI;
    }
  }

  fclose(filePtr);
}

void readCorrespondences()
{
  std::string fileName = pathFolder + "/correspondences.txt";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    RCLCPP_INFO(nh->get_logger(), "Cannot read input files, exit.");
    exit(1);
  }

  int val1, gridVoxelID, pathID;
  for (int i = 0; i < gridVoxelNum; i++) {
    val1 = fscanf(filePtr, "%d", &gridVoxelID);
    if (val1 != 1) {
      RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
        exit(1);
    }

    while (1) {
      val1 = fscanf(filePtr, "%d", &pathID);
      if (val1 != 1) {
        RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
          exit(1);
      }

      if (pathID != -1) {
        if (gridVoxelID >= 0 && gridVoxelID < gridVoxelNum && pathID >= 0 && pathID < pathNum) {
          correspondences[gridVoxelID].push_back(pathID);
        }
      } else {
        break;
      }
    }
  }

  fclose(filePtr);
}



// TODO需要将free path补充出来 
int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);

    // 根据odom来更新vehicle的值
    nh = rclcpp::Node::make_shared("localPlanner");

    nh->declare_parameter<std::string>("pathFolder", pathFolder);
    nh->declare_parameter<double>("vehicleLength", vehicleLength);
    nh->declare_parameter<double>("vehicleWidth", vehicleWidth);
    nh->declare_parameter<double>("sensorOffsetX", sensorOffsetX);
    nh->declare_parameter<double>("sensorOffsetY", sensorOffsetY);
    nh->declare_parameter<bool>("twoWayDrive", twoWayDrive);
    nh->declare_parameter<double>("laserVoxelSize", laserVoxelSize);
    nh->declare_parameter<double>("terrainVoxelSize", terrainVoxelSize);
    nh->declare_parameter<bool>("useTerrainAnalysis", useTerrainAnalysis);
    nh->declare_parameter<bool>("checkObstacle", checkObstacle);
    nh->declare_parameter<bool>("checkRotObstacle", checkRotObstacle);
    nh->declare_parameter<double>("adjacentRange", adjacentRange);
    nh->declare_parameter<double>("obstacleHeightThre", obstacleHeightThre);
    nh->declare_parameter<double>("groundHeightThre", groundHeightThre);
    nh->declare_parameter<double>("costHeightThre", costHeightThre);
    nh->declare_parameter<double>("costScore", costScore);
    nh->declare_parameter<bool>("useCost", useCost);
    nh->declare_parameter<int>("pointPerPathThre", pointPerPathThre);
    nh->declare_parameter<double>("minRelZ", minRelZ);
    nh->declare_parameter<double>("maxRelZ", maxRelZ);
    nh->declare_parameter<double>("maxSpeed", maxSpeed);
    nh->declare_parameter<double>("dirWeight", dirWeight);
    nh->declare_parameter<double>("dirThre", dirThre);
    nh->declare_parameter<bool>("dirToVehicle", dirToVehicle);
    nh->declare_parameter<double>("pathScale", pathScale);
    nh->declare_parameter<double>("minPathScale", minPathScale);
    nh->declare_parameter<double>("pathScaleStep", pathScaleStep);
    nh->declare_parameter<bool>("pathScaleBySpeed", pathScaleBySpeed);
    nh->declare_parameter<double>("minPathRange", minPathRange);
    nh->declare_parameter<double>("pathRangeStep", pathRangeStep);
    nh->declare_parameter<bool>("pathRangeBySpeed", pathRangeBySpeed);
    nh->declare_parameter<bool>("pathCropByGoal", pathCropByGoal);
    nh->declare_parameter<bool>("autonomyMode", autonomyMode);
    nh->declare_parameter<double>("autonomySpeed", autonomySpeed);
    nh->declare_parameter<double>("joyToSpeedDelay", joyToSpeedDelay);
    nh->declare_parameter<double>("joyToCheckObstacleDelay", joyToCheckObstacleDelay);
    nh->declare_parameter<double>("goalClearRange", goalClearRange);
    nh->declare_parameter<double>("goalX", goalX);
    nh->declare_parameter<double>("goalY", goalY);

    nh->get_parameter("pathFolder", pathFolder);
    nh->get_parameter("vehicleLength", vehicleLength);
    nh->get_parameter("vehicleWidth", vehicleWidth);
    nh->get_parameter("sensorOffsetX", sensorOffsetX);
    nh->get_parameter("sensorOffsetY", sensorOffsetY);
    nh->get_parameter("twoWayDrive", twoWayDrive);
    nh->get_parameter("laserVoxelSize", laserVoxelSize);
    nh->get_parameter("terrainVoxelSize", terrainVoxelSize);
    nh->get_parameter("useTerrainAnalysis", useTerrainAnalysis);
    nh->get_parameter("checkObstacle", checkObstacle);
    nh->get_parameter("checkRotObstacle", checkRotObstacle);
    nh->get_parameter("adjacentRange", adjacentRange);
    nh->get_parameter("obstacleHeightThre", obstacleHeightThre);
    nh->get_parameter("groundHeightThre", groundHeightThre);
    nh->get_parameter("costHeightThre", costHeightThre);
    nh->get_parameter("costScore", costScore);
    nh->get_parameter("useCost", useCost);
    nh->get_parameter("pointPerPathThre", pointPerPathThre);
    nh->get_parameter("minRelZ", minRelZ);
    nh->get_parameter("maxRelZ", maxRelZ);
    nh->get_parameter("maxSpeed", maxSpeed);
    nh->get_parameter("dirWeight", dirWeight);
    nh->get_parameter("dirThre", dirThre);
    nh->get_parameter("dirToVehicle", dirToVehicle);
    nh->get_parameter("pathScale", pathScale);
    nh->get_parameter("minPathScale", minPathScale);
    nh->get_parameter("pathScaleStep", pathScaleStep);
    nh->get_parameter("pathScaleBySpeed", pathScaleBySpeed);
    nh->get_parameter("minPathRange", minPathRange);
    nh->get_parameter("pathRangeStep", pathRangeStep);
    nh->get_parameter("pathRangeBySpeed", pathRangeBySpeed);
    nh->get_parameter("pathCropByGoal", pathCropByGoal);
    nh->get_parameter("autonomyMode", autonomyMode);
    nh->get_parameter("autonomySpeed", autonomySpeed);
    nh->get_parameter("joyToSpeedDelay", joyToSpeedDelay);
    nh->get_parameter("joyToCheckObstacleDelay", joyToCheckObstacleDelay);
    nh->get_parameter("goalClearRange", goalClearRange);
    nh->get_parameter("goalX", goalX);
    nh->get_parameter("goalY", goalY);


    // 这里要适配fast lio2 将/odom改为/Odometry
    auto subOdometry = nh->create_subscription<nav_msgs::msg::Odometry>("/Odometry",5,odometryHandle);

    // 这里雷达的坐标系也需要转换一下
    auto subLaserCloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>("/registered_scan", 5, laserCloudHandler);

    auto subTerrainCloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>("/terrain_map",5,terrainCloudHandler);

    // NOTE:local planner中订阅的话题是way_point
    auto subGoal = nh->create_subscription<geometry_msgs::msg::PointStamped>("/way_point",5,goalHandler);

    auto pubLaserCloud = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/plannerCloud",5);
    auto pubLaserCloud2 = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/plannerCloudCropPlanner",5);

    auto pubObstacleCloud = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/visObstacleCloud",5);

    auto pubMarker = nh->create_publisher<visualization_msgs::msg::MarkerArray>("path_debug", 10);

    auto subBoundary = nh->create_subscription<geometry_msgs::msg::PolygonStamped>("/navigation_boundary",5,boundaryHandle);


    auto pubPath = nh->create_publisher<nav_msgs::msg::Path>("/local_path",5);

    #if PLOTPATHSET == 1
    auto pubFreePaths = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/free_paths",2);
    #endif

    laserDwzFilter.setLeafSize(laserVoxelSize, laserVoxelSize, laserVoxelSize);
    terrainDwzFilter.setLeafSize(terrainVoxelSize, terrainVoxelSize, terrainVoxelSize);

    // 初始化tf
    tf2_ros::Buffer tfBuffer(nh->get_clock());
    tf2_ros::TransformListener tfListener(tfBuffer);


    RCLCPP_INFO(nh->get_logger(),"Initialization complete.");
    nav_msgs::msg::Path path;


    
    // 重置laserCloudStack
    for (int i = 0; i < laserCloudStackNum; i++)
    {
        laserCloudStack[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    }
    for (int i = 0; i < groupNum; i++) {
        startPaths[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
    }
    #if PLOTPATHSET == 1
    for (int i = 0; i < pathNum; i++) {
        paths[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    }
    #endif
    for (int i = 0; i < gridVoxelNum; i++) {
        correspondences[i].resize(0);
    }

    // 读取路径
    readStartPaths();
    // RCLCPP_INFO(nh->get_logger(),"readStartPaths");
    #if PLOTPATHSET == 1
    readPaths();
    // RCLCPP_INFO(nh->get_logger(),"readPaths");
    #endif
    readPathList();
    // RCLCPP_INFO(nh->get_logger(),"readPathList");
    readCorrespondences();
    // RCLCPP_INFO(nh->get_logger(),"readCorrespondences");
    
    rclcpp::Rate rate(100);
    bool status = rclcpp::ok();
    // TAG设置点云发布的频率
    // rclcpp::Rate rate(50);
    while (status)
    {
        // RCLCPP_INFO(nh->get_logger(),"while ok");
        rclcpp::spin_some(nh);
        // rclcpp::spin(nh);
        
        
        // 先看看有新点云的情况 有新的newlaserCloud 或者 newTerrainCloud都会进入然后分别处理
        // RCLCPP_INFO(nh->get_logger(),"newlaserCloud :%d",newlaserCloud);
        if (newlaserCloud || newTerrainCloud)
        {
            if (newlaserCloud)
            {
                RCLCPP_INFO(nh->get_logger(),"new laser cloud");
                newlaserCloud = false;
    
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
                
                
            }
            
            if (newTerrainCloud)
            {
                // RCLCPP_INFO(nh->get_logger(),"new terrain cloud");
                newTerrainCloud = false;
                
                plannerCloud->clear();
                *plannerCloud = *terrainCloudDwz;
            }
            // TAG设置header 并且需要转成ros msg
            // 其实就是降采样后的点云
            sensor_msgs::msg::PointCloud2 ros_pc2;
            pcl::toROSMsg(*plannerCloud,ros_pc2);
            ros_pc2.header.frame_id = "camera_init";
            ros_pc2.header.stamp = nh->now();

            pubLaserCloud->publish(ros_pc2);

            
            float sinVehicleYaw = sin(vehicleYaw);
            float cosVehicleYaw = cos(vehicleYaw);

            pcl::PointXYZI point;
            plannerCloudCrop->clear();
            // int plannerCloudSize = plannerCloud->points.size();

            // 将 plannerCloud 转换到 body 坐标系下面并保存到 plannerCloudBody 下面
            try
            {
                geometry_msgs::msg::TransformStamped transformStamped;
                transformStamped = tfBuffer.lookupTransform("body","camera_init",rclcpp::Time(0));

                // 转换为Eigen矩阵
                Eigen::Affine3d tf_eigen = tf2::transformToEigen(transformStamped);
                Eigen::Matrix4f tf_matrix = tf_eigen.matrix().cast<float>();

                // 坐标变换
                pcl::transformPointCloud(*plannerCloud,*plannerCloudBody,tf_matrix);
                // RCLCPP_INFO(nh->get_logger(),"plannerCloud transformed from camera_init → lbody successfully, points: %zu", plannerCloudBody->points.size());
            }
            catch(const std::exception& e)
            {
                // std::cerr << e.what() << '\n';
                RCLCPP_WARN(nh->get_logger(),"TF transform (camera_init→lbody) failed: %s", e.what());
                // TODO 这里不能使用原始的点云 可以临时使用
                // HACK ：需要修复
                plannerCloudBody = plannerCloud;
            }
            
            // plannerCloudBody
            int plannerCloudSize = plannerCloudBody->points.size();

            for (int i = 0; i < plannerCloudSize; i++)
            {
                point.x = plannerCloudBody->points[i].x;
                point.y = plannerCloudBody->points[i].y;
                point.z = plannerCloudBody->points[i].z;
                point.intensity = plannerCloudBody->points[i].intensity;

                // HACK 这里需要将坐标系改成livox frame或者自己手动进行坐标变换 
                // 要将livox frame转换到 livox frame坐标系下面 然后再计算距离
                float dis = sqrt(point.x * point.x + point.y * point.y);
                if (dis < adjacentRange && ((point.z > minRelZ && point.z < maxRelZ) || useTerrainAnalysis)) {
                    // RCLCPP_INFO(nh->get_logger(),"push point");
                    // 这里的plannerCloudCrop被转换到了 livox frame坐标系下面
                    plannerCloudCrop->push_back(point);
                }
            }
            // RCLCPP_INFO(nh->get_logger(), "plannerCloudSize: %zu, plannerCloudCropSize1: %zu", 
            //     plannerCloud->points.size(), plannerCloudCrop->points.size());


            // 将plannerCloud显示出来看一看
            // plannerCloud
            sensor_msgs::msg::PointCloud2 plannerCloudMsg;
            pcl::toROSMsg(*plannerCloudCrop,plannerCloudMsg);
            plannerCloudMsg.header.stamp = nh->get_clock()->now();
            plannerCloudMsg.header.frame_id = "livox_frame";
            pubLaserCloud2->publish(plannerCloudMsg);

            // 计算最近与最远的点云的距离
            // RCLCPP_INFO(nh->get_logger(), "plannerCloudCrop size = %ld", plannerCloudCrop->points.size());
            float minDis = 999, maxDis = 0;
            for (auto &p : plannerCloudCrop->points) {
                float d = sqrt(p.x * p.x + p.y * p.y);
                if (d < minDis) minDis = d;
                if (d > maxDis) maxDis = d;
            }
            // RCLCPP_INFO(nh->get_logger(), "[Cloud Stats] minDis=%.3f  maxDis=%.3f", minDis, maxDis);


            // int boundaryCloudSize = boundaryCloud->

            // HACK 这里存在问题
            float pathRange = adjacentRange;
            // if (pathRange)
            // {
            //     pathRange = adjacentRange * joySpeed;
            // }
            pathRange = 4.25;
            
            if (pathRange < minPathRange) 
            {
                pathRange = minPathRange;
            }
            // RCLCPP_INFO(nh->get_logger(),"pathRange : %f",pathRange);
            // RCLCPP_INFO(nh->get_logger(),"adjacentRange : %f",adjacentRange);
            float relativeGoalDis = adjacentRange;
            
            // TODO计算目标角度和距离
            // if (autonomyMode)
            if (true)
            {
                // 假设 transform 是 camera_init -> base_link
                tf2::Transform T_camera_to_base;
                tf2::fromMsg(tfBuffer.lookupTransform("camera_init", "base_link", tf2::TimePointZero).transform, T_camera_to_base);

                // 求逆变换 base_link -> camera_init
                tf2::Transform T_base_to_camera = T_camera_to_base.inverse();

                // 目标点在camera坐标系下
                tf2::Vector3 p_camera(goalX, goalY, 0);

                // 变换到base_link下
                tf2::Vector3 p_base = T_base_to_camera * p_camera;

                // 计算角度
                float dx = p_base.x();
                float dy = p_base.y();
                // 现在joyDir在base link坐标系下面
                float joyDir = std::atan2(dy, dx) * 180 / M_PI;

                RCLCPP_INFO(nh->get_logger(),"dx : %.2f , dy: %.2f joyDir: %.2f",dx,dy,joyDir);

            }
            
            bool pathFound = false;
            float defPathScale = pathScale;
            // 没有理会这里是什么意思
            if (pathScaleBySpeed)
            {
                pathScale = defPathScale * joySpeed;
            }
            if (pathScale < minPathScale)
            {
                pathScale = minPathScale;
            }

            // =================================================================================================
            //                                  TAG 这里开始设置路径
            while (pathScale >= minPathScale && pathRange >= minPathRange) {
                // 清空之前的评分器
                // 总的路径数量 36*pathNum 
                for (int i = 0; i < 36 * pathNum; i++) 
                {
                    clearPathList[i] = 0;
                    pathPenaltyList[i] = 0;
                }
                // 每个方向有groupNum个路径组 每个路径组对应一组不同形状的轨迹
                for (int i = 0; i < 36 * groupNum; i++) 
                {
                    clearPathPerGroupScore[i] = 0;
                }

                float minObsAngCW = -180.0;
                float minObsAngCCW = 180.0;
                float diameter = sqrt(vehicleLength / 2.0 * vehicleLength / 2.0 + vehicleWidth / 2.0 * vehicleWidth / 2.0);
                float angOffset = atan2(vehicleWidth, vehicleLength) * 180.0 / PI;

                // TAG:添加obstacleVisCloud标记哪些被识别成了障碍物
                pcl::PointCloud<pcl::PointXYZI>::Ptr obstacleVisCloud(new pcl::PointCloud<pcl::PointXYZI>());

                // plannerCloudCrop 障碍物点云
                int plannerCloudCropSize = plannerCloudCrop->points.size();
                // 遍历局部感知点云
                // 这里是通过 plannerCloudCrop 来判断的


                for (int i = 0; i < plannerCloudCropSize; i++)
                {
                    // pathScale 路径尺度（路径的大小或长度与某个参考值（如车辆尺寸或环境尺寸）的比例关系），在狭窄的空间中减小路径规模，或在开放的空间中增加路径规模以优化行进路线
                    float x = plannerCloudCrop->points[i].x / pathScale;
                    float y = plannerCloudCrop->points[i].y / pathScale;
                    float h = plannerCloudCrop->points[i].intensity;
                    // 检查plannerCloudCrop没有问题
                    float dis = std::sqrt(x*x + y*y);

                    // TAG 
                    bool cond1 = dis < pathRange / pathScale;
                    // false || true
                    bool cond2 = (dis <= (relativeGoalDis + goalClearRange) / pathScale || !pathCropByGoal);
                    bool cond3 = checkObstacle;

                    // RCLCPP_INFO(nh->get_logger(),
                    //     "[DEBUG] point[%d]: dis=%.3f  rangeLmt=%.3f  goalLmt=%.3f  relGoalDis=%.3f  goalClearRange=%.3f  pathScale=%.3f",
                    //     i, dis, pathRange / pathScale, (relativeGoalDis + goalClearRange) / pathScale,
                    //     relativeGoalDis, goalClearRange, pathScale);

                    // RCLCPP_INFO(nh->get_logger(),
                    //     "          cond1(dis<range)= %d | cond2(goalCrop)= %d | cond3(checkObs)= %d | pathCropByGoal=%d",
                    //     cond1, cond2, cond3, pathCropByGoal);

                    // // 统计skip情况
                    // if (!(cond1 && cond2 && cond3)) {
                    //     RCLCPP_WARN(nh->get_logger(),
                    //         "[WARN] skip point[%d]: dis=%.3f  (cond1=%d cond2=%d cond3=%d) | rangeLmt=%.3f goalLmt=%.3f checkObs=%d",
                    //         i, dis, cond1, cond2, cond3, pathRange / pathScale,
                    //         (relativeGoalDis + goalClearRange) / pathScale, checkObstacle);
                    // }
                    
                    // NOTE:现在的问题是距离过远 导致被过滤掉
                    // 判断条件：1.小于路径宽度（点云在车辆检测范围内）2.代检测点到车辆距离dis小于车到目标点距离（离目标太远无意义） 3.启动障碍物检测的点
                    if (dis < pathRange / pathScale && (dis <= (relativeGoalDis + goalClearRange) / pathScale || !pathCropByGoal) && checkObstacle) 
                    {
                        // 这里也通过啦

                        // 尝试旋转36个方向检查障碍点是否会阻挡该方向下的候选路径
                        for (int rotDir = 0; rotDir < 36; rotDir++) 
                        {
                            // 每个旋转方向 当前位置转路径点的角度 
                            float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
                            // 当前候选路径方向（rotDir）与目标方向（joyDir）之间的角度差
                            float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
                        if (angDiff > 180.0) 
                        {
                            angDiff = 360.0 - angDiff;
                        }
                        if ((angDiff > dirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
                            ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 && dirToVehicle)) 
                        {
                            // (angDiff > dirThre && !dirToVehicle) 不允许倒车 候选路径与目标方向的偏差大于阈值 dirThre
                            // 
                            continue;
                        }

                        // x y 是 livox frame 坐标系下面的
                        // 将x2 y2旋转rotAng
                        // x2 在候选路径方向上（路径前方）的投影距离
                        float x2 = cos(rotAng) * x + sin(rotAng) * y;
                        float y2 = -sin(rotAng) * x + cos(rotAng) * y;
                        float scaleY = x2 / gridVoxelOffsetX + searchRadius / gridVoxelOffsetY 
                        * (gridVoxelOffsetX - x2) / gridVoxelOffsetX;
                        
                        // 计算该 plannerCloudCropSize（i） 点云对应的体素网格的索引（体素网格下包含searchRadius范围内的path_id）
                        int indX = int((gridVoxelOffsetX + gridVoxelSize / 2 - x2) / gridVoxelSize);
                        int indY = int((gridVoxelOffsetY + gridVoxelSize / 2 - y2 / scaleY) / gridVoxelSize);

                        if (indX >= 0 && indX < gridVoxelNumX && indY >= 0 && indY < gridVoxelNumY) 
                        {
                            // 二维索引映射到一维
                            int ind = gridVoxelNumY * indX + indY;
                            int blockedPathByVoxelNum = correspondences[ind].size();
                            // RCLCPP_INFO(nh->get_logger(),"indX >= 0 && indX < gridVoxelNumX && indY >= 0 && indY < gridVoxelNumY");
                            // 这里也通过了
                            for (int j = 0; j < blockedPathByVoxelNum; j++) 
                            {
                                // 如果是高障碍 增加clearPathList计数
                                // RCLCPP_INFO(nh->get_logger(),"for (int j = 0; j < blockedPathByVoxelNum; j++)");
                                // 这里正常通过了 
                                if (h > obstacleHeightThre || !useTerrainAnalysis) 
                                {
                                    // RCLCPP_INFO(nh->get_logger(),"h > obstacleHeightThre || !useTerrainAnalysis");
                                    // 这里也正常通过了
                                    clearPathList[pathNum * rotDir + correspondences[ind][j]]++;
                                    pcl::PointXYZI p;
                                    p.x = plannerCloudCrop->points[i].x;
                                    p.y = plannerCloudCrop->points[i].y;
                                    p.z = plannerCloudCrop->points[i].z;
                                    p.intensity = 200;  // 障碍点高亮
                                    obstacleVisCloud->push_back(p);
                                } else {
                                    // 其余情况不算阻挡但是增加路径的惩罚
                                    if (pathPenaltyList[pathNum * rotDir + correspondences[ind][j]] < h && h > groundHeightThre) 
                                    {
                                        pathPenaltyList[pathNum * rotDir + correspondences[ind][j]] = h;
                                    }
                                }
                            }
                        }
                        }
                    }

                    // 检查障碍物是否会阻止车辆绕中心旋转
                    if (dis < diameter / pathScale && (fabs(x) > vehicleLength / pathScale / 2.0 || fabs(y) > vehicleWidth / pathScale / 2.0) && 
                        (h > obstacleHeightThre || !useTerrainAnalysis) && checkRotObstacle) 
                    {
                        float angObs = atan2(y, x) * 180.0 / PI;
                        
                        RCLCPP_INFO(nh->get_logger(),"angObs %d",angObs);
                        // 这里的判断语句从来没有进入过
                        if (angObs > 0) {
                        if (minObsAngCCW > angObs - angOffset) minObsAngCCW = angObs - angOffset;
                        if (minObsAngCW < angObs + angOffset - 180.0) minObsAngCW = angObs + angOffset - 180.0;
                        } else {
                        if (minObsAngCW < angObs + angOffset) minObsAngCW = angObs + angOffset;
                        if (minObsAngCCW > 180.0 + angObs - angOffset) minObsAngCCW = 180.0 + angObs - angOffset;
                        }
                    }
                    // RCLCPP_INFO(nh->get_logger(),"pathScale >= minPathScale && pathRange >= minPathRange");
                }

                // TAG发布障碍物点云
                sensor_msgs::msg::PointCloud2 visMsg;
                pcl::toROSMsg(*obstacleVisCloud, visMsg);
                // 有点理解为什么要求全都转到camera init坐标系下面了
                visMsg.header.frame_id = "livox_frame";
                visMsg.header.stamp = nh->now();
                pubObstacleCloud->publish(visMsg);

                if (minObsAngCW > 0)
                {
                    minObsAngCW = 0;
                } 
                
                if (minObsAngCCW < 0) 
                {
                    minObsAngCCW = 0;
                }
                
                
                visualization_msgs::msg::MarkerArray markerArray;
                for (int i = 0; i < 36 * pathNum; i++) {
                    int rotDir = int(i / pathNum);
                    
                    float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
                    if (angDiff > 180.0) {
                        angDiff = 360.0 - angDiff;
                    }
                    if ((angDiff > dirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
                        ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 && dirToVehicle)) {
                        continue;
                    }
                    // RCLCPP_INFO(nh->get_logger(),"angdiff :%f",angDiff);

                    // 对候选路径进行打分
                    // 第i条路检测到的障碍物的数量
                    // if (clearPathList[i] < pointPerPathThre) 
                    if (true) 
                    {
                        // pathPenaltyList是路径的地形代价 障碍惩罚或者高度惩罚
                        float penaltyScore = 1.0 - pathPenaltyList[i] / costHeightThre;
                        // 保底分
                        if (penaltyScore < costScore)
                        {
                            penaltyScore = costScore;
                        }
                        
                        // 当前目标方向与候选路径的实际方向之间的角度差
                        float dirDiff = fabs(joyDir - endDirPathList[i % pathNum] - (10.0 * rotDir - 180.0));
                        // 限制在0~180
                        if (dirDiff > 360.0) {
                            dirDiff -= 360.0;
                        }
                        if (dirDiff > 180.0) {
                            dirDiff = 360.0 - dirDiff;
                        }

                        if (rotDir % 5 == 0 && i % pathNum == 0) 
                        {
                            visualization_msgs::msg::Marker m;
                            m.header.frame_id = "livox_frame";
                            m.header.stamp = nh->now();
                            m.ns = "dir_diff";
                            m.id = rotDir;
                            m.type = visualization_msgs::msg::Marker::ARROW;
                            m.action = visualization_msgs::msg::Marker::ADD;
                            m.scale.x = 0.5;
                            m.scale.y = 0.05;
                            m.scale.z = 0.05;
                            m.color.r = 0.0;
                            m.color.g = 0.8;
                            m.color.b = 1.0;
                            m.color.a = 0.8;

                            geometry_msgs::msg::Point p0, p1;
                            p0.x = 0.0;
                            p0.y = 0.0;
                            p1.x = 2.0 * cos((10.0 * rotDir - 180.0) * M_PI / 180.0);
                            p1.y = 2.0 * sin((10.0 * rotDir - 180.0) * M_PI / 180.0);
                            m.points.push_back(p0);
                            m.points.push_back(p1);

                            visualization_msgs::msg::Marker text;
                            text.header = m.header;
                            text.ns = "dir_diff_text";
                            text.id = 100 + rotDir;
                            text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                            text.action = visualization_msgs::msg::Marker::ADD;
                            text.pose.position.x = p1.x;
                            text.pose.position.y = p1.y;
                            text.pose.position.z = 0.5;
                            text.scale.z = 0.3;
                            text.color.r = 1.0;
                            text.color.g = 1.0;
                            text.color.b = 1.0;
                            text.color.a = 1.0;
                            text.text = std::to_string((int)dirDiff) + "°";

                            markerArray.markers.push_back(m);
                            markerArray.markers.push_back(text);
                        }




                        float rotDirW;
                        if (rotDir < 18) 
                        {
                            rotDirW = fabs(fabs(rotDir - 9) + 1);
                        }
                        else {
                            rotDirW = fabs(fabs(rotDir - 27) + 1);
                        }
                        // 1 - 四次根号(权重系数 * 路径终点与目标点之间的角度差值) 路径的方向与当前车辆朝向（车辆y轴=yaw角）的角度差  penaltyScore 路径上的障碍给出的惩罚得分 angDiff
                        float score = (1 - sqrt(sqrt(dirWeight * dirDiff))) * rotDirW * rotDirW * rotDirW * rotDirW * penaltyScore;
                        if (score > 0) {
                            //将所有path_id下的分数加到对应path_id下的groupid中，用于选择对应rotdir的groupid（确定第一级路径）
                            //定位到特定路径组groupid，groupNum * rotDir是该方向上的groupid起始序号，pathList[i % pathNum]]0-343该条路径对应的groupid（0-7）中的一个
                            clearPathPerGroupScore[groupNum * rotDir + pathList[i % pathNum]] += score;
                        }
                        // for (int rotDir = 0; rotDir < 36; rotDir++) {
                        //     std::stringstream ss;
                        //     ss << "rotDir " << rotDir << " scores: ";
                        //     for (int g = 0; g < groupNum; g++) {
                        //         ss << clearPathPerGroupScore[rotDir * groupNum + g] << " ";
                        //     }
                        //     RCLCPP_INFO(nh->get_logger(), "%s", ss.str().c_str());
                        // }

                        if (!markerArray.markers.empty()) 
                        {
                            pubMarker->publish(markerArray);
                        }
                    }
                }

 

                
                float maxScore = 0;
                int selectedGroupID = -1;
                // 遍历所有方向路径组 选出得分最高的可行路径组
                // groupNum 第一次分裂出来的组数


                for (int i = 0; i < 36 * groupNum; i++) {
                    //遍历可选路径（36*7）即（rotdir朝向*第一级group_id）
                    // rotDir路径方向
                    int rotDir = int(i / groupNum);
                    float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
                    float rotDeg = 10.0 * rotDir;
                    if (rotDeg > 180.0) rotDeg -= 360.0;
                    if (maxScore < clearPathPerGroupScore[i] && ((rotAng * 180.0 / PI > minObsAngCW && rotAng * 180.0 / PI < minObsAngCCW) || 
                        (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && twoWayDrive) || !checkRotObstacle)) {
                        // 选择 7* 36中分数最大的
                        maxScore = clearPathPerGroupScore[i];
                        // 记录所选路径的ID
                        selectedGroupID = i;
                    }
                }

                // =============================================================================
                // TAG 添加历史的惯性
                for (int i = 0; i < 36 * groupNum; i++) {
                    if (clearPathPerGroupScore[i] > maxScore)  {
                        // 如果得分明显更高，直接选
                        if (clearPathPerGroupScore[i] > scoreThreshold) {
                            maxScore = clearPathPerGroupScore[i];
                            selectedGroupID = i;
                        } 
                        // 如果得分接近，优先选上一帧的路径
                        else if (lastSelectedGroupID != -1 && i == lastSelectedGroupID) {
                            maxScore = clearPathPerGroupScore[i];
                            selectedGroupID = i;
                        }
                    }
                }
                // TAG 添加历史的惯性

                // 更新历史选择
                if (selectedGroupID >= 0) {
                    lastSelectedGroupID = selectedGroupID;
                }

                

                // 构造输出路径
                if (selectedGroupID >= 0) 
                {
                    int rotDir = int(selectedGroupID / groupNum);
                    float rotAng = (10.0 * rotDir - 180.0) * PI / 180;

                    selectedGroupID = selectedGroupID % groupNum;
                    int selectedPathLength = startPaths[selectedGroupID]->points.size();
                    path.poses.resize(selectedPathLength);
                    for (int i = 0; i < selectedPathLength; i++) {
                        float x = startPaths[selectedGroupID]->points[i].x;
                        float y = startPaths[selectedGroupID]->points[i].y;
                        float z = startPaths[selectedGroupID]->points[i].z;
                        float dis = sqrt(x * x + y * y);

                        // TODO：这里的pose计算也许需要改一下
                        if (dis <= pathRange / pathScale && dis <= relativeGoalDis / pathScale) {
                            path.poses[i].pose.position.x = pathScale * (cos(rotAng) * x - sin(rotAng) * y);
                            path.poses[i].pose.position.y = pathScale * (sin(rotAng) * x + cos(rotAng) * y);
                            path.poses[i].pose.position.z = pathScale * z;
                        } else {
                            path.poses.resize(i);
                            break;
                        }
                    }

                    path.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
                    // HACK 这个vehicle是什么
                    // path.header.frame_id = "body";
                    path.header.frame_id = "base_link";
                    pubPath->publish(path);

                    #if PLOTPATHSET == 1
                    freePaths->clear();
                    // 候选路径可视化
                    for (int i = 0; i < 36 * pathNum; i++) {
                        // rotDir 当前方向的旋转角度
                        int rotDir = int(i / pathNum);
                        float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
                        float rotDeg = 10.0 * rotDir;
                        if (rotDeg > 180.0) rotDeg -= 360.0;
                        // 候选路径过滤
                        float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
                        if (angDiff > 180.0) {
                            angDiff = 360.0 - angDiff;
                        }
                        if ((angDiff > dirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
                            ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 && dirToVehicle) || 
                            !((rotAng * 180.0 / PI > minObsAngCW && rotAng * 180.0 / PI < minObsAngCCW) || 
                            (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && twoWayDrive) || !checkRotObstacle)) {
                        continue;
                        }

                        if (clearPathList[i] < pointPerPathThre) {
                            int freePathLength = paths[i % pathNum]->points.size();
                            for (int j = 0; j < freePathLength; j++) {
                                point = paths[i % pathNum]->points[j];

                                float x = point.x;
                                float y = point.y;
                                float z = point.z;

                                float dis = sqrt(x * x + y * y);
                                if (dis <= pathRange / pathScale && (dis <= (relativeGoalDis + goalClearRange) / pathScale || !pathCropByGoal)) {
                                    point.x = pathScale * (cos(rotAng) * x - sin(rotAng) * y);
                                    point.y = pathScale * (sin(rotAng) * x + cos(rotAng) * y);
                                    point.z = pathScale * z;
                                    point.intensity = 1.0;

                                    freePaths->push_back(point);
                                }
                            }
                        }
                    }

                    sensor_msgs::msg::PointCloud2 freePaths2;
                    pcl::toROSMsg(*freePaths, freePaths2);
                    freePaths2.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
                    // NOTE这里的vehicle应该是车辆的中心
                    freePaths2.header.frame_id = "base_link";
                    pubFreePaths->publish(freePaths2);
                    #endif
                }

                if (selectedGroupID < 0) {
                    if (pathScale >= minPathScale + pathScaleStep) {
                        pathScale -= pathScaleStep;
                        pathRange = adjacentRange * pathScale / defPathScale;
                    } else {
                        pathRange -= pathRangeStep;
                    }
                    } else {
                    pathFound = true;
                    break;
                }
            }
            pathScale = defPathScale;

            // rclcpp::info(rclcpp::get_logger("example_logger"), "Path found: %s", pathFound ? "true" : "false");4
            // RCLCPP_INFO(nh->get_logger(),"Path found: %s", pathFound ? "true" : "false");

            if (!pathFound) {
                path.poses.resize(1);
                path.poses[0].pose.position.x = 0;
                path.poses[0].pose.position.y = 0;
                path.poses[0].pose.position.z = 0;

                path.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
                path.header.frame_id = "base_link";
                pubPath->publish(path);

                #if PLOTPATHSET == 1
                freePaths->clear();
                sensor_msgs::msg::PointCloud2 freePaths2;
                pcl::toROSMsg(*freePaths, freePaths2);
                freePaths2.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
                freePaths2.header.frame_id = "base_link";
                pubFreePaths->publish(freePaths2);
                #endif
            }
            

        }
        
        status = rclcpp::ok();
        rate.sleep();
    }
    
    // rclcpp::shutdown();
    return 0;
}

