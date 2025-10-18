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
#include <sensor_msgs/msg/imu.hpp>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/transform_listener.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <fstream>
#include <errno.h>
#include <string.h>
#include <pcl/io/ply_io.h>   

#include <chrono>
using namespace std::chrono_literals;

const double PI = 3.1415926;

#define PLOTPATHSET 1

class LocalPlanner : public rclcpp::Node
{
public:
    LocalPlanner() : Node("localPlanner"), 
                    tfBuffer(this->get_clock()),
                    tfListener(tfBuffer)
    {
        initializeParameters();
        setupPublishersAndSubscribers();
        initializeFilters();
        initializePointClouds();
        loadPathFiles();

        // 将原来的 while 循环改成定时器
        timer = this->create_wall_timer(
            std::chrono::milliseconds(50),  // 20hz
            std::bind(&LocalPlanner::processData,this)
        );
        
        RCLCPP_INFO(this->get_logger(), "LocalPlanner initialized successfully.");
    }

private:

    rclcpp::TimerBase::SharedPtr timer;

    // Parameters
    std::string pathFolder;
    double vehicleLength;
    double vehicleWidth;
    double sensorOffsetX;
    double sensorOffsetY;
    bool twoWayDrive;
    double laserVoxelSize;
    double terrainVoxelSize;
    bool useTerrainAnalysis;
    bool checkObstacle;
    bool checkRotObstacle;
    double adjacentRange;
    double obstacleHeightThre;
    double groundHeightThre;
    double costHeightThre;
    double costScore;
    bool useCost;
    const int laserCloudStackNum = 1;
    int laserCloudCount;
    int pointPerPathThre;
    double minRelZ;
    double maxRelZ;
    double maxSpeed;
    double dirWeight;
    double dirThre;
    bool dirToVehicle;
    double pathScale;
    double minPathScale;
    double pathScaleStep;
    bool pathScaleBySpeed;
    double minPathRange;
    double pathRangeStep;
    bool pathRangeBySpeed;
    bool pathCropByGoal;
    bool autonomyMode;
    double autonomySpeed;
    double joyToSpeedDelay;
    double joyToCheckObstacleDelay;
    double goalClearRange;
    double goalX;
    double goalY;

    // State variables
    float joySpeed;
    float joySpeedRaw;
    float joyDir;
    bool newlaserCloud = false;
    bool newTerrainCloud = false;
    double odomTime = 0;
    double joyTime = 0;
    float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
    float vehicleX = 0, vehicleY = 0, vehicleZ = 0;

    // Constants
    const int pathNum = 343;
    const int groupNum = 7;
    float gridVoxelSize = 0.02;
    float searchRadius = 0.45;
    float gridVoxelOffsetX = 3.2;
    float gridVoxelOffsetY = 4.5;
    const int gridVoxelNumX = 161;
    const int gridVoxelNumY = 451;
    const int gridVoxelNum = gridVoxelNumX * gridVoxelNumY;

    // Arrays and vectors
    int pathList[343] = {0};
    float endDirPathList[343] = {0};
    int clearPathList[36 * 343] = {0};
    float pathPenaltyList[36 * 343] = {0};
    float clearPathPerGroupScore[36 * 7] = {0};
    std::vector<int> correspondences[72611]; // 161*451 = 72611

    // Point clouds
    pcl::VoxelGrid<pcl::PointXYZI> laserDwzFilter, terrainDwzFilter;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDWZ;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> laserCloudStack;
    pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudCrop;
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz;
    pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloudCrop;
    pcl::PointCloud<pcl::PointXYZI>::Ptr boundaryCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloudBody;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> startPaths;
#if PLOTPATHSET == 1
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> paths;
    pcl::PointCloud<pcl::PointXYZI>::Ptr freePaths;
#endif

    // Selection variables
    static int lastSelectedGroupID;
    float maxScore = 0;
    int selectedGroupID = -1;
    float scoreThreshold = maxScore * 0.9;

    // ROS components
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subTerrainCloud;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subGoal;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr subBoundary;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud2;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubObstacleCloud;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubMarker;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;
#if PLOTPATHSET == 1
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubFreePaths;
#endif

    // TF2 components - must be initialized in member initializer list
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    void initializeParameters()
    {
        // Declare parameters
        this->declare_parameter<std::string>("pathFolder", "");
        this->declare_parameter<double>("vehicleLength", 0.6);
        this->declare_parameter<double>("vehicleWidth", 0.6);
        this->declare_parameter<double>("sensorOffsetX", 0.0);
        this->declare_parameter<double>("sensorOffsetY", 0.0);
        this->declare_parameter<bool>("twoWayDrive", true);
        this->declare_parameter<double>("laserVoxelSize", 0.05);
        this->declare_parameter<double>("terrainVoxelSize", 0.2);
        this->declare_parameter<bool>("useTerrainAnalysis", false);
        this->declare_parameter<bool>("checkObstacle", true);
        this->declare_parameter<bool>("checkRotObstacle", false);
        this->declare_parameter<double>("adjacentRange", 3.5);
        this->declare_parameter<double>("obstacleHeightThre", 0.2);
        this->declare_parameter<double>("groundHeightThre", 0.1);
        this->declare_parameter<double>("costHeightThre", 0.1);
        this->declare_parameter<double>("costScore", 0.02);
        this->declare_parameter<bool>("useCost", false);
        this->declare_parameter<int>("pointPerPathThre", 2);
        this->declare_parameter<double>("minRelZ", -0.5);
        this->declare_parameter<double>("maxRelZ", 0.25);
        this->declare_parameter<double>("maxSpeed", 1.0);
        this->declare_parameter<double>("dirWeight", 0.02);
        this->declare_parameter<double>("dirThre", 90.0);
        this->declare_parameter<bool>("dirToVehicle", false);
        this->declare_parameter<double>("pathScale", 1.0);
        this->declare_parameter<double>("minPathScale", 0.75);
        this->declare_parameter<double>("pathScaleStep", 0.25);
        this->declare_parameter<bool>("pathScaleBySpeed", true);
        this->declare_parameter<double>("minPathRange", 1.0);
        this->declare_parameter<double>("pathRangeStep", 0.5);
        this->declare_parameter<bool>("pathRangeBySpeed", true);
        this->declare_parameter<bool>("pathCropByGoal", true);
        this->declare_parameter<bool>("autonomyMode", false);
        this->declare_parameter<double>("autonomySpeed", 1.0);
        this->declare_parameter<double>("joyToSpeedDelay", 2.0);
        this->declare_parameter<double>("joyToCheckObstacleDelay", 5.0);
        this->declare_parameter<double>("goalClearRange", 0.5);
        this->declare_parameter<double>("goalX", 0.0);
        this->declare_parameter<double>("goalY", 0.0);

        // Get parameters
        this->get_parameter("pathFolder", pathFolder);
        this->get_parameter("vehicleLength", vehicleLength);
        this->get_parameter("vehicleWidth", vehicleWidth);
        this->get_parameter("sensorOffsetX", sensorOffsetX);
        this->get_parameter("sensorOffsetY", sensorOffsetY);
        this->get_parameter("twoWayDrive", twoWayDrive);
        this->get_parameter("laserVoxelSize", laserVoxelSize);
        this->get_parameter("terrainVoxelSize", terrainVoxelSize);
        this->get_parameter("useTerrainAnalysis", useTerrainAnalysis);
        this->get_parameter("checkObstacle", checkObstacle);
        this->get_parameter("checkRotObstacle", checkRotObstacle);
        this->get_parameter("adjacentRange", adjacentRange);
        this->get_parameter("obstacleHeightThre", obstacleHeightThre);
        this->get_parameter("groundHeightThre", groundHeightThre);
        this->get_parameter("costHeightThre", costHeightThre);
        this->get_parameter("costScore", costScore);
        this->get_parameter("useCost", useCost);
        this->get_parameter("pointPerPathThre", pointPerPathThre);
        this->get_parameter("minRelZ", minRelZ);
        this->get_parameter("maxRelZ", maxRelZ);
        this->get_parameter("maxSpeed", maxSpeed);
        this->get_parameter("dirWeight", dirWeight);
        this->get_parameter("dirThre", dirThre);
        this->get_parameter("dirToVehicle", dirToVehicle);
        this->get_parameter("pathScale", pathScale);
        this->get_parameter("minPathScale", minPathScale);
        this->get_parameter("pathScaleStep", pathScaleStep);
        this->get_parameter("pathScaleBySpeed", pathScaleBySpeed);
        this->get_parameter("minPathRange", minPathRange);
        this->get_parameter("pathRangeStep", pathRangeStep);
        this->get_parameter("pathRangeBySpeed", pathRangeBySpeed);
        this->get_parameter("pathCropByGoal", pathCropByGoal);
        this->get_parameter("autonomyMode", autonomyMode);
        this->get_parameter("autonomySpeed", autonomySpeed);
        this->get_parameter("joyToSpeedDelay", joyToSpeedDelay);
        this->get_parameter("joyToCheckObstacleDelay", joyToCheckObstacleDelay);
        this->get_parameter("goalClearRange", goalClearRange);
        this->get_parameter("goalX", goalX);
        this->get_parameter("goalY", goalY);

        // Set default path folder
        // TODO 这里需要修
        if (pathFolder.empty()) {
            pathFolder = "/home/z/rm_simulation/src/local_planner/paths";
        }
    }

    void setupPublishersAndSubscribers()
    {
        // Subscribers
        subOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 5, std::bind(&LocalPlanner::odometryHandle, this, std::placeholders::_1));
        
        subLaserCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/registered_scan", 5, std::bind(&LocalPlanner::laserCloudHandler, this, std::placeholders::_1));
        
        subTerrainCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/terrain_map", 5, std::bind(&LocalPlanner::terrainCloudHandler, this, std::placeholders::_1));
        
        subGoal = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/way_point", 5, std::bind(&LocalPlanner::goalHandler, this, std::placeholders::_1));
        
        subBoundary = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
            "/navigation_boundary", 5, std::bind(&LocalPlanner::boundaryHandle, this, std::placeholders::_1));

        // Publishers
        pubLaserCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/plannerCloud", 5);
        pubLaserCloud2 = this->create_publisher<sensor_msgs::msg::PointCloud2>("/plannerCloudCropPlanner", 5);
        pubObstacleCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/visObstacleCloud", 5);
        pubMarker = this->create_publisher<visualization_msgs::msg::MarkerArray>("/path_debug", 10);
        pubPath = this->create_publisher<nav_msgs::msg::Path>("/local_path", 5);
        
#if PLOTPATHSET == 1
        pubFreePaths = this->create_publisher<sensor_msgs::msg::PointCloud2>("/free_paths", 2);
#endif
    }

    void initializeFilters()
    {
        laserDwzFilter.setLeafSize(laserVoxelSize, laserVoxelSize, laserVoxelSize);
        terrainDwzFilter.setLeafSize(terrainVoxelSize, terrainVoxelSize, terrainVoxelSize);
    }

    void initializePointClouds()
    {
        // Initialize point clouds
        laserCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        laserCloudCrop.reset(new pcl::PointCloud<pcl::PointXYZI>());
        laserCloudDWZ.reset(new pcl::PointCloud<pcl::PointXYZI>());
        plannerCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        terrainCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        terrainCloudCrop.reset(new pcl::PointCloud<pcl::PointXYZI>());
        terrainCloudDwz.reset(new pcl::PointCloud<pcl::PointXYZI>());
        plannerCloudCrop.reset(new pcl::PointCloud<pcl::PointXYZI>());
        boundaryCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        plannerCloudBody.reset(new pcl::PointCloud<pcl::PointXYZI>());

        // Initialize laser cloud stack
        for (int i = 0; i < laserCloudStackNum; i++) {
            laserCloudStack.push_back(pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>()));
        }

        // Initialize start paths
        for (int i = 0; i < groupNum; i++) {
            startPaths.push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>()));
        }

#if PLOTPATHSET == 1
        // Initialize paths
        for (int i = 0; i < pathNum; i++) {
            paths.push_back(pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>()));
        }
        freePaths.reset(new pcl::PointCloud<pcl::PointXYZI>());
#endif

        // Initialize correspondences
        for (int i = 0; i < gridVoxelNum; i++) {
            correspondences[i].resize(0);
        }
    }

    // Callback functions
    void odometryHandle(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
    {
        // 这里的vehicle是odom坐标系下面的 需要转换到map坐标系下面
        odomTime = rclcpp::Time(odom->header.stamp).seconds();
        double roll, pitch, yaw;
        geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
        tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

        vehicleRoll = roll;
        vehiclePitch = pitch;
        vehicleYaw = yaw;
        vehicleX = odom->pose.pose.position.x;
        vehicleY = odom->pose.pose.position.y;
        vehicleZ = odom->pose.pose.position.z;

        // RCLCPP_DEBUG(this->get_logger(), "vehicleX: %f, vehicleY: %f", vehicleX, vehicleY);

        // 转换到map坐标系下面
        try
        {
            geometry_msgs::msg::TransformStamped map_to_odom = tfBuffer.lookupTransform("map","odom",odom->header.stamp,100ms);

            // 构造odom下面的pose
            geometry_msgs::msg::PoseStamped odom_pose,map_pose;
            odom_pose.header = odom->header;
            odom_pose.pose = odom->pose.pose;

            // 执行坐标变换
            tf2::doTransform(odom_pose,map_pose,map_to_odom);

            vehicleX = map_pose.pose.position.x;
            vehicleY = map_pose.pose.position.y;
            vehicleZ = map_pose.pose.position.z;

            
            
          }
          catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "TF transform (map->odom) failed: %s", ex.what());
          }
          
        //   RCLCPP_INFO(this->get_logger(),
        //            "Transformed to map frame: X=%.3f, Y=%.3f, Z=%.3f",
        //            vehicleX, vehicleY, vehicleZ);

    }

    void laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud2)
    {
        if (!useTerrainAnalysis) {
            laserCloud->clear();
            pcl::fromROSMsg(*laserCloud2, *laserCloud);

            pcl::PointXYZI point;
            laserCloudCrop->clear();
            int laserCloudSize = laserCloud->points.size();
            for (int i = 0; i < laserCloudSize; i++) {
                point = laserCloud->points[i];
                float pointX = point.x;
                float pointY = point.y;
                float pointZ = point.z;
                float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + 
                                (pointY - vehicleY) * (pointY - vehicleY));
                if (dis < adjacentRange) {
                    point.x = pointX;
                    point.y = pointY;
                    point.z = pointZ;
                    laserCloudCrop->push_back(point);
                }
            }

            laserCloudDWZ->clear();
            laserDwzFilter.setInputCloud(laserCloudCrop);
            laserDwzFilter.filter(*laserCloudDWZ);

            newlaserCloud = true;
        }
    }

    void terrainCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrainCloud2)
    {
        if (useTerrainAnalysis) {
            terrainCloud->clear();
            pcl::fromROSMsg(*terrainCloud2, *terrainCloud);

            pcl::PointXYZI point;
            terrainCloudCrop->clear();
            int terrainCloudSize = terrainCloud->points.size();
            for (int i = 0; i < terrainCloudSize; i++) {
                point = terrainCloud->points[i];
                float pointX = point.x;
                float pointY = point.y;
                float pointZ = point.z;
                float dis = std::sqrt((pointX - vehicleX) * (pointX - vehicleX) +
                                     (pointY - vehicleY) * (pointY - vehicleY));
                if (dis < adjacentRange && (point.intensity > obstacleHeightThre || useCost)) {
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

    void boundaryHandle(const geometry_msgs::msg::PolygonStamped::ConstSharedPtr /*boundary*/)
    {
        // Implementation for boundary handling
    }

    void goalHandler(const geometry_msgs::msg::PointStamped::ConstSharedPtr goal)
    {
        std::string frame_id = goal->header.frame_id;
        goalX = goal->point.x;
        goalY = goal->point.y;
        RCLCPP_DEBUG(this->get_logger(), "Goal updated: (%f, %f)", goalX, goalY);
    }

    // File reading functions (safe versions)
    static int readPlyHeader_safe(FILE *filePtr, bool &ok)
    {
        ok = false;
        if (!filePtr) return 0;
        char str[128];
        int val;
        int pointNum = 0;
        std::string strCur, strLast;
        
        while (strCur != "end_header") {
            val = fscanf(filePtr, "%127s", str);
            if (val != 1) {
                return 0;
            }
            strLast = strCur;
            strCur = std::string(str);
            if (strCur == "vertex" && strLast == "element") {
                val = fscanf(filePtr, "%d", &pointNum);
                if (val != 1) {
                    return 0;
                }
            }
        }
        ok = true;
        return pointNum;
    }

    bool readStartPaths_safe()
    {
        std::string fileName = pathFolder + "/startPaths.ply";
        FILE *filePtr = fopen(fileName.c_str(), "r");
        if (filePtr == NULL) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open %s: %s", fileName.c_str(), strerror(errno));
            return false;
        }

        bool header_ok = false;
        int pointNum = readPlyHeader_safe(filePtr, header_ok);
        if (!header_ok || pointNum <= 0) {
            fclose(filePtr);
            RCLCPP_ERROR(this->get_logger(), "Invalid PLY header or zero points in %s", fileName.c_str());
            return false;
        }

        pcl::PointXYZ point;
        int val1, val2, val3, val4, groupID;
        for (int i = 0; i < pointNum; ++i) {
            val1 = fscanf(filePtr, "%f", &point.x);
            val2 = fscanf(filePtr, "%f", &point.y);
            val3 = fscanf(filePtr, "%f", &point.z);
            val4 = fscanf(filePtr, "%d", &groupID);

            if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1) {
                fclose(filePtr);
                RCLCPP_ERROR(this->get_logger(), "Error reading point %d in %s", i, fileName.c_str());
                return false;
            }

            if (groupID >= 0 && groupID < groupNum) {
                startPaths[groupID]->push_back(point);
            }
        }

        fclose(filePtr);
        RCLCPP_INFO(this->get_logger(), "Loaded %d start points from %s", pointNum, fileName.c_str());
        return true;
    }

#if PLOTPATHSET == 1
    bool readPaths_safe()
    {
        std::string fileName = pathFolder + "/paths.ply";
        FILE *filePtr = fopen(fileName.c_str(), "r");
        if (filePtr == NULL) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open %s: %s", fileName.c_str(), strerror(errno));
            return false;
        }

        bool header_ok = false;
        int pointNum = readPlyHeader_safe(filePtr, header_ok);
        if (!header_ok || pointNum <= 0) {
            fclose(filePtr);
            RCLCPP_ERROR(this->get_logger(), "Invalid PLY header or zero points in %s", fileName.c_str());
            return false;
        }

        pcl::PointXYZI point;
        int pointSkipNum = 30;
        int pointSkipCount = 0;
        int val1, val2, val3, val4, val5, pathID;
        for (int i = 0; i < pointNum; ++i) {
            val1 = fscanf(filePtr, "%f", &point.x);
            val2 = fscanf(filePtr, "%f", &point.y);
            val3 = fscanf(filePtr, "%f", &point.z);
            val4 = fscanf(filePtr, "%d", &pathID);
            val5 = fscanf(filePtr, "%f", &point.intensity);

            if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
                fclose(filePtr);
                RCLCPP_ERROR(this->get_logger(), "Error reading points in %s at index %d", fileName.c_str(), i);
                return false;
            }

            if (pathID >= 0 && pathID < pathNum) {
                ++pointSkipCount;
                if (pointSkipCount > pointSkipNum) {
                    paths[pathID]->push_back(point);
                    pointSkipCount = 0;
                }
            }
        }

        fclose(filePtr);
        RCLCPP_INFO(this->get_logger(), "Loaded %d path points from %s", pointNum, fileName.c_str());
        return true;
    }
#endif

    bool readPathList_safe()
    {
        std::string fileName = pathFolder + "/pathList.ply";
        FILE *filePtr = fopen(fileName.c_str(), "r");
        if (filePtr == NULL) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open %s: %s", fileName.c_str(), strerror(errno));
            return false;
        }

        bool header_ok = false;
        int headerCount = readPlyHeader_safe(filePtr, header_ok);
        if (!header_ok) {
            fclose(filePtr);
            RCLCPP_ERROR(this->get_logger(), "Invalid header in %s", fileName.c_str());
            return false;
        }

        if (pathNum != headerCount) {
            fclose(filePtr);
            RCLCPP_ERROR(this->get_logger(), "Path number mismatch in %s: expected %d, got %d", 
                        fileName.c_str(), pathNum, headerCount);
            return false;
        }

        int val1, val2, val3, val4, val5, pathID, groupID;
        float endX, endY, endZ;
        for (int i = 0; i < pathNum; ++i) {
            val1 = fscanf(filePtr, "%f", &endX);
            val2 = fscanf(filePtr, "%f", &endY);
            val3 = fscanf(filePtr, "%f", &endZ);
            val4 = fscanf(filePtr, "%d", &pathID);
            val5 = fscanf(filePtr, "%d", &groupID);

            if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
                fclose(filePtr);
                RCLCPP_ERROR(this->get_logger(), "Error reading path list element %d in %s", i, fileName.c_str());
                return false;
            }

            if (pathID >= 0 && pathID < pathNum && groupID >= 0 && groupID < groupNum) {
                pathList[pathID] = groupID;
                endDirPathList[pathID] = 2.0 * atan2(endY, endX) * 180.0 / PI;
            }
        }

        fclose(filePtr);
        RCLCPP_INFO(this->get_logger(), "Loaded path list from %s", fileName.c_str());
        return true;
    }

    bool readCorrespondences_safe()
    {
        std::string fileName = pathFolder + "/correspondences.txt";
        FILE *filePtr = fopen(fileName.c_str(), "r");
        if (filePtr == NULL) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open %s: %s", fileName.c_str(), strerror(errno));
            return false;
        }

        int val1, gridVoxelID, pathID;
        for (int i = 0; i < gridVoxelNum; ++i) {
            val1 = fscanf(filePtr, "%d", &gridVoxelID);
            if (val1 != 1) {
                fclose(filePtr);
                RCLCPP_ERROR(this->get_logger(), "Error reading gridVoxelID in %s at index %d", fileName.c_str(), i);
                return false;
            }

            while (true) {
                val1 = fscanf(filePtr, "%d", &pathID);
                if (val1 != 1) {
                    fclose(filePtr);
                    RCLCPP_ERROR(this->get_logger(), "Error reading pathID in %s", fileName.c_str());
                    return false;
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
        RCLCPP_INFO(this->get_logger(), "Loaded correspondences from %s", fileName.c_str());
        return true;
    }

    void loadPathFiles()
    {
        RCLCPP_INFO(this->get_logger(), "pathFolder = %s", pathFolder.c_str());

        if (!readStartPaths_safe()) {
            RCLCPP_ERROR(this->get_logger(), "readStartPaths_safe failed. Shutting down.");
            rclcpp::shutdown();
            return;
        }

#if PLOTPATHSET == 1
        if (!readPaths_safe()) {
            RCLCPP_ERROR(this->get_logger(), "readPaths_safe failed. Shutting down.");
            rclcpp::shutdown();
            return;
        }
#endif

        if (!readPathList_safe()) {
            RCLCPP_ERROR(this->get_logger(), "readPathList_safe failed. Shutting down.");
            rclcpp::shutdown();
            return;
        }

        if (!readCorrespondences_safe()) {
            RCLCPP_ERROR(this->get_logger(), "readCorrespondences_safe failed. Shutting down.");
            rclcpp::shutdown();
            return;
        }
    }


    // 在类的私有成员中添加
    void visualizeDirectionDifferences(float joyDir, float pathRange)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        
        // 清除之前的标记
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);
        
        // 为每个方向创建标记
        for (int rotDir = 0; rotDir < 36; rotDir++)
        {
            float directionAngle = 10.0 * rotDir - 180.0; // 方向角度
            float angDiff = fabs(joyDir - directionAngle); // 角度差
            
            // 规范化角度差到 [0, 180]
            if (angDiff > 180) {
                angDiff = 360 - angDiff;
            }
            
            // 创建文本标记
            visualization_msgs::msg::Marker text_marker;
            text_marker.header.frame_id = "base_link"; // 使用车辆坐标系
            text_marker.header.stamp = this->now();
            text_marker.ns = "direction_differences";
            text_marker.id = rotDir;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            
            // 计算标记位置（在车辆周围的圆上）
            float angle_rad = directionAngle * PI / 180.0;
            float radius = pathRange * 0.8; // 使用路径范围的80%作为显示半径
            text_marker.pose.position.x = radius * cos(angle_rad);
            text_marker.pose.position.y = radius * sin(angle_rad);
            text_marker.pose.position.z = 1.0; // 稍微抬高避免与地面重叠
            
            // 设置文本内容
            text_marker.text = std::to_string(static_cast<int>(angDiff)) + "°";
            
            // 根据角度差设置颜色（角度差越小，颜色越绿）
            text_marker.scale.z = 0.3; // 文本大小
            text_marker.color.a = 1.0; // 不透明度
            
            // 颜色映射：0°=绿色，180°=红色
            float color_ratio = angDiff / 180.0;
            text_marker.color.r = color_ratio;     // 红色分量
            text_marker.color.g = 1.0 - color_ratio; // 绿色分量
            text_marker.color.b = 0.0;             // 蓝色分量
            
            marker_array.markers.push_back(text_marker);
            
            // 可选：添加方向箭头
            visualization_msgs::msg::Marker arrow_marker;
            arrow_marker.header.frame_id = "base_link";
            arrow_marker.header.stamp = this->now();
            arrow_marker.ns = "direction_arrows";
            arrow_marker.id = rotDir;
            arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
            arrow_marker.action = visualization_msgs::msg::Marker::ADD;
            
            // 箭头起始位置
            arrow_marker.pose.position.x = 0;
            arrow_marker.pose.position.y = 0;
            arrow_marker.pose.position.z = 0.5;
            
            // 设置箭头方向
            tf2::Quaternion quat;
            quat.setRPY(0, 0, angle_rad);
            arrow_marker.pose.orientation = tf2::toMsg(quat);
            
            // 设置箭头大小和颜色
            arrow_marker.scale.x = radius * 0.8; // 箭头长度
            arrow_marker.scale.y = 0.1;          // 箭头宽度
            arrow_marker.scale.z = 0.1;          // 箭头高度
            
            arrow_marker.color.a = 0.5; // 半透明
            arrow_marker.color.r = color_ratio;
            arrow_marker.color.g = 1.0 - color_ratio;
            arrow_marker.color.b = 0.0;
            
            marker_array.markers.push_back(arrow_marker);
        }
        
        // 发布标记数组
        pubMarker->publish(marker_array);
    }

    void processData()
    {
        if (newlaserCloud || newTerrainCloud)
        {
            if (newlaserCloud)
            {
                /* code */
            }

            if (newTerrainCloud)
            {
                newTerrainCloud = false;

                plannerCloud->clear();
                *plannerCloud = *terrainCloudDwz;
            }
            
        }

        // 将 plannerCloud pub出来
        sensor_msgs::msg::PointCloud2 ros_pc2;
        pcl::toROSMsg(*plannerCloud,ros_pc2);
        ros_pc2.header.frame_id = "map";
        ros_pc2.header.stamp = this->now();
        pubLaserCloud->publish(ros_pc2);
        
        pcl::PointXYZI point;
        plannerCloudCrop->clear();

        // 
        int plannerCloudSize = plannerCloud->points.size();
        for (int i = 0; i < plannerCloudSize; i++)
        {
            // 现在point cloud在map坐标系下面
            // vehicle也在map坐标系下面
            float pointX1 = plannerCloud->points[i].x - vehicleX;
            float pointY1 = plannerCloud->points[i].y - vehicleY;
            float pointZ1 = plannerCloud->points[i].z - vehicleZ;

            point.x = pointX1;
            point.y = pointY1;
            point.z = pointZ1;
            point.intensity = plannerCloud->points[i].intensity;

            float dis = sqrt(point.x * point.x + point.y * point.y);
            // RCLCPP_INFO(this->get_logger(),"dis : %.2f",dis);
            if (dis < adjacentRange && ((point.z > minRelZ && point.z < maxRelZ) || useTerrainAnalysis)) {
                plannerCloudCrop->push_back(point);
            }
        }

        // pub出来看看
        sensor_msgs::msg::PointCloud2 plannerCloudMsg;
        pcl::toROSMsg(*plannerCloudCrop,plannerCloudMsg);
        plannerCloudMsg.header.stamp = this->get_clock()->now();
        // HACKplannerCloudCrop的坐标系是 base_link 或者 livox_frame ??
        plannerCloudMsg.header.frame_id = "base_link";
        pubLaserCloud2->publish(plannerCloudMsg);
        

        float pathRange = adjacentRange;
        // RCLCPP_INFO(this->get_logger(),"pathRange : %.2f",pathRange);
        if (pathRange < minPathRange) 
        {
            pathRange = minPathRange;
        }

        float relativeGoalDis = adjacentRange;
        
        if (autonomyMode)
        {
            // NOTE:要考虑车辆的相对位置 !!!!!!!
            float relativeGoalX = goalX - vehicleX;
            float relativeGoalY = goalY - vehicleY;

            // 计算绝对角度（相对于地图坐标系）
            float absoluteGoalDir = atan2(relativeGoalY, relativeGoalX) * 180 / PI;

            // 减去车辆航向角，得到相对于车辆前方的角度
            joyDir = absoluteGoalDir - vehicleYaw * 180 / PI;

            // 规范化到 [-180, 180] 范围
            while (joyDir > 180) joyDir -= 360;
            while (joyDir < -180) joyDir += 360;

            relativeGoalDis = sqrt(relativeGoalX * relativeGoalX + relativeGoalY * relativeGoalY);

            // RCLCPP_INFO(this->get_logger(), "绝对角度: %.2f°, 车辆航向: %.2f°, 相对角度: %.2f°, 距离: %.2f", 
            //     absoluteGoalDir, vehicleYaw * 180 / PI, joyDir, relativeGoalDis);
            if (!twoWayDrive) 
            {
                if (joyDir > 90.0) 
                {
                    joyDir = 90.0;
                }
                else if (joyDir < -90.0) 
                {
                    joyDir = -90.0;
                }
            }
        }

        // ===========================================================================
        //                              TAG开始设置路径
        bool pathFind = false;
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
        while (pathScale >= minPathScale && pathRange >= minPathRange) 
        {
            // 清空之前的评分器
            for (int i = 0; i < 36 * pathNum; i++) 
            {
                clearPathList[i] = 0;
                pathPenaltyList[i] = 0;
            }

            // 每个方向都有gropNum个路径组 每个路径组对应一组不同形状的轨迹
            for (int i = 0; i < 36 * groupNum; i++)
            {
                clearPathPerGroupScore[i] = 0;
            }

            float minObsAngCW = -180.0;
            float minObsAngCCW = 180.0;
            float diameter = sqrt(vehicleLength / 2.0 * vehicleLength / 2.0 + vehicleWidth / 2.0 * vehicleWidth / 2.0);
            float angOffset = atan2(vehicleWidth, vehicleLength) * 180.0 / PI;

            pcl::PointCloud<pcl::PointXYZI>::Ptr obstacleVisCloud(new pcl::PointCloud<pcl::PointXYZI>());
            // 障碍物点云
            int plannerCloudCropSize = plannerCloudCrop->points.size();
            for (int i = 0; i < plannerCloudCropSize; i++)
            {
                // pathScale 路径尺度（路径的大小或长度与某个参考值（如车辆尺寸或环境尺寸）的比例关系），在狭窄的空间中减小路径规模，或在开放的空间中增加路径规模以优化行进路线
                // plannerCloudCrop是map坐标系下面的
                float x = plannerCloudCrop->points[i].x / pathScale;
                float y = plannerCloudCrop->points[i].y / pathScale;
                float h = plannerCloudCrop->points[i].intensity;

                float dis = std::sqrt(x*x + y*y);

                if (dis < pathRange / pathScale && (dis <= (relativeGoalDis + goalClearRange) / pathScale || !pathCropByGoal) && checkObstacle) 
                {
                    // 尝试旋转36个方向检查障碍点是否会阻挡该方向下的候选路径
                    for (int rotDir = 0; rotDir < 36; rotDir++)
                    {
                        // 每个旋转方向 当前位置转路径点的角度 
                        float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
                        // 当前候选路径方向（rotDir）与目标方向（joyDir）之间的角度差
                        float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
                    }
                    
                }

            }

            // 可视化角度差
            visualizeDirectionDifferences(joyDir, pathRange);
            
        }
        
    }

};

// Static member initialization
int LocalPlanner::lastSelectedGroupID = -1;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}