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

const double PI = 3.1415926;

#define PLOTPATHSET 1

class LocalPlanner : public rclcpp::Node
{
public:
    LocalPlanner() : Node("localPlanner"), 
                    tf_buffer_(this->get_clock()),
                    tf_listener_(tf_buffer_)
    {
        initializeParameters();
        setupPublishersAndSubscribers();
        initializeFilters();
        initializePointClouds();
        loadPathFiles();
        
        RCLCPP_INFO(this->get_logger(), "LocalPlanner initialized successfully.");
    }

private:
    // Parameters
    std::string pathFolder_;
    double vehicleLength_;
    double vehicleWidth_;
    double sensorOffsetX_;
    double sensorOffsetY_;
    bool twoWayDrive_;
    double laserVoxelSize_;
    double terrainVoxelSize_;
    bool useTerrainAnalysis_;
    bool checkObstacle_;
    bool checkRotObstacle_;
    double adjacentRange_;
    double obstacleHeightThre_;
    double groundHeightThre_;
    double costHeightThre_;
    double costScore_;
    bool useCost_;
    const int laserCloudStackNum_ = 1;
    int laserCloudCount_;
    int pointPerPathThre_;
    double minRelZ_;
    double maxRelZ_;
    double maxSpeed_;
    double dirWeight_;
    double dirThre_;
    bool dirToVehicle_;
    double pathScale_;
    double minPathScale_;
    double pathScaleStep_;
    bool pathScaleBySpeed_;
    double minPathRange_;
    double pathRangeStep_;
    bool pathRangeBySpeed_;
    bool pathCropByGoal_;
    bool autonomyMode_;
    double autonomySpeed_;
    double joyToSpeedDelay_;
    double joyToCheckObstacleDelay_;
    double goalClearRange_;
    double goalX_;
    double goalY_;

    // State variables
    float joySpeed_;
    float joySpeedRaw_;
    float joyDir_;
    bool newlaserCloud_ = false;
    bool newTerrainCloud_ = false;
    double odomTime_ = 0;
    double joyTime_ = 0;
    float vehicleRoll_ = 0, vehiclePitch_ = 0, vehicleYaw_ = 0;
    float vehicleX_ = 0, vehicleY_ = 0, vehicleZ_ = 0;

    // Constants
    const int pathNum_ = 343;
    const int groupNum_ = 7;
    float gridVoxelSize_ = 0.02;
    float searchRadius_ = 0.45;
    float gridVoxelOffsetX_ = 3.2;
    float gridVoxelOffsetY_ = 4.5;
    const int gridVoxelNumX_ = 161;
    const int gridVoxelNumY_ = 451;
    const int gridVoxelNum_ = gridVoxelNumX_ * gridVoxelNumY_;

    // Arrays and vectors
    int pathList_[343] = {0};
    float endDirPathList_[343] = {0};
    int clearPathList_[36 * 343] = {0};
    float pathPenaltyList_[36 * 343] = {0};
    float clearPathPerGroupScore_[36 * 7] = {0};
    std::vector<int> correspondences_[72611]; // 161*451 = 72611

    // Point clouds
    pcl::VoxelGrid<pcl::PointXYZI> laserDwzFilter_, terrainDwzFilter_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDWZ_;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> laserCloudStack_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudCrop_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloudCrop_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr boundaryCloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloudBody_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> startPaths_;
#if PLOTPATHSET == 1
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> paths_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr freePaths_;
#endif

    // Selection variables
    static int lastSelectedGroupID_;
    float maxScore_ = 0;
    int selectedGroupID_ = -1;
    float scoreThreshold_ = maxScore_ * 0.9;

    // ROS components
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subTerrainCloud_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subGoal_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr subBoundary_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud2_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubObstacleCloud_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubMarker_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
#if PLOTPATHSET == 1
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubFreePaths_;
#endif

    // TF2 components - must be initialized in member initializer list
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

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
        this->get_parameter("pathFolder", pathFolder_);
        this->get_parameter("vehicleLength", vehicleLength_);
        this->get_parameter("vehicleWidth", vehicleWidth_);
        this->get_parameter("sensorOffsetX", sensorOffsetX_);
        this->get_parameter("sensorOffsetY", sensorOffsetY_);
        this->get_parameter("twoWayDrive", twoWayDrive_);
        this->get_parameter("laserVoxelSize", laserVoxelSize_);
        this->get_parameter("terrainVoxelSize", terrainVoxelSize_);
        this->get_parameter("useTerrainAnalysis", useTerrainAnalysis_);
        this->get_parameter("checkObstacle", checkObstacle_);
        this->get_parameter("checkRotObstacle", checkRotObstacle_);
        this->get_parameter("adjacentRange", adjacentRange_);
        this->get_parameter("obstacleHeightThre", obstacleHeightThre_);
        this->get_parameter("groundHeightThre", groundHeightThre_);
        this->get_parameter("costHeightThre", costHeightThre_);
        this->get_parameter("costScore", costScore_);
        this->get_parameter("useCost", useCost_);
        this->get_parameter("pointPerPathThre", pointPerPathThre_);
        this->get_parameter("minRelZ", minRelZ_);
        this->get_parameter("maxRelZ", maxRelZ_);
        this->get_parameter("maxSpeed", maxSpeed_);
        this->get_parameter("dirWeight", dirWeight_);
        this->get_parameter("dirThre", dirThre_);
        this->get_parameter("dirToVehicle", dirToVehicle_);
        this->get_parameter("pathScale", pathScale_);
        this->get_parameter("minPathScale", minPathScale_);
        this->get_parameter("pathScaleStep", pathScaleStep_);
        this->get_parameter("pathScaleBySpeed", pathScaleBySpeed_);
        this->get_parameter("minPathRange", minPathRange_);
        this->get_parameter("pathRangeStep", pathRangeStep_);
        this->get_parameter("pathRangeBySpeed", pathRangeBySpeed_);
        this->get_parameter("pathCropByGoal", pathCropByGoal_);
        this->get_parameter("autonomyMode", autonomyMode_);
        this->get_parameter("autonomySpeed", autonomySpeed_);
        this->get_parameter("joyToSpeedDelay", joyToSpeedDelay_);
        this->get_parameter("joyToCheckObstacleDelay", joyToCheckObstacleDelay_);
        this->get_parameter("goalClearRange", goalClearRange_);
        this->get_parameter("goalX", goalX_);
        this->get_parameter("goalY", goalY_);

        // Set default path folder
        if (pathFolder_.empty()) {
            pathFolder_ = "/home/z/rm_simulation/src/local_planner/paths";
        }
    }

    void setupPublishersAndSubscribers()
    {
        // Subscribers
        subOdometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 5, std::bind(&LocalPlanner::odometryHandle, this, std::placeholders::_1));
        
        subLaserCloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/registered_scan", 5, std::bind(&LocalPlanner::laserCloudHandler, this, std::placeholders::_1));
        
        subTerrainCloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/terrain_map", 5, std::bind(&LocalPlanner::terrainCloudHandler, this, std::placeholders::_1));
        
        subGoal_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/way_point", 5, std::bind(&LocalPlanner::goalHandler, this, std::placeholders::_1));
        
        subBoundary_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
            "/navigation_boundary", 5, std::bind(&LocalPlanner::boundaryHandle, this, std::placeholders::_1));

        // Publishers
        pubLaserCloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/plannerCloud", 5);
        pubLaserCloud2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/plannerCloudCropPlanner", 5);
        pubObstacleCloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/visObstacleCloud", 5);
        pubMarker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/path_debug", 10);
        pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/local_path", 5);
        
#if PLOTPATHSET == 1
        pubFreePaths_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/free_paths", 2);
#endif
    }

    void initializeFilters()
    {
        laserDwzFilter_.setLeafSize(laserVoxelSize_, laserVoxelSize_, laserVoxelSize_);
        terrainDwzFilter_.setLeafSize(terrainVoxelSize_, terrainVoxelSize_, terrainVoxelSize_);
    }

    void initializePointClouds()
    {
        // Initialize point clouds
        laserCloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        laserCloudCrop_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        laserCloudDWZ_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        plannerCloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        terrainCloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        terrainCloudCrop_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        terrainCloudDwz_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        plannerCloudCrop_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        boundaryCloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        plannerCloudBody_.reset(new pcl::PointCloud<pcl::PointXYZI>());

        // Initialize laser cloud stack
        for (int i = 0; i < laserCloudStackNum_; i++) {
            laserCloudStack_.push_back(pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>()));
        }

        // Initialize start paths
        for (int i = 0; i < groupNum_; i++) {
            startPaths_.push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>()));
        }

#if PLOTPATHSET == 1
        // Initialize paths
        for (int i = 0; i < pathNum_; i++) {
            paths_.push_back(pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>()));
        }
        freePaths_.reset(new pcl::PointCloud<pcl::PointXYZI>());
#endif

        // Initialize correspondences
        for (int i = 0; i < gridVoxelNum_; i++) {
            correspondences_[i].resize(0);
        }
    }

    // Callback functions
    void odometryHandle(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
    {
        odomTime_ = rclcpp::Time(odom->header.stamp).seconds();
        double roll, pitch, yaw;
        geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
        tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

        vehicleRoll_ = roll;
        vehiclePitch_ = pitch;
        vehicleYaw_ = yaw;
        vehicleX_ = odom->pose.pose.position.x;
        vehicleY_ = odom->pose.pose.position.y;
        vehicleZ_ = odom->pose.pose.position.z;

        RCLCPP_DEBUG(this->get_logger(), "vehicleX: %f, vehicleY: %f", vehicleX_, vehicleY_);
    }

    void laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud2)
    {
        if (!useTerrainAnalysis_) {
            laserCloud_->clear();
            pcl::fromROSMsg(*laserCloud2, *laserCloud_);

            pcl::PointXYZI point;
            laserCloudCrop_->clear();
            int laserCloudSize = laserCloud_->points.size();
            for (int i = 0; i < laserCloudSize; i++) {
                point = laserCloud_->points[i];
                float pointX = point.x;
                float pointY = point.y;
                float pointZ = point.z;
                float dis = sqrt((pointX - vehicleX_) * (pointX - vehicleX_) + 
                                (pointY - vehicleY_) * (pointY - vehicleY_));
                if (dis < adjacentRange_) {
                    point.x = pointX;
                    point.y = pointY;
                    point.z = pointZ;
                    laserCloudCrop_->push_back(point);
                }
            }

            laserCloudDWZ_->clear();
            laserDwzFilter_.setInputCloud(laserCloudCrop_);
            laserDwzFilter_.filter(*laserCloudDWZ_);

            newlaserCloud_ = true;
        }
    }

    void terrainCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrainCloud2)
    {
        if (useTerrainAnalysis_) {
            terrainCloud_->clear();
            pcl::fromROSMsg(*terrainCloud2, *terrainCloud_);

            pcl::PointXYZI point;
            terrainCloudCrop_->clear();
            int terrainCloudSize = terrainCloud_->points.size();
            for (int i = 0; i < terrainCloudSize; i++) {
                point = terrainCloud_->points[i];
                float pointX = point.x;
                float pointY = point.y;
                float pointZ = point.z;
                float dis = std::sqrt((pointX - vehicleX_) * (pointX - vehicleX_) +
                                     (pointY - vehicleY_) * (pointY - vehicleY_));
                if (dis < adjacentRange_ && (point.intensity > obstacleHeightThre_ || useCost_)) {
                    point.x = pointX;
                    point.y = pointY;
                    point.z = pointZ;
                    terrainCloudCrop_->push_back(point);
                }
            }

            terrainCloudDwz_->clear();
            terrainDwzFilter_.setInputCloud(terrainCloudCrop_);
            terrainDwzFilter_.filter(*terrainCloudDwz_);

            newTerrainCloud_ = true;
        }
    }

    void boundaryHandle(const geometry_msgs::msg::PolygonStamped::ConstSharedPtr /*boundary*/)
    {
        // Implementation for boundary handling
    }

    void goalHandler(const geometry_msgs::msg::PointStamped::ConstSharedPtr goal)
    {
        std::string frame_id = goal->header.frame_id;
        goalX_ = goal->point.x;
        goalY_ = goal->point.y;
        RCLCPP_DEBUG(this->get_logger(), "Goal updated: (%f, %f)", goalX_, goalY_);
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
        std::string fileName = pathFolder_ + "/startPaths.ply";
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

            if (groupID >= 0 && groupID < groupNum_) {
                startPaths_[groupID]->push_back(point);
            }
        }

        fclose(filePtr);
        RCLCPP_INFO(this->get_logger(), "Loaded %d start points from %s", pointNum, fileName.c_str());
        return true;
    }

#if PLOTPATHSET == 1
    bool readPaths_safe()
    {
        std::string fileName = pathFolder_ + "/paths.ply";
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

            if (pathID >= 0 && pathID < pathNum_) {
                ++pointSkipCount;
                if (pointSkipCount > pointSkipNum) {
                    paths_[pathID]->push_back(point);
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
        std::string fileName = pathFolder_ + "/pathList.ply";
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

        if (pathNum_ != headerCount) {
            fclose(filePtr);
            RCLCPP_ERROR(this->get_logger(), "Path number mismatch in %s: expected %d, got %d", 
                        fileName.c_str(), pathNum_, headerCount);
            return false;
        }

        int val1, val2, val3, val4, val5, pathID, groupID;
        float endX, endY, endZ;
        for (int i = 0; i < pathNum_; ++i) {
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

            if (pathID >= 0 && pathID < pathNum_ && groupID >= 0 && groupID < groupNum_) {
                pathList_[pathID] = groupID;
                endDirPathList_[pathID] = 2.0 * atan2(endY, endX) * 180.0 / PI;
            }
        }

        fclose(filePtr);
        RCLCPP_INFO(this->get_logger(), "Loaded path list from %s", fileName.c_str());
        return true;
    }

    bool readCorrespondences_safe()
    {
        std::string fileName = pathFolder_ + "/correspondences.txt";
        FILE *filePtr = fopen(fileName.c_str(), "r");
        if (filePtr == NULL) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open %s: %s", fileName.c_str(), strerror(errno));
            return false;
        }

        int val1, gridVoxelID, pathID;
        for (int i = 0; i < gridVoxelNum_; ++i) {
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
                    if (gridVoxelID >= 0 && gridVoxelID < gridVoxelNum_ && pathID >= 0 && pathID < pathNum_) {
                        correspondences_[gridVoxelID].push_back(pathID);
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
        RCLCPP_INFO(this->get_logger(), "pathFolder = %s", pathFolder_.c_str());

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
};

// Static member initialization
int LocalPlanner::lastSelectedGroupID_ = -1;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}