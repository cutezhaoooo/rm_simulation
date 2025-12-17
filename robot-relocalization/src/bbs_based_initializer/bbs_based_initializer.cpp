#include <bbs_based_initializer.hpp>

#include <pointcloud_iof/pcl_eigen_converter.hpp>
#include <pointcloud_iof/pcd_loader.hpp>
#include <pointcloud_iof/gravity_alignment.hpp>
#include <chrono>
// #include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include <yaml-cpp/yaml.h>
#include <math.h>
#include <type_traits>

// Define BBS3D instance based on compilation mode
#ifdef USE_CUDA
#define BBS3D_INSTANCE gpu_bbs3d
using BBS3D_TYPE = float;
using BBS3D_VECTOR3 = Eigen::Vector3f;
using BBS3D_MATRIX4 = Eigen::Matrix4f;
#else
#define BBS3D_INSTANCE cpu_bbs3d
using BBS3D_TYPE = double;
using BBS3D_VECTOR3 = Eigen::Vector3d;
using BBS3D_MATRIX4 = Eigen::Matrix4d;
#endif

// Type conversion helper functions
template<typename T>
T convert_eigen_vector(const Eigen::Vector3d& vec) {
    if constexpr (std::is_same_v<T, Eigen::Vector3f>) {
        return vec.template cast<float>();
    } else {
        return vec;
    }
}

template<typename T>
Eigen::Vector3d convert_to_double_vector(const T& vec) {
    if constexpr (std::is_same_v<T, Eigen::Vector3f>) {
        return vec.template cast<double>();
    } else {
        return vec;
    }
}

Eigen::Vector3d to_eigen(const std::vector<double> &vec) {
    Eigen::Vector3d e_vec;
    for (int i = 0; i < 3; ++i) {
        if (vec[i] == 6.28) {
            e_vec(i) = 2 * M_PI;
        } else {
            e_vec(i) = vec[i];
        }
    }
    return e_vec;
}

BbsBasedInitializer::BbsBasedInitializer(const rclcpp::NodeOptions &node_options) : Node(
    "bbs_based_initializer", node_options) {
    //  ==== Load config file ====
    std::cout << "[ROS2] Loading config file..." << std::endl;
    
    // Display compilation mode
#ifdef USE_CUDA
    std::cout << "[INFO] Running in GPU mode with CUDA acceleration" << std::endl;
#else
    std::cout << "[INFO] Running in CPU mode" << std::endl;
#endif
    
    // 加载函数的配置文件
    std::string config = this->declare_parameter<std::string>("config");
    if (!load_config(config)) {
        std::cout << "[ERROR] Loading config file failed" << std::endl;
    };

    // ==== ROS 2 ser ====
    // 创建了一个 trigger_service bool 类型的服务
    trigger_ser_ = this->create_service<std_srvs::srv::SetBool>("trigger_service",
                                                                     std::bind(&BbsBasedInitializer::trigger_callback,
                                                                               this, std::placeholders::_1,
                                                                               std::placeholders::_2));
    // ==== ROS 2 sub ====
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic_name,
        100,
        std::bind(&BbsBasedInitializer::cloud_callback, this, std::placeholders::_1));

    map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        map_topic_name,
        100,
        std::bind(&BbsBasedInitializer::map_callback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic_name, 100,
                                                                std::bind(&BbsBasedInitializer::imu_callback, this,
                                                                          std::placeholders::_1));

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic_name, 10);


    // initialize map load status
    is_map_loaded = false;
}

BbsBasedInitializer::~BbsBasedInitializer() {
}

void BbsBasedInitializer::trigger_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                           const std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    std::cout << "[Trigger] Service called with data: " << (request->data ? "true" : "false") << std::endl;
    
    // 检查是否存在 map
    if (!is_map_loaded) {
        std::cout << "[Trigger] ERROR: Map not loaded yet!" << std::endl;
        response->success = false;
        response->message = "Map not loaded";
        return;
    }
    
    if (!request->data) {
        std::cout << "[Trigger] Service called with data=false, ignoring" << std::endl;
        response->success = false;
        response->message = "Service called with data=false";
        return;
    }
    
    if (!source_cloud_msg_) {
        std::cout << "[Trigger] ERROR: Point cloud message not received!" << std::endl;
        std::cout << "[Trigger] Please check if LiDAR topic '" << lidar_topic_name << "' is publishing" << std::endl;
        response->success = false;
        response->message = "Point cloud message not received";
        return;
    }

    if (imu_buffer.empty()) {
        std::cout << "[Trigger] ERROR: IMU message not received!" << std::endl;
        std::cout << "[Trigger] Please check if IMU topic '" << imu_topic_name << "' is publishing" << std::endl;
        response->success = false;
        response->message = "IMU message not received";
        return;
    }
    
    std::cout << "[Trigger] All prerequisites met, starting localization..." << std::endl;
    std::cout << "[Trigger] Point cloud size: " << source_cloud_msg_->width * source_cloud_msg_->height << " points" << std::endl;
    std::cout << "[Trigger] IMU buffer size: " << imu_buffer.size() << " messages" << std::endl;
    // 点云数据预处理
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*source_cloud_msg_, *src_cloud);

    // filter
    // 降采样
    if (src_leaf_size != 0.0f) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> filter;
        filter.setLeafSize(src_leaf_size, src_leaf_size, src_leaf_size);
        filter.setInputCloud(src_cloud);
        filter.filter(*filtered_cloud_ptr);
        *src_cloud = *filtered_cloud_ptr;
    }

    // Cut scan range
    // 限制扫描范围 过滤掉里传感器很近或者很远的点 也就是只用适中范围的点做匹配
    if (!(min_scan_range == 0.0 && max_scan_range == 0.0)) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cut_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        for (size_t i = 0; i < src_cloud->points.size(); ++i) {
            pcl::PointXYZ point = src_cloud->points[i];
            double norm = pcl::euclideanDistance(point, pcl::PointXYZ(0.0f, 0.0f, 0.0f));

            if (norm >= min_scan_range && norm <= max_scan_range) {
                cut_cloud_ptr->points.push_back(point);
            }
        }
        *src_cloud = *cut_cloud_ptr;
    }

    // 根据imu的加速度 对点云做旋转 使其在地图坐标系下正方向一致
    int imu_index = get_nearest_imu_index(imu_buffer, source_cloud_msg_->header.stamp);
    const auto imu_msg = imu_buffer[imu_index];
    const Eigen::Vector3d acc = {
        imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z
    };
    pcl::transformPointCloud(*src_cloud, *src_cloud, pciof::calc_gravity_alignment_matrix(acc.cast<float>()));

    std::vector<Eigen::Vector3f> src_points;
    pciof::pcl_to_eigen(src_cloud, src_points);
    BBS3D_INSTANCE.set_src_points(src_points);

    std::cout << "[Localize] start" << std::endl;
#ifdef USE_CUDA
    std::cout << "[Localize] Using GPU acceleration for localization" << std::endl;
#else
    std::cout << "[Localize] Using CPU for localization" << std::endl;
#endif
    BBS3D_INSTANCE.localize(); // gloal localization

    if (!BBS3D_INSTANCE.has_localized()) {
        if (BBS3D_INSTANCE.has_timed_out()) {
            std::cout << "[Failed] Localization timed out." << std::endl;
            response->success = false;
            response->message = "Localization timed out";
        } else {
            std::cout << "[Failed] Score is below the threshold." << std::endl;
            std::cout << "[Failed] Best score: " << BBS3D_INSTANCE.get_best_score() << std::endl;
            std::cout << "[Failed] Score percentage: " << BBS3D_INSTANCE.get_best_score_percentage() << std::endl;
            response->success = false;
            response->message = "Score below threshold - best score: " + std::to_string(BBS3D_INSTANCE.get_best_score());
        }
        return;
    }

    std::cout << "[Localize] Execution time: " << BBS3D_INSTANCE.get_elapsed_time() << "[msec] " << std::endl;
    std::cout << "[Localize] score: " << BBS3D_INSTANCE.get_best_score() << std::endl;

    auto estimated_pose = BBS3D_INSTANCE.get_global_pose();

    // generate ros message
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_to_pub(
        new geometry_msgs::msg::PoseWithCovarianceStamped);
    // PoseWithCovarianceStamped
    // - header
    // - pose (PoseWithCovariance)
    //   - pose (Pose)
    //     - position
    //     - orientation
    pose_to_pub->header.frame_id = "map";
    pose_to_pub->header.stamp = source_cloud_msg_->header.stamp; // timestamp of lidar topic
    pose_to_pub->pose.pose.position.x = static_cast<double>(estimated_pose(0, 3));
    pose_to_pub->pose.pose.position.y = static_cast<double>(estimated_pose(1, 3));
    pose_to_pub->pose.pose.position.z = static_cast<double>(estimated_pose(2, 3));
    
    // Convert rotation matrix to quaternion (handle both float and double types)
    Eigen::Matrix3f rotation_matrix_f = estimated_pose.block<3, 3>(0, 0).template cast<float>();
    Eigen::Quaternionf q(rotation_matrix_f);
    pose_to_pub->pose.pose.orientation.x = q.x();
    pose_to_pub->pose.pose.orientation.y = q.y();
    pose_to_pub->pose.pose.orientation.z = q.z();
    pose_to_pub->pose.pose.orientation.w = q.w();
    // FIXME: whether to initialize covariance?
    pose_pub_->publish(*pose_to_pub);

    response->success = true;
    response->message = "Service processed successfully";
}


// get imu message which has closest timestamp with given timestamp
int BbsBasedInitializer::get_nearest_imu_index(const std::vector<sensor_msgs::msg::Imu> &imu_buffer,
                                               const builtin_interfaces::msg::Time &stamp) {
    int imu_index = 0;
    double min_diff = 1000;
    for (int i = 0; i < imu_buffer.size(); ++i) {
        double diff = std::abs(
            imu_buffer[i].header.stamp.sec + imu_buffer[i].header.stamp.nanosec * 1e-9 - source_cloud_msg_->header.stamp
            .sec -
            source_cloud_msg_->header.stamp.nanosec * 1e-9);
        if (diff < min_diff) {
            imu_index = i;
            min_diff = diff;
        }
    }
    return imu_index;
}

void BbsBasedInitializer::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!msg) {
        std::cout << "[LiDAR] Received null point cloud message" << std::endl;
        return;
    }
    source_cloud_msg_ = msg;
    std::cout << "[LiDAR] Received point cloud with " << msg->width * msg->height << " points" << std::endl;
}

void BbsBasedInitializer::map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::cout << "[Map] Map callback triggered" << std::endl;

    if (is_map_loaded) {
        std::cout << "[Map] Map already loaded, skipping..." << std::endl;
        return;
    }
    
    if (!msg) {
        std::cout << "[Map] ERROR: Received null map message!" << std::endl;
        return;
    }
    
    std::cout << "[Map] Received map with " << msg->width * msg->height << " points" << std::endl;

    // convert ROS message
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud_ptr);
    // do downsample when leaf size is provided
    if (map_leaf_size != 0.0f) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::ApproximateVoxelGrid<pcl::PointXYZ> filter;
        filter.setLeafSize(map_leaf_size, map_leaf_size, map_leaf_size);
        filter.setInputCloud(cloud_ptr);
        filter.filter(*filtered_cloud_ptr);
        *cloud_ptr = *filtered_cloud_ptr;
    }

    // pcl to eigen
    std::vector<Eigen::Vector3f> map_points;
    pciof::pcl_to_eigen(cloud_ptr, map_points);

    std::cout << "[BBS3D:init] Creating hierarchical voxel map..." << std::endl;
#ifdef USE_CUDA
    std::cout << "[BBS3D:init] Using GPU acceleration for voxel map creation" << std::endl;
#else
    std::cout << "[BBS3D:init] Using CPU for voxel map creation" << std::endl;
#endif
    BBS3D_INSTANCE.set_tar_points(map_points, static_cast<BBS3D_TYPE>(min_level_res), max_level);
    BBS3D_INSTANCE.set_trans_search_range(map_points);

    BBS3D_INSTANCE.set_angular_search_range(convert_eigen_vector<BBS3D_VECTOR3>(min_rpy), convert_eigen_vector<BBS3D_VECTOR3>(max_rpy));
    BBS3D_INSTANCE.set_score_threshold_percentage(static_cast<BBS3D_TYPE>(score_threshold_percentage));
    if (timeout_msec > 0) {
        BBS3D_INSTANCE.enable_timeout();
        BBS3D_INSTANCE.set_timeout_duration_in_msec(timeout_msec);
    }

    std::cout << "[BBS3D:init] READY!" << std::endl;

    is_map_loaded = true;
}

void BbsBasedInitializer::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (!msg) {
        std::cout << "[IMU] Received null IMU message" << std::endl;
        return;
    }
    imu_buffer.emplace_back(*msg);
    if (imu_buffer.size() > 30) {
        imu_buffer.erase(imu_buffer.begin());
    }
    
    // Only print IMU info occasionally to avoid spam
    static int imu_count = 0;
    if (++imu_count % 50 == 0) {  // Print every 50th message
        std::cout << "[IMU] Received IMU data, buffer size: " << imu_buffer.size() << std::endl;
    }
}

bool BbsBasedInitializer::load_config(const std::string &config) {
    YAML::Node conf = YAML::LoadFile(config);

    std::cout << "[YAML] Loading topic name..." << std::endl;
    lidar_topic_name = conf["lidar_topic_name"].as<std::string>();
    map_topic_name = conf["map_topic_name"].as<std::string>();
    imu_topic_name = conf["imu_topic_name"].as<std::string>();
    std::cout << "[YAML] LiDAR topic name..." << lidar_topic_name << std::endl;
    std::cout << "[YAML] imu topic name..." << imu_topic_name << std::endl;

    trigger_topic_name = conf["trigger_topic_name"].as<std::string>();
    // for pub
    pose_topic_name = conf["pose_topic_name"].as<std::string>();

    std::cout << "[YAML] Loading 3D-BBS parameters..." << std::endl;
    min_level_res = conf["min_level_res"].as<double>();
    max_level = conf["max_level"].as<int>();

    if (min_level_res == 0.0 || max_level == 0) {
        std::cout << "[ERROR] Set min_level and num_layers except for 0" << std::endl;
        return false;
    }

    std::cout << "[YAML] Loading angular search range..." << std::endl;
    std::vector<double> min_rpy_temp = conf["min_rpy"].as<std::vector<double> >();
    std::vector<double> max_rpy_temp = conf["max_rpy"].as<std::vector<double> >();
    if (min_rpy_temp.size() == 3 && max_rpy_temp.size() == 3) {
        min_rpy = to_eigen(min_rpy_temp);
        max_rpy = to_eigen(max_rpy_temp);
    } else {
        std::cout << "[ERROR] Set min_rpy and max_rpy correctly" << std::endl;
        return false;
    }

    std::cout << "[YAML] Loading score threshold percentage..." << std::endl;
    score_threshold_percentage = conf["score_threshold_percentage"].as<double>();

    std::cout << "[YAML] Loading downsample parameters..." << std::endl;
    map_leaf_size = conf["map_leaf_size"].as<float>();
    src_leaf_size = conf["src_leaf_size"].as<float>();
    min_scan_range = conf["min_scan_range"].as<double>();
    max_scan_range = conf["max_scan_range"].as<double>();

    timeout_msec = conf["timeout_msec"].as<int>();

    return true;
}
