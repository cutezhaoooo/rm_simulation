#ifndef LQR_CONTROLLER_HPP
#define LQR_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "angles/angles.h"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.hpp"

#include "visualization_msgs/msg/marker.hpp"

#include "eigen3/Eigen/Dense"
#include <memory>

class LqrController : public rclcpp::Node
{
public:
    LqrController();
    ~LqrController();

    void initializer();
    void pathHandle(const nav_msgs::msg::Path::SharedPtr path);

private:


    // 参数
    double max_v_, min_v_, max_v_inc_;
    double max_w_, min_w_, max_w_inc_;
    Eigen::Matrix3d Q_;
    Eigen::Matrix2d R_;
    int lqr_max_iter_;
    double lqr_eps_iter_;
    std::string odom_frame_;
    double goal_dist_tol_;
    double rotate_tol_;
    double lookahead_time_;
    double min_lookahead_dist_;
    double max_lookahead_dist_;
    double rotate_to_heading_angular_vel_ = 1.5;
    double max_angular_accel_ = 3.0;
    double rotate_to_heading_min_angle_ = 3.0;
    double d_time=0.1;

    // 上一次的速度 初始值都为0
    geometry_msgs::msg::Twist last_cmd_vel_;


    // ROS2 订阅者和发布者
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_marker_pub_;

    // 初始化tf2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 其他成员函数声明
    void odomHandle(const nav_msgs::msg::Odometry::SharedPtr odom);
    void controlTimerCallback();
    double linearRegularization(double v,double vd);

    
    // 内部状态
    nav_msgs::msg::Odometry current_odom_;
    nav_msgs::msg::Path current_path_;
    bool odom_initialized_;
    bool path_initialized_;

    geometry_msgs::msg::Twist cmd_vel;

    // 标记ID
    int marker_id_;

    // 定时器
    rclcpp::TimerBase::SharedPtr control_timer_;

private:

    double getLookAheadDistance(const geometry_msgs::msg::Twist &speed);

    double getCuspDist(const nav_msgs::msg::Path & path);

    geometry_msgs::msg::PoseStamped getLookAheadPoint(const double & lookahead_dist,const nav_msgs::msg::Path & path);

    geometry_msgs::msg::Quaternion getOrientation(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2);

    geometry_msgs::msg::Point circleSegmentIntersection(const geometry_msgs::msg::Point & p1,const geometry_msgs::msg::Point & p2,double r);

    void visualizeLookaheadPoint(const geometry_msgs::msg::PoseStamped& lookahead_pt, double lookahead_dist);

    bool shouldRotateToGoal(const geometry_msgs::msg::Pose & cur,const geometry_msgs::msg::Pose & goal);

    double dist(const std::pair<double, double>& node1, const std::pair<double, double>& node2);
    double dist(const geometry_msgs::msg::PoseStamped & node1, const geometry_msgs::msg::PoseStamped& node2);

    double rotateToHeading(const double & angle_to_path,const geometry_msgs::msg::Twist & curr_speed);

    bool shouldRotateToPath(
        const geometry_msgs::msg::PoseStamped & target_pose,
        double & angle_to_path, double & sign);

    double computePathCurvatureAtLookahead(
        const geometry_msgs::msg::PoseStamped & lookahead_point,
        const nav_msgs::msg::Path & path,
        double & out_theta);
    
    
    Eigen::Vector2d _lqrControl(Eigen::Vector3d s, Eigen::Vector3d s_d, Eigen::Vector2d u_r, double d_time);

    double regularizeAngle(double angle);
    double angularRegularization(double wt, double w_d);


};

#endif