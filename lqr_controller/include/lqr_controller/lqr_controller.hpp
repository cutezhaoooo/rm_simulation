#ifndef LQR_CONTROLLER_HPP
#define LQR_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"

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

    // ROS2 订阅者和发布者
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;

    // 初始化tf2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 其他成员函数声明
    void odomHandle(const nav_msgs::msg::Odometry::SharedPtr odom);
    void controlTimerCallback();
    
    // 内部状态
    nav_msgs::msg::Odometry current_odom_;
    nav_msgs::msg::Path current_path_;
    bool odom_initialized_;
    bool path_initialized_;

    // 定时器
    rclcpp::TimerBase::SharedPtr control_timer_;
};

#endif