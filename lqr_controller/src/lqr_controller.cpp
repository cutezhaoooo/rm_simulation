#include "lqr_controller/lqr_controller.hpp"
#include <functional>
#include <memory>

using namespace std::chrono_literals;

LqrController::LqrController() 
    : Node("lqr_controller")
{
    // 在构造函数中初始化成员变量
    initializer();
}

LqrController::~LqrController()
{
    // 清理资源（如果需要）
}

void LqrController::initializer()
{
    // 初始化参数
    max_v_ = 0.5;
    min_v_ = 0.0;
    max_v_inc_ = 0.1;
    max_w_ = 1.0;
    min_w_ = -1.0;
    max_w_inc_ = 0.5;
    lqr_max_iter_ = 150;
    lqr_eps_iter_ = 0.01;
    odom_frame_ = "odom";
    goal_dist_tol_ = 0.1;
    rotate_tol_ = 0.1;
    lookahead_time_ = 1.5;
    min_lookahead_dist_ = 0.3;
    max_lookahead_dist_ = 3.0;

    // 初始化 Q 和 R 矩阵
    Q_ = Eigen::Matrix3d::Identity();
    R_ = Eigen::Matrix2d::Identity();

    // 初始化状态标志
    odom_initialized_ = false;
    path_initialized_ = false;

    // 创建订阅者和发布者
    // path在map坐标系下面
    sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
        "local_path", 
        5, 
        std::bind(&LqrController::pathHandle, this, std::placeholders::_1));

    // odom是odom到base link的坐标变换 如果不在同一个坐标系下面就需要转换一下
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "Odometry",
        10,
        std::bind(&LqrController::odomHandle, this, std::placeholders::_1));

    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // 创建控制定时器（例如 10Hz）
    control_timer_ = this->create_wall_timer(
        100ms, 
        std::bind(&LqrController::controlTimerCallback, this));

    RCLCPP_INFO(this->get_logger(), "LQR Controller initialized");
}

void LqrController::pathHandle(const nav_msgs::msg::Path::SharedPtr path)
{
    current_path_ = *path;
    path_initialized_ = true;
    RCLCPP_DEBUG(this->get_logger(), "Received path with %zu points", path->poses.size());
}

void LqrController::odomHandle(const nav_msgs::msg::Odometry::SharedPtr odom)
{
    current_odom_ = *odom;
    odom_initialized_ = true;
}

void LqrController::controlTimerCallback()
{
    // 检查是否已经收到必要的数据
    if (!odom_initialized_ || !path_initialized_) {
        RCLCPP_DEBUG(this->get_logger(), "Waiting for odometry and path data...");
        return;
    }

    if (current_path_.poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "Path is empty");
        return;
    }

    // 在这里实现 LQR 控制逻辑
    // 1. 获取当前机器人状态
    // 2. 计算前瞻点
    // 3. 执行 LQR 控制计算
    // 4. 发布控制命令

    RCLCPP_DEBUG(this->get_logger(), "Executing LQR control");
    
    // 示例：发布零速度（实际实现中替换为 LQR 计算结果）
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    pub_cmd_vel_->publish(cmd_vel);
}