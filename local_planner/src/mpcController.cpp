#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // 修复Matrix3x3问题
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nlopt.hpp>
#include <cmath>
#include <vector>
#include "eigen3/Eigen/Dense"
#include <iostream>
#include <memory>
#include <angles/angles.h>
#include <random>
#include <limits>
#include <mutex>

// 机器人状态结构体
struct RobotState
{
    double x;     // x坐标
    double y;     // y坐标
    double theta; // 朝向角 (rad)
    double v;     // 线速度 (m/s)
    double omega; // 角速度 (rad/s)
};

// 控制量结构体
struct Control
{
    double v;     // 线速度控制量
    double omega; // 角速度控制量
};

// 路径点结构体
struct PathPoint
{
    double x;     // x坐标
    double y;     // y坐标
    double theta; // 期望朝向角
};

double min_radius = std::numeric_limits<double>::max();

// 控制参数和约束
namespace Param
{
    // 时间参数
    constexpr double Ts = 0.05; // 控制周期 (s)
    constexpr int N = 30;       // 预测时域长度

    // 机器人物理约束
    constexpr double MinR = 1.5;        // 最小转弯半径 (m)
    constexpr double MaxLinear = 2.0;   // 最大线速度 (m/s)
    constexpr double MinLinear = -2.0;  // 最小线速度 (m/s)
    constexpr double MaxAngular = 2.0;  // 最大角速度 (rad/s)
    constexpr double MinAngular = -2.0; // 最小角速度 (rad/s)

    // 加速度约束
    constexpr double MaxAccelLinear = 1.5;  // 最大线加速度 (m/s²)
    constexpr double MinAccelLinear = -1.5; // 最大线减速度 (m/s²)
    constexpr double MaxAccelAngular = 6;   // 最大角加速度 (rad/s²)
    constexpr double MinAccelAngular = -6;  // 最大角减速度 (rad/s²)

    // MPC权重
    constexpr double Wx = 20.0;          // x位置权重
    constexpr double Wy = 20.0;          // y位置权重
    constexpr double Wtheta = 10.0;      // 朝向角权重
    constexpr double Wv = 0.02;          // 线速度权重
    constexpr double Womega = 0.1;       // 角速度权重
    constexpr double WRadius = 1.0;      // 转弯半径权重
    constexpr double kStopPenalty = 0.0; // 停止惩罚
    constexpr double Rv = 0.1;           // 线速度变化权重
    constexpr double Romega = 0.1;       // 角速度变化权重

    // 优化器参数
    constexpr double OptTolerance = 1e-3;   // 优化容差
    constexpr int GlobalMaxEval = 500;      // 全局优化最大评估次数
    constexpr int LocalMaxEval = 1000;      // 局部优化最大评估次数
    constexpr double GlobalInitNoise = 0.2; // 全局优化初始值扰动范围
}

class MPCController : public rclcpp::Node
{
private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    RobotState current_state_;
    std::vector<PathPoint> reference_path_;
    Control previous_control_;

    // 优化器相关
    nlopt::opt global_optimizer_; // 全局优化器
    nlopt::opt local_optimizer_;  // 局部优化器
    std::vector<double> opt_result_;
    double opt_fval_;

    size_t last_index_{0};
    std::mt19937 rng_; // 随机数生成器
    std::mutex path_mutex_; // 保护路径访问的互斥锁

public:
    MPCController()
        : Node("mpc_path_tracker_global_local"),
          global_optimizer_(nlopt::GN_ISRES, 2 * Param::N), // 改用支持约束的全局优化算法
          local_optimizer_(nlopt::LD_SLSQP, 2 * Param::N),  // 局部优化器
          rng_(std::random_device{}())
    {
        // 初始化机器人状态
        current_state_ = {0.0, 0.0, 0, 0.0, 0.0};
        previous_control_ = {0.0, 0.0};

        // 配置优化器
        configureGlobalOptimizer();
        configureLocalOptimizer();

        // 初始化ROS发布器和订阅器
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("reference_path", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_chassis", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 订阅 local_path 和 odom
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/local_path",
            10,
            std::bind(&MPCController::pathCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", // 或 "/odom"，根据你的里程计话题名称
            10,
            std::bind(&MPCController::odomCallback, this, std::placeholders::_1));

        // 控制定时器
        control_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(Param::Ts),
            std::bind(&MPCController::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "MPC Path Tracker initialized, waiting for /local_path and /odom...");
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 更新机器人状态
        current_state_.x = msg->pose.pose.position.x;
        current_state_.y = msg->pose.pose.position.y;

        // 使用 tf2_geometry_msgs 提取欧拉角
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        current_state_.theta = yaw;

        // 从 twist 获取速度（可选，用于更精确的模型）
        current_state_.v = msg->twist.twist.linear.x;
        current_state_.omega = msg->twist.twist.angular.z;
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty path!");
            return;
        }

        std::lock_guard<std::mutex> lock(path_mutex_);
        reference_path_.clear();
        reference_path_.reserve(msg->poses.size());

        for (const auto &pose_stamped : msg->poses)
        {
            PathPoint p;
            p.x = pose_stamped.pose.position.x;
            p.y = pose_stamped.pose.position.y;

            // 提取 yaw 角度
            tf2::Quaternion q;
            tf2::fromMsg(pose_stamped.pose.orientation, q);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            p.theta = yaw;

            reference_path_.push_back(p);
        }

        // 重置 last_index_，避免越界
        last_index_ = 0;

        RCLCPP_DEBUG(this->get_logger(), "Received new path with %zu points", reference_path_.size());
    }

    // 配置全局优化器（使用支持约束的算法）
    void configureGlobalOptimizer()
    {
        global_optimizer_.set_min_objective(MPCController::objectiveFunction, this);

        // 变量上下界
        std::vector<double> lb(2 * Param::N);
        std::vector<double> ub(2 * Param::N);
        for (int i = 0; i < Param::N; ++i)
        {
            lb[2 * i] = Param::MinLinear;
            ub[2 * i] = Param::MaxLinear;
            lb[2 * i + 1] = Param::MinAngular;
            ub[2 * i + 1] = Param::MaxAngular;
        }
        global_optimizer_.set_lower_bounds(lb);
        global_optimizer_.set_upper_bounds(ub);

        // 全局优化参数
        global_optimizer_.set_xtol_rel(1e-2);
        global_optimizer_.set_maxeval(Param::GlobalMaxEval);
    }

    // 配置局部优化器
    void configureLocalOptimizer()
    {
        local_optimizer_.set_min_objective(MPCController::objectiveFunction, this);

        // 变量上下界
        std::vector<double> lb(2 * Param::N);
        std::vector<double> ub(2 * Param::N);
        for (int i = 0; i < Param::N; ++i)
        {
            lb[2 * i] = Param::MinLinear;
            ub[2 * i] = Param::MaxLinear;
            lb[2 * i + 1] = Param::MinAngular;
            ub[2 * i + 1] = Param::MaxAngular;
        }
        local_optimizer_.set_lower_bounds(lb);
        local_optimizer_.set_upper_bounds(ub);

        // 应用约束
        local_optimizer_.add_inequality_constraint(MPCController::accelerationConstraints, this, Param::OptTolerance);
        local_optimizer_.add_inequality_constraint(MPCController::turningRadiusConstraint, this, Param::OptTolerance);

        // 局部优化参数
        local_optimizer_.set_xtol_rel(Param::OptTolerance);
        local_optimizer_.set_maxeval(Param::LocalMaxEval);
        local_optimizer_.set_param("initial_step", 0.01);
    }

    // 查找参考路径最近点
    int findNearestWaypoint(const RobotState &state, size_t start_index)
    {
        if (reference_path_.empty())
            return 0;

        double min_dist = INFINITY;
        int nearest_idx = last_index_;
        int path_size = reference_path_.size();

        // 搜索范围：last_index_附近50个点
        for (int i = 0; i < 50; ++i)
        {
            int idx = (last_index_ + i) % path_size;
            double dx = state.x - reference_path_[idx].x;
            double dy = state.y - reference_path_[idx].y;
            double dist = dx * dx + dy * dy;

            if (dist < min_dist)
            {
                min_dist = dist;
                nearest_idx = idx;
            }
        }

        last_index_ = nearest_idx;
        return nearest_idx;
    }

    // 车辆运动学模型
    RobotState predictState(const RobotState &current, const Control &u, double dt, MPCController *mpc)
    {
        RobotState next;
        next.x = current.x + current.v * cos(current.theta) * dt;
        next.y = current.y + current.v * sin(current.theta) * dt;
        next.theta = angles::normalize_angle(current.theta + current.omega * dt);

        // 直接应用控制量
        next.v = u.v;
        next.omega = u.omega;

        return next;
    }

    // 目标函数
    static double objectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void *data)
    {
        MPCController *mpc = static_cast<MPCController *>(data);
        double cost = 0.0;
        constexpr double kEps = 1e-6;

        // 获取当前状态并找到参考路径最近点
        RobotState current = mpc->current_state_;
        int nearest_idx = mpc->findNearestWaypoint(current, mpc->last_index_);
        mpc->last_index_ = nearest_idx;

        // 预测未来状态并计算成本
        for (int i = 0; i < Param::N; ++i)
        {
            Control u = {x[2 * i], x[2 * i + 1]};
            current = mpc->predictState(current, u, Param::Ts, mpc);
            int ref_idx = (nearest_idx + i) % mpc->reference_path_.size();
            const PathPoint &ref = mpc->reference_path_[ref_idx];

            // 位置误差成本
            cost += Param::Wx * pow(current.x - ref.x, 2);
            cost += Param::Wy * pow(current.y - ref.y, 2);
            cost += Param::Wtheta * pow(angles::shortest_angular_distance(current.theta, ref.theta), 2);

            // 转弯半径成本
            if (fabs(current.omega) > kEps)
            {
                double radius = fabs(current.v / current.omega);
                if (radius > kEps)
                    cost += Param::WRadius * (1.0 / radius);
            }

            // 速度平滑与惩罚成本
            double ref_v = Param::MaxLinear;
            cost += Param::Wv * (1.0 / pow(M_E, current.v));
            cost += Param::Womega * pow(current.omega, 2);

            // 控制量平滑性成本
            if (i == 0)
            {
                cost += Param::Rv * fabs(u.v - mpc->previous_control_.v) / Param::Ts;
                cost += Param::Romega * fabs(u.omega - mpc->previous_control_.omega) / Param::Ts;
            }
            else
            {
                cost += Param::Rv * fabs(u.v - x[2 * (i - 1)]) / Param::Ts;
                cost += Param::Romega * fabs(u.omega - x[2 * (i - 1) + 1]) / Param::Ts;
            }
        }

        // 计算梯度（仅局部优化器需要）
        if (!grad.empty() && mpc->local_optimizer_.get_algorithm() == nlopt::LD_SLSQP)
        {
            double eps = 1e-3;
            std::vector<double> x_eps = x;
            std::vector<double> tp;

            // 数值梯度计算
            for (size_t i = 0; i < x.size(); ++i)
            {
                x_eps[i] += eps;
                double f_plus = objectiveFunction(x_eps, tp, data);
                x_eps[i] -= 2 * eps;
                double f_minus = objectiveFunction(x_eps, tp, data);
                grad[i] = (f_plus - f_minus) / (2 * eps);
                x_eps[i] = x[i];
            }
        }

        return cost;
    }

    // 加速度约束
    static double accelerationConstraints(const std::vector<double> &x, std::vector<double> &grad, void *data)
    {
        MPCController *mpc = static_cast<MPCController *>(data);
        double max_violation = 0.0;

        // 首步控制量与历史控制量的加速度约束
        double dv = x[0] - mpc->previous_control_.v;
        double domega = x[1] - mpc->previous_control_.omega;
        double accel_linear = dv / Param::Ts;
        double accel_angular = domega / Param::Ts;
        max_violation = std::max({max_violation,
                                  accel_linear - Param::MaxAccelLinear,
                                  Param::MinAccelLinear - accel_linear,
                                  accel_angular - Param::MaxAccelAngular,
                                  Param::MinAccelAngular - accel_angular});

        // 后续控制量之间的加速度约束
        for (int i = 1; i < Param::N; ++i)
        {
            dv = x[2 * i] - x[2 * (i - 1)];
            domega = x[2 * i + 1] - x[2 * (i - 1) + 1];
            accel_linear = dv / Param::Ts;
            accel_angular = domega / Param::Ts;
            max_violation = std::max({max_violation,
                                      accel_linear - Param::MaxAccelLinear,
                                      Param::MinAccelLinear - accel_linear,
                                      accel_angular - Param::MaxAccelAngular,
                                      Param::MinAccelAngular - accel_angular});
        }

        // 计算梯度（仅局部优化器需要）
        if (!grad.empty() && mpc->local_optimizer_.get_algorithm() == nlopt::LD_SLSQP)
            std::fill(grad.begin(), grad.end(), 0.0);

        return max_violation;
    }

    // 最小转弯半径约束（修复了mpc未声明的问题）
    static double turningRadiusConstraint(const std::vector<double> &x, std::vector<double> &grad, void *data)
    {
        // 关键修复：从data参数获取mpc指针
        MPCController *mpc = static_cast<MPCController *>(data);

        constexpr double eps = 1e-3;
        double max_violation = 0.0;

        // 检查每个时刻的转弯半径
        for (int i = 0; i < Param::N; ++i)
        {
            double v = x[2 * i];
            double omega = x[2 * i + 1];
            if (fabs(omega) > eps)
            {
                double radius = fabs(v / omega);
                max_violation = std::max(max_violation, Param::MinR - radius);
            }
        }

        // 计算梯度（仅局部优化器需要）
        if (!grad.empty() && mpc->local_optimizer_.get_algorithm() == nlopt::LD_SLSQP)
            std::fill(grad.begin(), grad.end(), 0.0);

        return max_violation;
    }

    // 核心优化流程
    Control computeControl()
    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        if (reference_path_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No path available for optimization, returning zero control.");
            return {0.0, 0.0};
        }

        // 1. 生成全局优化的初始值（带随机扰动）
        std::vector<double> global_x0(2 * Param::N);
        std::uniform_real_distribution<double> dist_v(-Param::GlobalInitNoise, Param::GlobalInitNoise);
        std::uniform_real_distribution<double> dist_omega(-Param::GlobalInitNoise, Param::GlobalInitNoise);

        for (int i = 0; i < Param::N; ++i)
        {
            // 在历史控制量基础上添加小扰动
            global_x0[2 * i] = std::clamp(previous_control_.v + dist_v(rng_), Param::MinLinear, Param::MaxLinear);
            global_x0[2 * i + 1] = std::clamp(previous_control_.omega + dist_omega(rng_), Param::MinAngular, Param::MaxAngular);
        }

        // 2. 执行全局优化
        double global_fval = 0.0;
        nlopt::result global_result = nlopt::FAILURE;
        try
        {
            global_result = global_optimizer_.optimize(global_x0, global_fval);
            RCLCPP_DEBUG(this->get_logger(), "Global Optimization: result=%d, cost=%f", global_result, global_fval);
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "Global Optimization failed: %s (fallback to local init)", e.what());
            // 全局优化失败时，用历史控制量作为局部优化的初始值
            global_x0.assign(2 * Param::N, 0.0);
            for (int i = 0; i < Param::N; ++i)
            {
                global_x0[2 * i] = previous_control_.v;
                global_x0[2 * i + 1] = previous_control_.omega;
            }
        }

        // 3. 执行局部优化
        std::vector<double> local_x0 = global_x0; // 全局结果→局部初始值
        double local_fval = 0.0;
        nlopt::result local_result = nlopt::FAILURE;
        try
        {
            local_result = local_optimizer_.optimize(local_x0, local_fval);
            RCLCPP_DEBUG(this->get_logger(), "Local Optimization: result=%d, cost=%f", local_result, local_fval);
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "Local Optimization failed: %s (fallback to previous control)", e.what());
            return previous_control_;
        }

        // 4. 结果验证与输出
        if (local_result < 0 || std::isnan(local_fval) || std::isinf(local_fval))
        {
            RCLCPP_WARN(this->get_logger(), "Local Optimization produced invalid result (fallback to previous control)");
            return previous_control_;
        }

        // 保存最终优化结果
        opt_result_ = local_x0;
        opt_fval_ = local_fval;

        // 返回第一个控制量
        Control u;
        u.v = std::clamp(local_x0[0], Param::MinLinear, Param::MaxLinear);
        u.omega = std::clamp(local_x0[1], Param::MinAngular, Param::MaxAngular);
        return u;
    }

    // 状态更新（这里可以改为使用odom反馈，当前保持开环积分用于预测）
    void updateRobotState(const Control &u)
    {
        // 对于实际控制，状态由 odomCallback 更新
        // 此函数用于预测轨迹，不更新 current_state_
        // current_state_ = predictState(current_state_, u, Param::Ts, this);
        previous_control_ = u;
    }

    // 可视化发布
    void publishVisualization()
    {
        // 1️⃣ 发布 camera_init -> body 变换
        geometry_msgs::msg::TransformStamped t_body;
        t_body.header.stamp = this->get_clock()->now();
        t_body.header.frame_id = "camera_init";  // 世界坐标
        t_body.child_frame_id = "body";          // 机器人主体
        t_body.transform.translation.x = current_state_.x;
        t_body.transform.translation.y = current_state_.y;
        t_body.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, current_state_.theta);
        t_body.transform.rotation = tf2::toMsg(q);

        tf_broadcaster_->sendTransform(t_body);

        // 2️⃣ 发布 body -> base_link 变换（静态偏移）
        geometry_msgs::msg::TransformStamped t_base;
        t_base.header.stamp = this->get_clock()->now();
        t_base.header.frame_id = "body";
        t_base.child_frame_id = "base_link";
        t_base.transform.translation.x = 0.0;
        t_base.transform.translation.y = 0.0;
        t_base.transform.translation.z = 0.0;
        t_base.transform.rotation.x = 0.0;
        t_base.transform.rotation.y = 0.0;
        t_base.transform.rotation.z = 0.0;
        t_base.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(t_base);

        // 3️⃣ 发布机器人位置Marker（在 camera_init 坐标下）
        visualization_msgs::msg::Marker robot_marker;
        robot_marker.header.frame_id = "camera_init";
        robot_marker.header.stamp = this->get_clock()->now();
        robot_marker.ns = "robot";
        robot_marker.id = 0;
        robot_marker.type = visualization_msgs::msg::Marker::ARROW;
        robot_marker.action = visualization_msgs::msg::Marker::ADD;
        robot_marker.pose.position.x = current_state_.x;
        robot_marker.pose.position.y = current_state_.y;
        robot_marker.pose.position.z = 0.0;
        robot_marker.pose.orientation = tf2::toMsg(q);
        robot_marker.scale.x = 0.5;
        robot_marker.scale.y = 0.1;
        robot_marker.scale.z = 0.2;
        robot_marker.color.r = 1.0f;
        robot_marker.color.g = 0.0f;
        robot_marker.color.b = 0.0f;
        robot_marker.color.a = 1.0f;
        robot_marker.lifetime = rclcpp::Duration(std::chrono::duration<double>(Param::Ts * 2));
        marker_pub_->publish(robot_marker);

        // 4️⃣ 发布参考路径
        if (!reference_path_.empty())
        {
            nav_msgs::msg::Path path_msg;
            path_msg.header.frame_id = "camera_init";
            path_msg.header.stamp = this->get_clock()->now();
            for (const auto &p : reference_path_)
            {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = path_msg.header;
                pose.pose.position.x = p.x;
                pose.pose.position.y = p.y;
                pose.pose.position.z = 0.0;

                tf2::Quaternion q_path;
                q_path.setRPY(0, 0, p.theta);
                pose.pose.orientation = tf2::toMsg(q_path);
                path_msg.poses.push_back(pose);
            }
            path_pub_->publish(path_msg);
        }

        // 5️⃣ 发布预测轨迹
        visualization_msgs::msg::Marker predict_marker;
        predict_marker.header.frame_id = "camera_init";
        predict_marker.header.stamp = this->get_clock()->now();
        predict_marker.ns = "prediction";
        predict_marker.id = 1;
        predict_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        predict_marker.action = visualization_msgs::msg::Marker::ADD;
        predict_marker.scale.x = 0.05;
        predict_marker.color.r = 0.0f;
        predict_marker.color.g = 1.0f;
        predict_marker.color.b = 0.0f;
        predict_marker.color.a = 1.0f;

        RobotState pred_state = current_state_;
        geometry_msgs::msg::Point pt;
        pt.x = pred_state.x;
        pt.y = pred_state.y;
        pt.z = 0.0;
        predict_marker.points.push_back(pt);

        for (int i = 0; i < Param::N && i < opt_result_.size() / 2; ++i)
        {
            Control u = {opt_result_[2 * i], opt_result_[2 * i + 1]};
            pred_state = predictState(pred_state, u, Param::Ts, this);
            pt.x = pred_state.x;
            pt.y = pred_state.y;
            pt.z = 0.0;
            predict_marker.points.push_back(pt);
        }

        predict_marker.lifetime = rclcpp::Duration(std::chrono::duration<double>(Param::Ts * 2));
        if (!predict_marker.points.empty())
            marker_pub_->publish(predict_marker);
    }


    // 控制主循环
    void controlLoop()
    {
        // 计算最优控制量
        Control u = computeControl();

        // 发布控制命令
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = u.v;
        cmd_vel.angular.z = u.omega;
        cmd_vel_pub_->publish(cmd_vel);

        // 更新内部状态（仅用于预测）
        updateRobotState(u);

        // 发布可视化信息
        // publishVisualization();

        // 打印状态信息
        double radius = (fabs(u.omega) > 1e-3) ? fabs(u.v / u.omega) : INFINITY;
        min_radius = std::min(min_radius, radius);
        RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "MPC Output: v=" << u.v << ", omega=" << u.omega 
            << ", radius=" << radius << ", min_radius=" << min_radius
            << ", path size=" << reference_path_.size());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPCController>());
    rclcpp::shutdown();
    return 0;
}



