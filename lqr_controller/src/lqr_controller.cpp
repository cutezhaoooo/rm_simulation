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

    // this->declare_parameter<double>("max_v",max_v_);
    // this->declare_parameter<double>("min_v_",min_v_);
    // this->declare_parameter<double>("max_v_inc",max_v_inc_);
    // this->declare_parameter<double>("max_w",max_w_);
    // this->declare_parameter<double>("min_w",min_w_);
    // this->declare_parameter<double>("max_w_inc",max_w_inc_);
    // this->declare_parameter<double>("lqr_max_iter",lqr_max_iter_);
    // this->declare_parameter<double>("lqr_eps_iter",lqr_eps_iter_);
    // this->declare_parameter<std::string>("odom_frame",odom_frame_);
    // this->declare_parameter<double>("goal_dist_tol",goal_dist_tol_);
    // this->declare_parameter<double>("rotate_tol",rotate_tol_);
    // this->declare_parameter<double>("lookahead_time",lookahead_time_);
    // this->declare_parameter<double>("min_lookahead_dist",min_lookahead_dist_);
    // this->declare_parameter<double>("max_lookahead_dist",max_lookahead_dist_);
    // this->declare_parameter<double>("rotate_to_heading_angular_vel",rotate_to_heading_angular_vel_);
    // this->declare_parameter<double>("max_angular_accel",max_angular_accel_);
    // this->declare_parameter<double>("rotate_to_heading_min_angle",rotate_to_heading_min_angle_);

    // this->get_parameter("max_v",max_v_);
    // this->get_parameter("min_v",min_v_);
    // this->get_parameter("max_v_inc",max_v_inc_);
    // this->get_parameter("max_w",max_w_);
    // this->get_parameter("min_w",min_w_);
    // this->get_parameter("max_w_inc",max_w_inc_);
    // this->get_parameter("lqr_max_iter",lqr_max_iter_);
    // this->get_parameter("lqr_eps_iter",lqr_eps_iter_);
    // this->get_parameter("odom_frame",odom_frame_);
    // this->get_parameter("goal_dist_tol",goal_dist_tol_);
    // this->get_parameter("rotate_tol",rotate_tol_);
    // this->get_parameter("lookahead_time",lookahead_time_);
    // this->get_parameter("min_lookahead_dist",min_lookahead_dist_);
    // this->get_parameter("max_lookahead_dist",max_lookahead_dist_);
    // this->get_parameter("rotate_to_heading_angular_vel",rotate_to_heading_angular_vel_);
    // this->get_parameter("max_angular_accel",max_angular_accel_);
    // this->get_parameter("rotate_to_heading_min_angle",rotate_to_heading_min_angle_);

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
    min_lookahead_dist_ = 1.0;
    max_lookahead_dist_ = 3.0;
    max_angular_accel_ = 3.0;
    const double d_time = 0.1; // 与定时器周期对应


    // 初始化速度
    last_cmd_vel_.linear.x = 0;
    last_cmd_vel_.linear.y = 0;

    // 初始化 Q 和 R 矩阵
    Q_ = Eigen::Matrix3d::Identity();
    R_ = Eigen::Matrix2d::Identity();

    // 初始化状态标志
    odom_initialized_ = false;
    path_initialized_ = false;

    // 创建订阅者和发布者
    // path在map坐标系下面
    sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
        "/local_path", 
        5, 
        std::bind(&LqrController::pathHandle, this, std::placeholders::_1));

    // odom是odom到base link的坐标变换 如果不在同一个坐标系下面就需要转换一下
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom",
        10,
        std::bind(&LqrController::odomHandle, this, std::placeholders::_1));

    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // 创建控制定时器（例如 10Hz）
    control_timer_ = this->create_wall_timer(
        100ms, 
        std::bind(&LqrController::controlTimerCallback, this));

    lookahead_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/lookaheader_marker",5);

    marker_id_ = 0;

    RCLCPP_INFO(this->get_logger(), "LQR Controller initialized");
}

void LqrController::visualizeLookaheadPoint(const geometry_msgs::msg::PoseStamped& lookahead_pt, 
                                          double lookahead_dist)
{
    visualization_msgs::msg::Marker marker;
    marker.header = lookahead_pt.header;
    marker.ns = "lookahead_point";
    marker.id = marker_id_++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose = lookahead_pt.pose;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    
    marker.color.r = 1.0;  // 红色
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    
    lookahead_marker_pub_->publish(marker);
    
    // 添加文本标记显示距离
    visualization_msgs::msg::Marker text_marker;
    text_marker.header = lookahead_pt.header;
    text_marker.ns = "lookahead_text";
    text_marker.id = marker_id_++;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    
    text_marker.pose = lookahead_pt.pose;
    text_marker.pose.position.z += 0.5;  // 在点上方显示文本
    
    text_marker.scale.z = 0.3;  // 文本大小
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    
    text_marker.text = "L: " + std::to_string(lookahead_dist).substr(0, 4) + "m";
    text_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    
    lookahead_marker_pub_->publish(text_marker);
}

void LqrController::pathHandle(const nav_msgs::msg::Path::SharedPtr path)
{
    // path也在map坐标系下面
    current_path_ = *path;
    path_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "Received path with %zu points", path->poses.size());
}

void LqrController::odomHandle(const nav_msgs::msg::Odometry::SharedPtr odom)
{
    // 这里的odom在map坐标系下面
    current_odom_ = *odom;
    odom_initialized_ = true;
}

double LqrController::getLookAheadDistance(const geometry_msgs::msg::Twist &speed)
{
    double vt = std::hypot(last_cmd_vel_.linear.x,last_cmd_vel_.linear.y); 
    // 可能是负数 不清楚有没有影响
    // double wt = last_cmd_vel_.angular.z;    
    double lookahead_dist = fabs(vt) * lookahead_time_; 
    return std::clamp(lookahead_dist,min_lookahead_dist_,max_lookahead_dist_);
}

double LqrController::getCuspDist(const nav_msgs::msg::Path & path)
{
    // TODO 目前的路径都是平滑过后的 不太可能出现 角点
    for (unsigned i = 1; i < path.poses.size() - 1; i++)
    {
        // double oa_x = path 
    }
    
}

geometry_msgs::msg::Quaternion LqrController::getOrientation(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2)
{
    tf2::Quaternion tf2_quat;

    double yaw = std::atan2(p2.y - p1.y,p2.x - p1.x);
    tf2_quat.setRPY(0.0,0.0,yaw);
    geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(tf2_quat);

    return quat_msg;
}

geometry_msgs::msg::Point LqrController::circleSegmentIntersection(const geometry_msgs::msg::Point & p1,const geometry_msgs::msg::Point & p2,double r)
{
    double x1 = p1.x;
    double x2 = p2.x;
    double y1 = p1.y;
    double y2 = p2.y;

    double dx = x2 - x1;
    double dy = y2 - y1;
    double dr2 = dx * dx + dy * dy;
    double D = x1 * y2 - x2 * y1;

    // Augmentation to only return point within segment
    double d1 = x1 * x1 + y1 * y1;
    double d2 = x2 * x2 + y2 * y2;
    double dd = d2 - d1;

    geometry_msgs::msg::Point p;
    double sqrt_term = std::sqrt(r * r * dr2 - D * D);
    p.x = (D * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
    p.y = (-D * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
    return p;
}


geometry_msgs::msg::PoseStamped LqrController::getLookAheadPoint(
    const double & lookahead_dist,
    const nav_msgs::msg::Path & path)
{
    if (path.poses.empty()) return geometry_msgs::msg::PoseStamped();

    // 当前车辆坐标
    const double rx = current_odom_.pose.pose.position.x;
    const double ry = current_odom_.pose.pose.position.y;

    // 找到第一个距离车辆大于 lookahead_dist 的路径点
    auto goal_pose_it = std::find_if(
        path.poses.begin(), path.poses.end(), [&](const auto & ps) {
            const double dx = ps.pose.position.x - rx;
            const double dy = ps.pose.position.y - ry;
            return std::hypot(dx, dy) >= lookahead_dist;
        });

    geometry_msgs::msg::PoseStamped pose;

    if (goal_pose_it == path.poses.end()) {
        // 使用最后一个点
        pose = path.poses.back();
        pose.pose.orientation = getOrientation(
            std::prev(path.poses.end(), 2)->pose.position,
            std::prev(path.poses.end())->pose.position);
        RCLCPP_INFO(this->get_logger(), "终点");
    } else if (goal_pose_it == path.poses.begin()) {
        pose = *goal_pose_it;
        pose.pose.orientation = getOrientation(
            path.poses.front().pose.position,
            std::next(path.poses.begin())->pose.position);
        RCLCPP_INFO(this->get_logger(), "起点");
    } else {
        // 插值
        RCLCPP_INFO(this->get_logger(), "插值");
        auto prev_pose_it = std::prev(goal_pose_it);
        pose.pose.position = circleSegmentIntersection(
            prev_pose_it->pose.position,
            goal_pose_it->pose.position,
            lookahead_dist);

        pose.header = prev_pose_it->header;
        pose.pose.orientation = getOrientation(
            prev_pose_it->pose.position,
            goal_pose_it->pose.position);
    }

    return pose;
}


// Calculate distance between the 2 nodes.
double LqrController::dist(const std::pair<double, double>& node1, const std::pair<double, double>& node2)
{
  return std::hypot(node1.first - node2.first, node1.second - node2.second);
}

double LqrController::dist(const geometry_msgs::msg::PoseStamped& node1, const geometry_msgs::msg::PoseStamped& node2)
{
  return std::hypot(node1.pose.position.x - node2.pose.position.x, node1.pose.position.y - node2.pose.position.y);
}

bool LqrController::shouldRotateToGoal(const geometry_msgs::msg::Pose& cur, 
                                      const geometry_msgs::msg::Pose& goal)
{
    double dx = goal.position.x - cur.position.x;
    double dy = goal.position.y - cur.position.y;
    double distance = std::hypot(dx, dy);
    
    // RCLCPP_INFO(this->get_logger(), "Distance to lookahead: %.2f", distance);
    return distance < goal_dist_tol_;
}

double LqrController::rotateToHeading(const double & angle_to_path,const geometry_msgs::msg::Twist & curr_speed)
{

    const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
    double angular_vel = sign * rotate_to_heading_angular_vel_;

    // HACK 看看怎么设置好 
    const double & dt = 0.1;
    const double min_feasible_angular_speed = curr_speed.angular.z - max_angular_accel_ * dt;
    const double max_feasible_angular_speed = curr_speed.angular.z + max_angular_accel_ * dt;
    // RCLCPP_INFO(this->get_logger(),"angular_vel : %.2f",angular_vel);
    // RCLCPP_INFO(this->get_logger(),"min_feasible_angular_speed : %.2f",min_feasible_angular_speed);
    // RCLCPP_INFO(this->get_logger(),"max_feasible_angular_speed : %.2f",max_feasible_angular_speed);
    angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);

    return angular_vel;
}

bool LqrController::shouldRotateToPath(const geometry_msgs::msg::PoseStamped & target_pose,double & angle_to_path,double & sign)
{
    // 计算目标点方向角
    angle_to_path = atan2(target_pose.pose.position.y,target_pose.pose.position.x);
    // 如果在倒车
    if (sign<0.0)
    {
        angle_to_path = angles::normalize_angle(angle_to_path + M_PI);
    }

    // 角度与目标的角度相差比较多
    return std::abs(angle_to_path) > rotate_to_heading_min_angle_; 
}

double LqrController::computePathCurvatureAtLookahead(
    const geometry_msgs::msg::PoseStamped & lookahead_point,
    const nav_msgs::msg::Path & path,
    double & out_theta)
{
    // 防护：路径长度
    if (path.poses.size() < 3) {
        // 用 lookahead_point 的朝向作为 theta（若有），曲率为 0
        out_theta = tf2::getYaw(lookahead_point.pose.orientation);
        return 0.0;
    }

    // 找到与 lookahead_point 距离最近的路径索引
    size_t closest_idx = 0;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < path.poses.size(); ++i) {
        double dx = path.poses[i].pose.position.x - lookahead_point.pose.position.x;
        double dy = path.poses[i].pose.position.y - lookahead_point.pose.position.y;
        double d2 = dx * dx + dy * dy;
        if (d2 < best_d2) {
        best_d2 = d2;
        closest_idx = i;
        }
    }

    // 取前后点索引（保证存在）
    size_t i_prev = (closest_idx == 0) ? 0 : closest_idx - 1;
    size_t i_next = (closest_idx + 1 >= path.poses.size()) ? path.poses.size() - 1 : closest_idx + 1;

    // 若刚好在头尾且无法找到三点，退化为使用邻近两点计算 theta，curvature = 0
    if (i_prev == i_next) {
        out_theta = tf2::getYaw(path.poses[closest_idx].pose.orientation);
        return 0.0;
    }

    // 三点 A (i_prev), B (closest_idx), C (i_next)
    const auto &A = path.poses[i_prev].pose.position;
    const auto &B = path.poses[closest_idx].pose.position;
    const auto &C = path.poses[i_next].pose.position;

    double ax = A.x, ay = A.y;
    double bx = B.x, by = B.y;
    double cx = C.x, cy = C.y;

    // 边长
    double a = std::hypot(bx - cx, by - cy); // |BC|
    double b = std::hypot(cx - ax, cy - ay); // |CA|
    double c = std::hypot(ax - bx, ay - by); // |AB|

    // 向量叉积（A->B) x (A->C)
    double cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);

    // 防止退化三角形
    double denom = a * b * c;
    const double EPS = 1e-9;
    if (denom < EPS) {
        // 三点近似共线或几何退化 -> 视为直线
        // 取 B->C 的切线角作为 theta（若实在退化则 fallback lookahead orientation）
        out_theta = std::atan2(cy - by, cx - bx);
        if (!std::isfinite(out_theta)) {
        out_theta = tf2::getYaw(lookahead_point.pose.orientation);
        }
        return 0.0;
    }

    // curvature with sign
    double kappa = (2.0 * cross) / denom; // signed curvature

    // 计算 theta：用 B->C 切线方向更能反映前方路径方向
    out_theta = std::atan2(cy - by, cx - bx);

    return kappa;
}

double LqrController::regularizeAngle(double angle)
{
    return angle - 2.0 * M_PI * std::floor((angle + M_PI) / (2.0 * M_PI));
}

Eigen::Vector2d LqrController::_lqrControl(Eigen::Vector3d s,Eigen::Vector3d s_d,Eigen::Vector2d u_r,double d_time)
{
    Eigen::Vector2d u;
    Eigen::Vector3d e(s - s_d);
    e[2] = regularizeAngle(e[2]);


    // state equation on error
    Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
    A(0, 2) = -u_r[0] * sin(s_d[2]) * d_time;
    A(1, 2) = u_r[0] * cos(s_d[2]) * d_time;



    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(3, 2);
    B(0, 0) = cos(s_d[2]) * d_time;
    B(1, 0) = sin(s_d[2]) * d_time;
    B(2, 1) = d_time;



    // discrete iteration Ricatti equation
    Eigen::Matrix3d P, P_;
    P = Q_;
    for (int i = 0; i < lqr_max_iter_; ++i)
    {
        Eigen::Matrix2d temp = R_ + B.transpose() * P * B;
        P_ = Q_ + A.transpose() * P * A - A.transpose() * P * B * temp.inverse() * B.transpose() * P * A;
        if ((P - P_).array().abs().maxCoeff() < lqr_eps_iter_)
        break;
        P = P_;
    }



    // feedback
    Eigen::MatrixXd K = -(R_ + B.transpose() * P_ * B).inverse() * B.transpose() * P_ * A;

    // std::cout<<"check Q_:"<<Q_<<std::endl;
    // std::cout<<"check R_:"<<R_<<std::endl;
    // std::cout<<"check B:"<<B<<std::endl;
    // std::cout<<"check P_:"<<P_<<std::endl;
    // std::cout<<"check A:"<<A<<std::endl;



    u = u_r + K * e;

    // std::cout<<"check ur:"<<u_r<<std::endl;
    // std::cout<<"check e:"<<e<<std::endl;
    // std::cout<<"check K:"<<K<<std::endl;
    // std::cout<<"check u:"<<u<<std::endl;

    return u;

}

double LqrController::linearRegularization(double v,double v_d)
{
    double v_inc = v_d - v;
    if (std::fabs(v_inc) > max_v_inc_)
        v_inc = std::copysign(max_v_inc_, v_inc);

    double v_cmd = v + v_inc;
    if (std::fabs(v_cmd) > max_v_)
        v_cmd = std::copysign(max_v_, v_cmd);
    else if (std::fabs(v_cmd) < min_v_)
        v_cmd = 0;

    return v_cmd;
}

double LqrController::angularRegularization(double wt, double w_d)
{
  if (std::fabs(w_d) > max_w_)
    w_d = std::copysign(max_w_, w_d);

  double w = wt;
  double w_inc = w_d - w;

  if (std::fabs(w_inc) > max_w_inc_)
    w_inc = std::copysign(max_w_inc_, w_inc);

  double w_cmd = w + w_inc;
  if (std::fabs(w_cmd) > max_w_)
    w_cmd = std::copysign(max_w_, w_cmd);
  else if (std::fabs(w_cmd) < min_w_)
    w_cmd = std::copysign(min_w_, w_cmd);

  return w_cmd;
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

    // RCLCPP_INFO(this->get_logger(),"controlTimerCallback");

    // 在这里实现 LQR 控制逻辑
    // 1. 获取当前机器人状态
    // 2. 计算前瞻点
    double vt = std::hypot(last_cmd_vel_.linear.x,last_cmd_vel_.linear.y);
    double wt = last_cmd_vel_.angular.z;
    double L = getLookAheadDistance(last_cmd_vel_);

    // Cusp check
    // const double dist_to_cusp = 
    auto lookahead_point = getLookAheadPoint(L,current_path_);

    // 可视化前瞻点
    visualizeLookaheadPoint(lookahead_point,L);

    // 当前的角度
    double theta = tf2::getYaw(current_odom_.pose.pose.orientation);

    // 判断是否需要倒车
    double sign = 1.0;
    sign = lookahead_point.pose.position.x >= 0.0 ? 1.0 : -1.0;
    

    // 查看是否需要转到终点
    double angle_to_heading;
    if(shouldRotateToGoal(current_odom_.pose.pose,lookahead_point.pose))
    {
        // 快到前瞻点了 也就是快到路径的终点了
        double angle_to_goal = tf2::getYaw(current_path_.poses.back().pose.orientation);
        // rotateToHeading(linear_vel)
        double linear_vel = 0.0;
        double angular_vel = rotateToHeading(angle_to_goal,last_cmd_vel_);
        cmd_vel.linear.x =  linear_vel;
        cmd_vel.angular.z = angular_vel;
        RCLCPP_INFO(this->get_logger(),"shouldRotateToGoal");
    }
    else if (shouldRotateToPath(lookahead_point,angle_to_heading,sign))
    {
        RCLCPP_INFO(this->get_logger(),"shouldRotateTo  Path angle_to_heading:%.2f",angle_to_heading);
        double linear_vel = 0.0;
        double angular_vel = rotateToHeading(angle_to_heading,last_cmd_vel_);
        cmd_vel.linear.x =  linear_vel;
        cmd_vel.angular.z = angular_vel;
    }else
    {
        // RCLCPP_INFO(this->get_logger(),"else  ");
        // lqr 控制
        Eigen::Vector3d s(current_odom_.pose.pose.position.x,current_odom_.pose.pose.position.y,theta); // 当前的状态
        // 前瞻点的曲率
        double theta_d = 0.0;
        double kappa_d = computePathCurvatureAtLookahead(lookahead_point, current_path_, theta_d);
        Eigen::Vector3d s_d(lookahead_point.pose.position.x,lookahead_point.pose.position.y,theta_d);    // 期望的状态
        Eigen::Vector2d u_r(vt,vt * kappa_d);                                                            // 输出
        Eigen::Vector2d u = _lqrControl(s,s_d,u_r,d_time);

        // vt
        cmd_vel.linear.x = linearRegularization(vt,u[0]);
        cmd_vel.angular.z = angularRegularization(wt,u[1]);

        if((std::fabs(cmd_vel.linear.x)> 100000)||(std::fabs(cmd_vel.angular.z)> 100000))
        {
            RCLCPP_WARN(this->get_logger(),"Control command abnormal, stop moving");
            // return false;
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
        }        
    }
    
    pub_cmd_vel_->publish(cmd_vel);
    
    last_cmd_vel_ = cmd_vel;

    RCLCPP_DEBUG(this->get_logger(), "Executing LQR control");
    
}