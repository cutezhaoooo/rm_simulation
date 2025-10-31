// lqr_controller.cpp
// ROS2 Humble - single-file LqrController with NaN protections, robust lookahead interpolation,
// stable LQR solves, and safer sign logic.

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <Eigen/Dense>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <limits>
#include <algorithm>

using namespace std::chrono_literals;

class LqrController : public rclcpp::Node
{
public:
  LqrController();
  ~LqrController() = default;

private:
  // Core callbacks
  void initializer();
  void pathHandle(const nav_msgs::msg::Path::SharedPtr path);
  void odomHandle(const nav_msgs::msg::Odometry::SharedPtr odom);
  void controlTimerCallback();

  // Utility
  double getLookAheadDistance(const geometry_msgs::msg::Twist &speed);
  bool circleSegmentIntersection(const geometry_msgs::msg::Point & p1,
                                 const geometry_msgs::msg::Point & p2,
                                 double r,
                                 geometry_msgs::msg::Point & out_p);
  geometry_msgs::msg::PoseStamped getLookAheadPoint(const double & lookahead_dist,
                                                    const nav_msgs::msg::Path & path);
  geometry_msgs::msg::Quaternion getOrientation(const geometry_msgs::msg::Point & p1,
                                                const geometry_msgs::msg::Point & p2);
  double computePathCurvatureAtLookahead(const geometry_msgs::msg::PoseStamped & lookahead_point,
                                         const nav_msgs::msg::Path & path,
                                         double & out_theta);
  double regularizeAngle(double angle);
  Eigen::Vector2d _lqrControl(const Eigen::Vector3d &s,
                              const Eigen::Vector3d &s_d,
                              const Eigen::Vector2d &u_r,
                              double d_time);
  double linearRegularization(double v, double v_d);
  double angularRegularization(double wt, double w_d);
  bool shouldRotateToGoal(const geometry_msgs::msg::Pose & cur, const geometry_msgs::msg::Pose & goal);
  double rotateToHeading(const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed);
  bool shouldRotateToPath(const geometry_msgs::msg::PoseStamped & target_pose, double & angle_to_path, double & sign);

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_marker_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // State
  nav_msgs::msg::Path current_path_;
  nav_msgs::msg::Odometry current_odom_;
  geometry_msgs::msg::Twist last_cmd_vel_;
  geometry_msgs::msg::Twist cmd_vel_;

  // Params
  double max_v_, min_v_, max_v_inc_;
  double max_w_, min_w_, max_w_inc_;
  int lqr_max_iter_;
  double lqr_eps_iter_;
  std::string odom_frame_;
  double goal_dist_tol_;
  double rotate_tol_;
  double lookahead_time_;
  double min_lookahead_dist_, max_lookahead_dist_;
  double rotate_to_heading_angular_vel_;
  double max_angular_accel_;
  double rotate_to_heading_min_angle_;
  double d_time;

  // LQR matrices
  Eigen::Matrix3d Q_;
  Eigen::Matrix2d R_;

  // flags
  bool odom_initialized_;
  bool path_initialized_;
  int marker_id_;

  // timers
  double timer_period_;
};

// ------------------ Implementation ------------------

LqrController::LqrController() : Node("lqr_controller")
{
  initializer();
}

void LqrController::initializer()
{
  // Default params (you can expose them with declare_parameter if desired)
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
  rotate_to_heading_angular_vel_ = 1.5;
  rotate_to_heading_min_angle_ = 0.2;

  d_time = 0.1;
  timer_period_ = 0.1;

  last_cmd_vel_.linear.x = 0.0;
  last_cmd_vel_.linear.y = 0.0;
  last_cmd_vel_.angular.z = 0.0;

  Q_ = Eigen::Matrix3d::Identity();
  R_ = Eigen::Matrix2d::Identity();

  odom_initialized_ = false;
  path_initialized_ = false;
  marker_id_ = 0;

  // subs / pubs
  sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
    "/local_path", rclcpp::QoS(5),
    std::bind(&LqrController::pathHandle, this, std::placeholders::_1));

  sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::QoS(10),
    std::bind(&LqrController::odomHandle, this, std::placeholders::_1));

  pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  lookahead_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/lookaheader_marker", 5);

  control_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(timer_period_),
    std::bind(&LqrController::controlTimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "LQR Controller initialized");
}

void LqrController::pathHandle(const nav_msgs::msg::Path::SharedPtr path)
{
  current_path_ = *path;
  path_initialized_ = true;
  // RCLCPP_INFO(this->get_logger(), "Received path with %zu points, frame_id=%s", path->poses.size(),
  //             path->header.frame_id.c_str());
}

void LqrController::odomHandle(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  current_odom_ = *odom;
  odom_initialized_ = true;
}

double LqrController::getLookAheadDistance(const geometry_msgs::msg::Twist &speed)
{
  double vt = std::hypot(last_cmd_vel_.linear.x, last_cmd_vel_.linear.y);
  double lookahead_dist = std::fabs(vt) * lookahead_time_;
  return std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
}

geometry_msgs::msg::Quaternion LqrController::getOrientation(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2)
{
  tf2::Quaternion tf2_quat;
  double yaw = std::atan2(p2.y - p1.y, p2.x - p1.x);
  tf2_quat.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(tf2_quat);
  return quat_msg;
}

bool LqrController::circleSegmentIntersection(const geometry_msgs::msg::Point & p1,
                                              const geometry_msgs::msg::Point & p2,
                                              double r,
                                              geometry_msgs::msg::Point & out_p)
{
  // 计算线段与圆相交
  double x1 = p1.x, x2 = p2.x;
  double y1 = p1.y, y2 = p2.y;

  double dx = x2 - x1;
  double dy = y2 - y1;
  double dr2 = dx * dx + dy * dy;
  double D = x1 * y2 - x2 * y1;

  const double EPS = 1e-10;
  double discriminant = r * r * dr2 - D * D;
  if (discriminant < -EPS || dr2 < EPS) {
    // no intersection or degenerate segment
    return false;
  }
  if (discriminant < 0.0) discriminant = 0.0;

  double sqrt_term = std::sqrt(discriminant);

  geometry_msgs::msg::Point cand1, cand2;
  cand1.x = (D * dy + dx * sqrt_term) / dr2;
  cand1.y = (-D * dx + dy * sqrt_term) / dr2;

  cand2.x = (D * dy - dx * sqrt_term) / dr2;
  cand2.y = (-D * dx - dy * sqrt_term) / dr2;

  auto on_segment = [&](const geometry_msgs::msg::Point & p) -> bool {
    double minx = std::min(x1, x2) - 1e-6, maxx = std::max(x1, x2) + 1e-6;
    double miny = std::min(y1, y2) - 1e-6, maxy = std::max(y1, y2) + 1e-6;
    return (p.x >= minx && p.x <= maxx && p.y >= miny && p.y <= maxy);
  };

  if (on_segment(cand1)) { out_p = cand1; return true; }
  if (on_segment(cand2)) { out_p = cand2; return true; }

  return false;
}

geometry_msgs::msg::PoseStamped LqrController::getLookAheadPoint(
  const double & lookahead_dist,
  const nav_msgs::msg::Path & path)
{
  geometry_msgs::msg::PoseStamped pose;
  if (path.poses.empty()) return pose;

  const double rx = current_odom_.pose.pose.position.x;
  const double ry = current_odom_.pose.pose.position.y;

  auto goal_pose_it = std::find_if(
    path.poses.begin(), path.poses.end(), [&](const auto & ps) {
      const double dx = ps.pose.position.x - rx;
      const double dy = ps.pose.position.y - ry;
      return std::hypot(dx, dy) >= lookahead_dist;
    });

  if (goal_pose_it == path.poses.end()) {
    pose = path.poses.back();
    if (path.poses.size() >= 2) {
      pose.pose.orientation = getOrientation(std::prev(path.poses.end(), 2)->pose.position,
                                            std::prev(path.poses.end())->pose.position);
    }
    // RCLCPP_INFO(this->get_logger(), "终点");
    return pose;
  } else if (goal_pose_it == path.poses.begin()) {
    pose = *goal_pose_it;
    if (path.poses.size() >= 2) {
      pose.pose.orientation = getOrientation(path.poses.front().pose.position,
                                            std::next(path.poses.begin())->pose.position);
    }
    // RCLCPP_INFO(this->get_logger(), "起点");
    return pose;
  } else {
    auto prev_pose_it = std::prev(goal_pose_it);
    geometry_msgs::msg::Point inter;
    bool ok = circleSegmentIntersection(prev_pose_it->pose.position,
                                        goal_pose_it->pose.position,
                                        lookahead_dist,
                                        inter);
    if (ok && std::isfinite(inter.x) && std::isfinite(inter.y)) {
      // x y均为有限值
      pose.pose.position = inter;
      pose.header = prev_pose_it->header;
      pose.pose.orientation = getOrientation(prev_pose_it->pose.position, goal_pose_it->pose.position);
      // RCLCPP_INFO(this->get_logger(), "插值(交点)");
    } else {
      // 交点失败
      // fallback linear interpolation along segment
      double dx = goal_pose_it->pose.position.x - prev_pose_it->pose.position.x;
      double dy = goal_pose_it->pose.position.y - prev_pose_it->pose.position.y;

      double seg_len = std::hypot(dx, dy);
      if (seg_len < 1e-6) {
        pose = *prev_pose_it;
      } else {
        double dist_prev_to_robot = std::hypot(prev_pose_it->pose.position.x - rx,
                                               prev_pose_it->pose.position.y - ry);
        double need = lookahead_dist - dist_prev_to_robot;
        double t = std::clamp(need / seg_len, 0.0, 1.0);
        pose.pose.position.x = prev_pose_it->pose.position.x + t * dx;
        pose.pose.position.y = prev_pose_it->pose.position.y + t * dy;
        pose.header = prev_pose_it->header;
        pose.pose.orientation = getOrientation(prev_pose_it->pose.position, goal_pose_it->pose.position);
      }
      RCLCPP_WARN(this->get_logger(), "插值(后备)，交点不存在或数值不稳定");
    }
    return pose;
  }
}

double LqrController::computePathCurvatureAtLookahead(
  const geometry_msgs::msg::PoseStamped & lookahead_point,
  const nav_msgs::msg::Path & path,
  double & out_theta)
{
  if (path.poses.size() < 3) {
    out_theta = tf2::getYaw(lookahead_point.pose.orientation);
    return 0.0;
  }

  size_t closest_idx = 0;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    double dx = path.poses[i].pose.position.x - lookahead_point.pose.position.x;
    double dy = path.poses[i].pose.position.y - lookahead_point.pose.position.y;
    double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) { best_d2 = d2; closest_idx = i; }
  }

  size_t i_prev = (closest_idx == 0) ? 0 : closest_idx - 1;
  size_t i_next = (closest_idx + 1 >= path.poses.size()) ? path.poses.size() - 1 : closest_idx + 1;

  if (i_prev == i_next) {
    out_theta = tf2::getYaw(path.poses[closest_idx].pose.orientation);
    return 0.0;
  }

  const auto &A = path.poses[i_prev].pose.position;
  const auto &B = path.poses[closest_idx].pose.position;
  const auto &C = path.poses[i_next].pose.position;

  double ax = A.x, ay = A.y;
  double bx = B.x, by = B.y;
  double cx = C.x, cy = C.y;

  double a = std::hypot(bx - cx, by - cy);
  double b = std::hypot(cx - ax, cy - ay);
  double c = std::hypot(ax - bx, ay - by);

  double cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);

  double denom = a * b * c;
  const double EPS = 1e-9;
  if (denom < EPS) {
    out_theta = std::atan2(cy - by, cx - bx);
    if (!std::isfinite(out_theta)) {
      out_theta = tf2::getYaw(lookahead_point.pose.orientation);
    }
    return 0.0;
  }

  double kappa = (2.0 * cross) / denom;
  out_theta = std::atan2(cy - by, cx - bx);

  return kappa;
}

double LqrController::regularizeAngle(double angle)
{
  return angle - 2.0 * M_PI * std::floor((angle + M_PI) / (2.0 * M_PI));
}

Eigen::Vector2d LqrController::_lqrControl(
                          const Eigen::Vector3d &s,
                          const Eigen::Vector3d &s_d,
                          const Eigen::Vector2d &u_r,
                          double d_time)
{
  Eigen::Vector2d u;
  Eigen::Vector3d e = (s - s_d);
  e[2] = regularizeAngle(e[2]);

  Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
  A(0, 2) = -u_r[0] * std::sin(s_d[2]) * d_time;
  A(1, 2) = u_r[0] * std::cos(s_d[2]) * d_time;

  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(3, 2);
  B(0, 0) = std::cos(s_d[2]) * d_time;
  B(1, 0) = std::sin(s_d[2]) * d_time;
  B(2, 1) = d_time;

  Eigen::Matrix3d P, P_;
  P = Q_;
  for (int i = 0; i < lqr_max_iter_; ++i) {
    Eigen::Matrix2d temp = R_ + B.transpose() * P * B;
    Eigen::FullPivLU<Eigen::Matrix2d> lu(temp);
    if (!lu.isInvertible()) {
      temp += Eigen::Matrix2d::Identity() * 1e-6;
    }
    Eigen::Matrix2d inv_temp = temp.ldlt().solve(Eigen::Matrix2d::Identity());
    P_ = Q_ + A.transpose() * P * A - A.transpose() * P * B * inv_temp * B.transpose() * P * A;
    if ((P - P_).array().abs().maxCoeff() < lqr_eps_iter_) break;
    P = P_;
  }

  Eigen::Matrix2d denom = (R_ + B.transpose() * P_ * B);
  Eigen::FullPivLU<Eigen::Matrix2d> lu2(denom);
  if (!lu2.isInvertible()) {
    denom += Eigen::Matrix2d::Identity() * 1e-6;
  }
  Eigen::MatrixXd K = -denom.ldlt().solve(B.transpose() * P_ * A);

  u = u_r + K * e;
  return u;
}

double LqrController::linearRegularization(double v, double v_d)
{
  double v_inc = v_d - v;
  if (std::fabs(v_inc) > max_v_inc_) v_inc = std::copysign(max_v_inc_, v_inc);
  double v_cmd = v + v_inc;
  if (std::fabs(v_cmd) > max_v_) v_cmd = std::copysign(max_v_, v_cmd);
  else if (std::fabs(v_cmd) < min_v_) v_cmd = 0;
  return v_cmd;
}

double LqrController::angularRegularization(double wt, double w_d)
{
  if (std::fabs(w_d) > max_w_) w_d = std::copysign(max_w_, w_d);
  double w = wt;
  double w_inc = w_d - w;
  if (std::fabs(w_inc) > max_w_inc_) w_inc = std::copysign(max_w_inc_, w_inc);
  double w_cmd = w + w_inc;
  if (std::fabs(w_cmd) > max_w_) w_cmd = std::copysign(max_w_, w_cmd);
  else if (std::fabs(w_cmd) < min_w_) w_cmd = std::copysign(min_w_, w_cmd);
  return w_cmd;
}

bool LqrController::shouldRotateToGoal(const geometry_msgs::msg::Pose & cur, const geometry_msgs::msg::Pose & goal)
{
  double dx = goal.position.x - cur.position.x;
  double dy = goal.position.y - cur.position.y;
  double distance = std::hypot(dx, dy);
  return distance < goal_dist_tol_;
}

double LqrController::rotateToHeading(const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed)
{
  const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
  double angular_vel = sign * rotate_to_heading_angular_vel_;

  const double dt = d_time;
  const double min_feasible_angular_speed = curr_speed.angular.z - max_angular_accel_ * dt;
  const double max_feasible_angular_speed = curr_speed.angular.z + max_angular_accel_ * dt;
  angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);
  return angular_vel;
}

// NOTE: This function expects target_pose to be in robot-local coordinates if used that way.
// We'll not rely on it in control loop; keep for completeness.
bool LqrController::shouldRotateToPath(const geometry_msgs::msg::PoseStamped & target_pose, double & angle_to_path, double & sign)
{
  // If target_pose is given in robot frame, atan2(y,x) is correct.
  angle_to_path = std::atan2(target_pose.pose.position.y, target_pose.pose.position.x);
  if (sign < 0.0) {
    angle_to_path = regularizeAngle(angle_to_path + M_PI);
  }
  return std::abs(angle_to_path) > rotate_to_heading_min_angle_;
}

void LqrController::controlTimerCallback()
{
  if (!odom_initialized_ || !path_initialized_) {
    RCLCPP_DEBUG(this->get_logger(), "Waiting for odometry and path data...");
    return;
  }
  if (current_path_.poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "Path is empty");
    return;
  }

  double vt = std::hypot(last_cmd_vel_.linear.x, last_cmd_vel_.linear.y);
  double wt = last_cmd_vel_.angular.z;
  double L = getLookAheadDistance(last_cmd_vel_);

  RCLCPP_INFO(this->get_logger(),"look ahead distance :%.2f",L);

  auto lookahead_point = getLookAheadPoint(L, current_path_);

  // visualize lookahead
  {
    visualization_msgs::msg::Marker marker;
    marker.header = lookahead_point.header;
    marker.ns = "lookahead_point";
    marker.id = marker_id_++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = lookahead_point.pose;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.3;
    marker.color.r = 1.0; marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    lookahead_marker_pub_->publish(marker);

    visualization_msgs::msg::Marker text_marker;
    text_marker.header = lookahead_point.header;
    text_marker.ns = "lookahead_text";
    text_marker.id = marker_id_++;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.pose = lookahead_point.pose;
    text_marker.pose.position.z += 0.5;
    text_marker.scale.z = 0.3;
    text_marker.color.r = 1.0; text_marker.color.g = 1.0; text_marker.color.b = 1.0; text_marker.color.a = 1.0;
    text_marker.text = "L: " + std::to_string(L).substr(0, 4) + "m";
    text_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    lookahead_marker_pub_->publish(text_marker);
  }

  // check finite
  if (!std::isfinite(lookahead_point.pose.position.x) || !std::isfinite(lookahead_point.pose.position.y)) {
    RCLCPP_ERROR(this->get_logger(), "lookahead_point contains non-finite values");
    geometry_msgs::msg::Twist safe_stop;
    pub_cmd_vel_->publish(safe_stop);
    return;
  }

  // robot's yaw
  double theta = tf2::getYaw(current_odom_.pose.pose.orientation);

  // compute sign based on robot-local projection (front/back)
  double dx = lookahead_point.pose.position.x - current_odom_.pose.pose.position.x;
  double dy = lookahead_point.pose.position.y - current_odom_.pose.pose.position.y;
  double vx_local = std::cos(theta) * dx + std::sin(theta) * dy;
  double sign = (vx_local >= 0.0) ? 1.0 : -1.0;

  // choose behavior: rotate to goal if very close
  if (shouldRotateToGoal(current_odom_.pose.pose, lookahead_point.pose)) {
    double angle_to_goal = tf2::getYaw(current_path_.poses.back().pose.orientation);
    double linear_vel = 0.0;
    double angular_vel = rotateToHeading(angle_to_goal, last_cmd_vel_);
    cmd_vel_.linear.x = linear_vel;
    cmd_vel_.angular.z = angular_vel;
    RCLCPP_INFO(this->get_logger(), "shouldRotateToGoal");
  } else {
    // check orientation difference to path (we compute angle_to_path in robot frame)
    // compute relative vector in robot frame for heading check
    double rel_x = dx * std::cos(-theta) - dy * std::sin(-theta);
    double rel_y = dx * std::sin(-theta) + dy * std::cos(-theta);
    double angle_to_path = std::atan2(rel_y, rel_x);

    if (std::fabs(angle_to_path) > rotate_to_heading_min_angle_) {
      // rotate in place toward path heading
      double linear_vel = 0.0;
      double angular_vel = rotateToHeading(angle_to_path, last_cmd_vel_);
      cmd_vel_.linear.x = linear_vel;
      cmd_vel_.angular.z = angular_vel;
      RCLCPP_INFO(this->get_logger(), "shouldRotateToPath angle_to_path: %.3f", angle_to_path);
    } else {
      // LQR control
      Eigen::Vector3d s(current_odom_.pose.pose.position.x,
                        current_odom_.pose.pose.position.y,
                        theta);
      double theta_d = 0.0;
      double kappa_d = computePathCurvatureAtLookahead(lookahead_point, current_path_, theta_d);
      Eigen::Vector3d s_d(lookahead_point.pose.position.x,
                          lookahead_point.pose.position.y,
                          theta_d);
      Eigen::Vector2d u_r(vt, vt * kappa_d);
      Eigen::Vector2d u = _lqrControl(s, s_d, u_r, d_time);

      cmd_vel_.linear.x = linearRegularization(vt, u[0]);
      cmd_vel_.angular.z = angularRegularization(wt, u[1]);

      if (!std::isfinite(cmd_vel_.linear.x) || !std::isfinite(cmd_vel_.angular.z) ||
          std::fabs(cmd_vel_.linear.x) > 1e6 || std::fabs(cmd_vel_.angular.z) > 1e6) {
        RCLCPP_WARN(this->get_logger(), "Control command abnormal -> stop");
        cmd_vel_.linear.x = 0.0;
        cmd_vel_.angular.z = 0.0;
      }
    }
  }

  // round to 2 decimals for clarity
  cmd_vel_.linear.x = std::roundf(cmd_vel_.linear.x * 100.0f) / 100.0f;
  cmd_vel_.angular.z = std::roundf(cmd_vel_.angular.z * 100.0f) / 100.0f;

  pub_cmd_vel_->publish(cmd_vel_);
  RCLCPP_INFO(this->get_logger(), "linear x :%.2f, angular z :%.2f", cmd_vel_.linear.x, cmd_vel_.angular.z);

  last_cmd_vel_ = cmd_vel_;
}

// ------------------ main ------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LqrController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
