#ifndef LQR_CONTROLLER
#define LQR_CONTROLLER

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"

#include "eigen3/Eigen/Dense"

namespace lqr
{

class lqr_controller
{
public:

    lqr_controller(double goal_dist_tol,double rotate_tol, double max_v, double min_v, double max_v_inc, 
                 double max_w, double min_w, double max_w_inc, Eigen::Matrix3d Q, Eigen::Matrix2d R,
                 int lqr_max_iter,double lqr_eps_iter, double lookahead_time, double min_lookahead_dist,
                 double max_lookahead_dist);

    ~lqr_controller();


private:

    double max_v_,min_v_,max_v_inc_;
    double max_w_,min_w_,max_w_inc_;

    Eigen::Matrix3d Q_;     // 状态误差矩阵
    Eigen::Matrix2d R_;     // 控制误差矩阵

    int lqr_max_iter_;   // maximum iteration for ricatti solution
    double lqr_eps_iter_;    // iteration ending threshold

    
    std::string odom_frame_;

    double goal_dist_tol_;
    double rotate_tol_;

    double lookahead_time_;      // lookahead time gain
    double min_lookahead_dist_;  // minimum lookahead distance
    double max_lookahead_dist_;  // maximum lookahead distance

};

}

#endif