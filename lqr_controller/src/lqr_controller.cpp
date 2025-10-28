#include <tf2/utils.h>

#include "lqr_controller/lqr_controller.hpp"

namespace lqr
{

lqr_controller::lqr_controller(double goal_dist_tol,double rotate_tol, double max_v, double min_v, double max_v_inc, 
                               double max_w, double min_w, double max_w_inc, Eigen::Matrix3d Q, Eigen::Matrix2d R,
                               int lqr_max_iter,double lqr_eps_iter, double lookahead_time, double min_lookahead_dist,
                               double max_lookahead_dist)
    : odom_frame_("odom")
    , goal_dist_tol_(goal_dist_tol)
    , rotate_tol_(rotate_tol)
    , max_v_(max_v)
    , min_v_(min_v)
    , max_v_inc_(max_v_inc)
    , max_w_(max_w)
    , min_w_(min_w)
    , max_w_inc_(max_w_inc)
    , Q_(Q)
    , R_(R)
    , lqr_max_iter_(lqr_max_iter)
    , lqr_eps_iter_(lqr_eps_iter)
    , lookahead_time_(lookahead_time)
    , min_lookahead_dist_(min_lookahead_dist)
    , max_lookahead_dist_(max_lookahead_dist)
    {
        // RCLCPP_INFO(this->get_logger(),"lqr controller init ");
    }

}


