#ifndef _KINODYNAMIC_ASTAR_H
#define _KINODYNAMIC_ASTAR_H

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Eigen>
#include <vector>
#include <iostream>
#include <boost/functional/hash.hpp>
#include <map>

// 编译期常量 值在编译期确定 
constexpr char IN_CLOSE_SET = 'a';
constexpr char IN_OPEN_SET = 'b';
constexpr char NOT_EXPAND = 'c';
constexpr double inf = 1 >> 30;


namespace fast_planner
{

class PathNode {
public:
    Eigen::Vector3i index;
    Eigen::Matrix<double, 6, 1> state;
    double g_score, f_score;
    Eigen::Vector3d input;
    double duration;
    double time;
    int time_idx;
    PathNode* parent;
    char node_state;

    PathNode() {
        parent = nullptr;
        node_state = NOT_EXPAND;
    }
    ~PathNode(){};
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


class KinodynamicAstar
{
private:
    

};


}


#endif