#pragma once

#include "pathfinding.hpp"
#include <Eigen/Dense>

Eigen::Vector3d joint_angle_to_cart_loc(const double angles[4]);

class Robot {
private:
    PathFinding pathfinding;
    Eigen::Vector3d cart_pos;

    int robot_setup();
    void robot_shutdown();
    int go_to(Eigen::Vector3d pos);

    Robot();
public:
    bool error = false;
    ~Robot();

    Eigen::Vector3d get_current_cart_loc();
    int move_linear(Eigen::Vector3d goal_pos);
    int move_radial(double p1[3], double p2[3], double p3[3]);
    int move_joint(double joints[4]);
};
