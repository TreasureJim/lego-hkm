#pragma once

#include "IMotion.hpp"
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

public:
    bool error = false;

    Robot();
    ~Robot();

    Eigen::Vector3d get_current_cart_loc();
    int move_joint(double joints[4]);

    int execute_motion(IMotion& motion, float interval_size = 0.05);
};

void robot_thread_func();
