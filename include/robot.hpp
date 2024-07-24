#pragma once


#include <Eigen/Dense>
#include <array>

Eigen::Vector3d joint_angle_to_cart_loc(const double angles[4]);

class Robot {
private:
    Eigen::Vector3d cart_pos;

    int robot_setup();
    void robot_shutdown();
    int go_to(Eigen::Vector3d pos);

public:
    bool error = false;

    Robot();
    ~Robot();

    Eigen::Vector3d get_current_cart_loc();
    int move_linear(Eigen::Vector3d start_pos, Eigen::Vector3d goal_pos);
    int move_radial(double p1[3], double p2[3], double p3[3]);
};