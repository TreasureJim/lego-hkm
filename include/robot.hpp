#pragma once

std::array<double, 3> joint_angle_to_cart_loc(const double angles[4]);

class Robot {
private:
    int robot_setup();
    void robot_shutdown();

public:
    bool error = false;

    Robot();
    ~Robot();

    std::array<double, 3> get_current_cart_loc();
    int move_linear(double start_pos[3], double goal_pos[3]);
    // int move_radial(double p1[3], double p2[3], double p3[3]);
};
