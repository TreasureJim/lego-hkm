#pragma once

int robot_setup();
void robot_shutdown();

int move_linear(double start_pos[3], double goal_pos[3]);
std::array<double, 3> get_current_cart_loc();

std::array<double, 3> joint_angle_to_cart_loc(const double angles[4]);
