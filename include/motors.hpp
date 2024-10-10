#pragma once
#include <cmath>
#include <cstdint>
#include <array>

void matrix_to_pos(double matrix[4][4], double pos[3]);

extern std::array<double, 4> DEFAULT_JOINT_ANGLES;
extern std::array<double, 4> current_joint_angles;

extern std::array<std::array<double, 2>, 3> motor_offset_values;

bool motor_setup();
void motor_shutdown();

double joint_angle_to_libservo_value(double joint_angle, uint8_t motor);
double motor_angle_to_joint_angle(int motor, double motor_angle);

/// This is used at the startup to allow the robot to go to default position and find its position of motors
/// Should **NOT** be used if motors are not in the default angles
void motor_reset_angle();

void motor_set_angle(double angles[4]);

/// Will linearly move slowly between `start_angle` and `goal_angle` with
/// `delay` ms delay between each intermediate position `step_size` is the
/// maximum size between each intermediate position in radians
void motor_transition_angle(double start_angle[4], double goal_angle[4],
                            uint8_t delay = 10,
                            double step_size = 10 * M_PI / 180);
