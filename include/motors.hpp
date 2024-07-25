#pragma once
#include <cmath>
#include <cstdint>

void matrix_to_pos(double matrix[4][4], double pos[3]);

extern double DEFAULT_JOINT_ANGLES[4];
extern double current_joint_angles[4];

bool motor_setup();
void motor_shutdown();

/// This is used at the startup to allow the robot to go to default position and find its position of motors
/// Should **NOT** be used if motors are not in the default angles
void motor_reset_angle();

/// Will linearly move slowly between `start_angle` and `goal_angle` with
/// `delay` ms delay between each intermediate position `step_size` is the
/// maximum size between each intermediate position in radians
void motor_transition_angle(double start_angle[4], double goal_angle[4],
                            uint8_t delay = 10,
                            double step_size = 10 * M_PI / 180);
