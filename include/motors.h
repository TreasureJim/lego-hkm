#pragma once
#include <cstdint>


void motor_setup();
void motor_cleanup();
void motor_reset_angle();
void motor_transition_angle(double start_angle[3], double goal_angle[3],
                            uint8_t delay = 10,
                            double step_size = 10 * M_PI / 180);
