#include "lego_model.hpp"
#include "motors.hpp"
#include <array>
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/thread.hpp>
#include <boost/thread/detail/thread.hpp>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

extern "C" {
#include "rc/adc.h"
#include "rc/servo.h"
#include "rc/time.h"
}

std::array<double, 4> default_joint_angles;
std::array<std::array<double, 2>, 3> motor_offset_values;

std::array<double, 4> current_joint_angles;

void matrix_to_pos(double matrix[4][4], double pos[3]) {
	pos[0] = matrix[0][3];
	pos[1] = matrix[1][3];
	pos[2] = matrix[2][3];
}

double joint_angle_to_libservo_value(double joint_angle, uint8_t motor) {
	const double *lego_lim = lego_model.joint_lims[motor];
	// transform angle between 0 and 1
	const double norm_angle = (joint_angle - lego_lim[0]) / (lego_lim[1] - lego_lim[0]);

	const std::array<double, 2> motor_lim = motor_offset_values[motor];
	return norm_angle * (motor_lim[1] - motor_lim[0]) + motor_lim[0];
}

double motor_angle_to_joint_angle(int motor, double motor_angle) {
	const std::array<double, 2> motor_lim = motor_offset_values[motor];
	double norm_value = (motor_angle - motor_lim[0]) / (motor_lim[1] - motor_lim[0]);

	const double *lego_lim = lego_model.joint_lims[motor];
	return norm_value * (lego_lim[1] - lego_lim[0]) + lego_lim[0];
}


bool motor_setup() {
	// read adc to make sure battery is connected
	if (rc_adc_init()) {
		fprintf(stderr, "[ERROR] failed to run rc_adc_init()\n");
		return false;
	}
	if (rc_adc_batt() < 6.0) {
		fprintf(stderr, "[ERROR] battery disconnected or insufficiently charged to drive servos\n");
		return false;
	}
	rc_adc_cleanup();

	if (rc_servo_init()) {
		fprintf(stderr, "[ERROR] Could not initialise servos.\n");
		return false;
	}
	if (rc_servo_power_rail_en(1)) {
		fprintf(stderr, "[ERROR] Could not enable 6V power rail.\n");
		return false;
	}

	return true;
}

void motor_set_angle(double angles[4]) {
	current_joint_angles[0] = joint_angle_to_libservo_value(angles[0], 0);
	current_joint_angles[1] = joint_angle_to_libservo_value(angles[1], 1);
	current_joint_angles[2] = joint_angle_to_libservo_value(angles[2], 2);

	for (int i = 0; i < 2; i++) {
		rc_servo_send_pulse_normalized(1, current_joint_angles[0]);
		rc_servo_send_pulse_normalized(2, current_joint_angles[1]);
		rc_servo_send_pulse_normalized(3, current_joint_angles[2]);
		rc_usleep(1e6 / 50);
	}
}

void motor_shutdown() {
	motor_set_angle(default_joint_angles.data());
	
	if (rc_servo_power_rail_en(0))
		fprintf(stderr, "ERROR: could not disable 6V power rail.\n");
}

void calculate_motor_default_angles() {
	for (int i = 0; i < 3; i++) {
		double motor_angle = motor_offset_values[i][0] + (motor_offset_values[i][1] - motor_offset_values[i][0]) / 2.0;
		default_joint_angles[i] = motor_angle_to_joint_angle(i, motor_angle);
	}
}

void motor_reset_angle() {
	calculate_motor_default_angles();

	current_joint_angles[0] = default_joint_angles[0];
	current_joint_angles[1] = default_joint_angles[1];
	current_joint_angles[2] = default_joint_angles[2];

	for (int i = 0; i < 50; i++) {
		rc_servo_send_pulse_normalized(1, joint_angle_to_libservo_value(current_joint_angles[0], 0));
		rc_servo_send_pulse_normalized(2, joint_angle_to_libservo_value(current_joint_angles[1], 1));
		rc_servo_send_pulse_normalized(3, joint_angle_to_libservo_value(current_joint_angles[2], 2));
		rc_usleep(1e6 / 50);
	}
}

void motor_transition_angle(double start_angle[4], double goal_angle[4], uint8_t delay, double step_size) {
	double largest_angle = 0.0;

	for (int i = 0; i < 4; i++) {
		double diff = abs(start_angle[i] - goal_angle[i]);
		if (diff > largest_angle)
			largest_angle = diff;
	}

	int num_steps = largest_angle / step_size;

	for (int n = 0; n < largest_angle / step_size; n++) {
		for (int j = 0; j < 3; j++) {
			double new_angle = start_angle[j] + (goal_angle[j] - start_angle[j]) / num_steps * n;
			auto translated_angle = joint_angle_to_libservo_value(new_angle, j);
			if (rc_servo_send_pulse_normalized(j + 1, translated_angle))
				fprintf(stderr, "[ERROR] could not move motor %d to angle %f\n", j + 1, translated_angle);
		}
		boost::this_thread::sleep(boost::posix_time::milliseconds(delay));
	}

	for (int j = 0; j < 3; j++) {
		rc_servo_send_pulse_normalized(j + 1, joint_angle_to_libservo_value(goal_angle[j], j));
	}
	boost::this_thread::sleep(boost::posix_time::milliseconds(delay));

	current_joint_angles[0] = goal_angle[0];
	current_joint_angles[1] = goal_angle[1];
	current_joint_angles[2] = goal_angle[2];
	current_joint_angles[3] = goal_angle[3];
}
