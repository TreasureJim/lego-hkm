#include "mark2_0_fixed.hpp"
#include "rc/adc.h"
#include "rc/servo.h"
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/thread.hpp>
#include <boost/thread/detail/thread.hpp>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

double DEFAULT_JOINT_ANGLES[] = {mark2_0_fixed.joint_lims[0][0] + 0.01, mark2_0_fixed.joint_lims[1][0] + 0.01,
                                 mark2_0_fixed.joint_lims[2][0] + 0.01, mark2_0_fixed.joint_lims[3][0] + 0.01};

double current_joint_angles[4];

void matrix_to_pos(double matrix[4][4], double pos[3]) {
	pos[0] = matrix[0][3];
	pos[1] = matrix[1][3];
	pos[2] = matrix[2][3];
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

void motor_shutdown() {
	if (rc_servo_power_rail_en(0))
		fprintf(stderr, "ERROR: could not disable 6V power rail.\n");
}

double joint_angle_to_libservo_value(double joint_angle) { return joint_angle / M_PI; }

void motor_reset_angle() {
	current_joint_angles[0] = DEFAULT_JOINT_ANGLES[0];
	current_joint_angles[1] = DEFAULT_JOINT_ANGLES[1];
	current_joint_angles[2] = DEFAULT_JOINT_ANGLES[2];
	current_joint_angles[3] = DEFAULT_JOINT_ANGLES[3];

	for (int i = 0; i < 50; i++) {
		rc_servo_send_pulse_normalized(1, joint_angle_to_libservo_value(current_joint_angles[0]));
		rc_servo_send_pulse_normalized(2, joint_angle_to_libservo_value(current_joint_angles[1]));
		rc_servo_send_pulse_normalized(3, joint_angle_to_libservo_value(current_joint_angles[2]));
		rc_servo_send_pulse_normalized(4, joint_angle_to_libservo_value(current_joint_angles[3]));
	}

	boost::this_thread::sleep(boost::posix_time::milliseconds(10));
}

void motor_transition_angle(double start_angle[4], double goal_angle[4], uint8_t delay = 10,
                            double step_size = 10 * M_PI / 180) {
	double largest_angle = 0.0;

	for (int i = 0; i < 4; i++) {
		double diff = abs(start_angle[i] - goal_angle[i]);
		if (diff > largest_angle)
			largest_angle = diff;
	}

	int num_steps = largest_angle / step_size;

	for (int n = 0; n < largest_angle / step_size; n++) {
		for (int j = 0; j < 4; j++) {
			double new_angle = start_angle[j] + (goal_angle[j] - start_angle[j]) / num_steps * n;
			auto translated_angle = joint_angle_to_libservo_value(new_angle);
			if (rc_servo_send_pulse_normalized(j + 1, translated_angle))
				fprintf(stderr, "[ERROR] could not move motor %d to angle %f\n", j + 1, translated_angle);
		}
		boost::this_thread::sleep(boost::posix_time::milliseconds(delay));
	}

	for (int j = 0; j < 4; j++) {
		rc_servo_send_pulse_normalized(j + 1, joint_angle_to_libservo_value(goal_angle[j]));
	}
	boost::this_thread::sleep(boost::posix_time::milliseconds(delay));

	current_joint_angles[0] = goal_angle[0];
	current_joint_angles[1] = goal_angle[1];
	current_joint_angles[2] = goal_angle[2];
	current_joint_angles[3] = goal_angle[3];
}
