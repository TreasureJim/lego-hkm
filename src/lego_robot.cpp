#include "IMotion.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <ompl/geometric/PathGeometric.h>

#define RAD_TO_DEG 57.295779513

#include "kinematics.h"
#include "motors.hpp"

#include "robot.hpp"

LegoRobot::LegoRobot(agile_pkm_model &model) : Robot(model) { error = !this->robot_setup(); }

LegoRobot::~LegoRobot() { this->robot_shutdown(); }

int LegoRobot::robot_setup() {
	if (!motor_setup())
		return false;

	// printf("Setting motors to default positions.\n");
	// motor_reset_angle();
	return true;
}

void LegoRobot::robot_shutdown() {
	// const double *default_angles = DEFAULT_JOINT_ANGLES.data();
	// this->go_to(joint_angle_to_cart_loc(default_angles));
	motor_shutdown();
}

int LegoRobot::go_to(Eigen::Vector3d pos) {
	double goal_servo_angles[4];
	if (cart_to_drive(&lego_model, pos.data(), 0.0, goal_servo_angles) < 0) {
		fprintf(stderr, "ERROR: generating motor position for x: %f, y: %f, z: %f.\n", pos.x(), pos.y(), pos.z());
		return 0;
	}

	motor_transition_angle(current_joint_angles.data(), goal_servo_angles);
	return 1;
}
