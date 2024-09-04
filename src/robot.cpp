#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <ompl/geometric/PathGeometric.h>

#define RAD_TO_DEG 57.295779513

#include "kinematics.h"
#include "math/3d_circle.hpp"
#include "motors.hpp"

#include "robot.hpp"

Eigen::Vector3d joint_angle_to_cart_loc(const double angles[4]) {
	double mat[4][4];
	double orient;
	fwd(&lego_model, angles, mat, &orient);

	return Eigen::Vector3d(mat[0][3], mat[1][3], mat[2][3]);
}

int Robot::robot_setup() {
	if (!motor_setup())
		return false;

	printf("Setting motors to default positions.\n");
	motor_reset_angle();
	return true;
}

int Robot::go_to(Eigen::Vector3d pos) {
	double goal_servo_angles[4];
	if (cart_to_drive(&lego_model, pos.data(), 0.0, goal_servo_angles) < 0) {
		fprintf(stderr, "ERROR: generating motor position for x: %f, y: %f, z: %f.\n", pos.x(), pos.y(), pos.z());
		return 0;
	}

	motor_transition_angle(current_joint_angles, goal_servo_angles);
	return 1;
}

void Robot::robot_shutdown() {
	double *default_angles = DEFAULT_JOINT_ANGLES;
	this->go_to(joint_angle_to_cart_loc(default_angles));
	motor_shutdown();
}

Robot::Robot() { error = !robot_setup(); }

Robot::~Robot() { robot_shutdown(); }

Eigen::Vector3d Robot::get_current_cart_loc() { return joint_angle_to_cart_loc(current_joint_angles); }

int Robot::move_joint(double joints[4]) {
	// checking if valid angles
	double matrix[4][4];
	double orient;
	fwd(&lego_model, joints, matrix, &orient);
	double pos[3];
	matrix_to_pos(matrix, pos);
	double q[4];
	if (cart_to_drive(&lego_model, pos, 0, q) < 0) {
		return 0;
	}

	// move motors
	motor_transition_angle(current_joint_angles, joints);
	return 1;
}

int Robot::execute_motion(IMotion& motion, float interval_size) {
	if (!motion.is_valid()) return 1;

	for(float p = 0.0; p <= 1.0; p += 0.05) {
		this->go_to(motion.GetPoint(p));
	}
}
