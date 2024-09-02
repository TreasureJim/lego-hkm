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

int Robot::move_linear(Eigen::Vector3d goal_pos) {
	auto path = this->pathfinding.find_path(this->get_current_cart_loc(), goal_pos);
	if (!path.has_value()) {
		fprintf(stderr, "ERROR: Could not plan path.\n");
		return 0;
	}

	for (const auto &coord : path.value()) {
		double servo_angles[4];
		if (cart_to_drive(&lego_model, coord.data(), 0.0, servo_angles) < 0) {
			fprintf(stderr, "Error generating motor position for x: %f, y: %f, z: %f.\n", coord[0], coord[1], coord[2]);
			continue;
		}

		// printf("Cart: x: %f, y: %f, z: %f. Drive positions: %f, %f, %f\n", coord[0], coord[1], coord[2], servo_angles[0] * RAD_TO_DEG, servo_angles[1] * RAD_TO_DEG, servo_angles[2] * RAD_TO_DEG);
		printf("Cart: x: %f, y: %f, z: %f. Drive positions: %f, %f, %f\n", coord[0], coord[1], coord[2], servo_angles[0], servo_angles[1], servo_angles[2]);
		this->go_to(coord);
	}

	return 0;
}

int Robot::move_arc(Eigen::Vector3d v2, Eigen::Vector3d v3) {
	Eigen::Vector3d v1 = this->get_current_cart_loc();

	Circle_3D circle(v1, v2, v3);
	if (!circle)
		return 0;

	for (float p = 0.0; p <= 1.0; p += 0.05) {
		this->go_to(circle.get_arc_coord(p));
	}

	return 1;
}

int Robot::move_circular(Eigen::Vector3d v2, Eigen::Vector3d v3) {
	Eigen::Vector3d v1 = this->get_current_cart_loc();

	Circle_3D circle(v1, v2, v3);
	if (!circle)
		return 0;

	for (float a = 0.0; a <= 2 * M_PI; a += 0.05) {
		this->go_to(circle.get_arc_coord(a));
	}

	return 1;
}

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
