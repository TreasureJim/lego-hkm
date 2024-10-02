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

Robot::Robot(agile_pkm_model &model) : model(model) {}

agile_pkm_model& Robot::get_model() {
	return this->model;
}

Eigen::Vector3d joint_angle_to_cart_loc(agile_pkm_model &model, const double angles[4]) {
	double mat[4][4];
	double orient;
	fwd(&model, angles, mat, &orient);

	return Eigen::Vector3d(mat[0][3], mat[1][3], mat[2][3]);
}

Eigen::Vector3d Robot::joint_angle_to_cart_loc(const double angles[4]) {
	return ::joint_angle_to_cart_loc(this->model, angles);
}

int Robot::execute_motion(IMotion &motion, float interval_size) {
	if (!motion.is_valid()) {
		fprintf(stderr, "[ERROR] Invalid motion.\n");
		return -1;
	}

	/* try {
	    MotionJoint &jointmotion = dynamic_cast<MotionJoint &>(motion);
	    this->move_joint(jointmotion.get_angles().data());
	    return 1;
	} catch (const std::bad_cast &e) {
	} */

	for (float p = 0.0; p <= 1.0; p += 0.05) {
		this->go_to(motion.GetPoint(p));
	}

	return 1;
}

/* int Robot::move_joint(double joints[4]) {
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
    motor_transition_angle(current_joint_angles.data(), joints);
    return 1;
} */
