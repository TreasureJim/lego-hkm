#include <Eigen/Dense>
#include <random>
#include "mark2_0_fixed.hpp"

std::default_random_engine re(std::random_device().entropy());
Eigen::Vector3d random_valid_cart_pos(void) {
	double joint_angles[4];
	for (int i = 0; i < 4; i++) {
		std::uniform_real_distribution<double> unif(mark2_0_fixed.joint_lims[i][0] + 0.0001,
		                                            mark2_0_fixed.joint_lims[i][1]);
		joint_angles[i] = unif(re);
	}

	double matrix[4][4];
	double angle = 0.0;
	fwd(&mark2_0_fixed, joint_angles, matrix, &angle);

	return Eigen::Vector3d(matrix[0][3], matrix[1][3], matrix[2][3]);
}