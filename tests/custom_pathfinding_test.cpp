#include "eigen_kinematics.hpp"
#include "custom_pathfinding.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <cassert>
#include <mark2_0_fixed.hpp>
#include <random>

#define FAIL 1

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

Eigen::Vector3d interpolate_pos(Eigen::Vector3d start, Eigen::Vector3d end, float p) {
	assert(p >= 0.0 && p <= 1.0);
	return start + (end - start) * p;
}

inline bool validate_position(Eigen::Vector3d pos) {
	return inverse(pos).has_value();
}

int main() {
	auto start = random_valid_cart_pos();
	auto end = random_valid_cart_pos();

	PathFinding pathfinding;
	auto path_opt = pathfinding.find_path(start, end);
	if (!path_opt.has_value()) return FAIL;
	auto path = path_opt.value();

	for (int i = 1; i < path.size(); i++) {
		for (float p = 0.0; p <= 1.0; p += 0.05) {
			if (!validate_position(interpolate_pos(path[i - 1], path[i], p))) return FAIL;
		}
	}

	return 0;
}
