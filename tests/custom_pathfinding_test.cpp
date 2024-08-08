#include "eigen_kinematics.hpp"
#include "custom_pathfinding.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <cassert>
#include <iostream>
#include <mark2_0_fixed.hpp>
#include "test_helpers.hpp"

#define FAIL 1

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
	std::cout << start << '\n' << end << std::endl;

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
