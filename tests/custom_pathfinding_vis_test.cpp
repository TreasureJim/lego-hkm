#include "pathfinding.hpp"
#include "test_helpers.hpp"
#include <Eigen/Dense>
#include <vector>

#define EXPORT __attribute__((visibility("default")))

extern "C" {

EXPORT double *find_path(double start_x, double start_y, double start_z, double goal_x, double goal_y, double goal_z,
                         int *path_size) {
	Eigen::Vector3d start(start_x, start_y, start_z);
	Eigen::Vector3d goal(goal_x, goal_y, goal_z);

	auto path_opt = PathFinding().find_path(start, goal);
	if (!path_opt.has_value()) {
		*path_size = 0;
		return nullptr;
	}

	auto path = path_opt.value();

	*path_size = path.size() * 3; // Total number of doubles

	double *result = new double[*path_size];
	for (int i = 0; i < path.size(); ++i) {
		result[i * 3 + 0] = path[i].x();
		result[i * 3 + 1] = path[i].y();
		result[i * 3 + 2] = path[i].z();
	}

	return result;
}

EXPORT double *find_path_random(int *path_size) {
	Eigen::Vector3d start = random_valid_cart_pos();
	Eigen::Vector3d goal = random_valid_cart_pos();

	return find_path(start.x(), start.y(), start.z(), goal.x(), goal.y(), goal.z(), path_size);
}

EXPORT void free_memory(double *array) { free(array); }
}
