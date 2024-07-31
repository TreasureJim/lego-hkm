#include "eigen_kinematics.hpp"
#include <Eigen/Dense>
#include <vector>

static const Eigen::Vector3d DEFAULT_LIM_MIN = {-1, -1, 0};
static const Eigen::Vector3d DEFAULT_LIM_MAX = {1, 1, 1};

static const Eigen::Vector3d UP_VEC = {0, 0, 1};

class PathFinding {
	Eigen::Vector3d limits_min;
	Eigen::Vector3d limits_max;

	std::optional<Eigen::Vector3d> find_last_valid_pos(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& end_pos);

  public:
	PathFinding(Eigen::Vector3d limits_min = DEFAULT_LIM_MIN, Eigen::Vector3d limits_max = DEFAULT_LIM_MAX);
	std::vector<Eigen::Vector3d> find_path(Eigen::Vector3d start_pos, Eigen::Vector3d goal_pos);
};

PathFinding::PathFinding(Eigen::Vector3d limits_min, Eigen::Vector3d limits_max) {
	this->limits_min = limits_min;
	this->limits_max = limits_max;
}

inline bool pos_within_bounding_box(Eigen::Vector3d limits_min, Eigen::Vector3d limits_max, Eigen::Vector3d pos) {
	return pos.x() < limits_min.x() || pos.x() > limits_max.x() ||
		pos.y() < limits_min.y() || pos.y() > limits_max.y() ||
		pos.z() < limits_min.z() || pos.z() > limits_max.z();
}

std::optional<Eigen::Vector3d> PathFinding::find_last_valid_pos(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& end_pos) {
	// check if within limits
	if (!pos_within_bounding_box(this->limits_min, this->limits_max, start_pos)) return std::nullopt;
	if (!pos_within_bounding_box(this->limits_min, this->limits_max, end_pos)) return std::nullopt;

	// find first invalid position
	Eigen::Vector3d first_invalid;
	bool no_invalid = true;
	for (float perc = 0.0; perc <= 1.0; perc += 0.1) {
		auto pos = start_pos + (end_pos - start_pos) * perc;

		if (!inverse(pos).has_value()) {
			first_invalid = pos;
			no_invalid = false;
			break;
		}
	}
	if (no_invalid) return std::nullopt;

	// traverse back until valid
	for (float perc = 0.0; perc <= 1.0; perc += 0.1) {
		Eigen::Vector3d pos = first_invalid - (first_invalid - start_pos) * perc;
		if (inverse(pos).has_value()) return pos;
	}

	return std::nullopt;
}

// TODO: implement function for determining intersection position on bounding box from origin and direction

std::vector<Eigen::Vector3d> PathFinding::find_path(Eigen::Vector3d start_pos, Eigen::Vector3d goal_pos) {
	std::vector<Eigen::Vector3d> path = {start_pos};
	
	while(true) {
		auto pos_opt = find_last_valid_pos(path.back(), goal_pos);
		// check if path from last node to goal is clear
		if (!pos_opt.has_value()) break;
		path.push_back(pos_opt.value());

		auto vec_cross = (path.end()[-2] - path.back()).cross(UP_VEC).normalized();
		auto test_pos // TOOD: finish
	}

	path.push_back(goal_pos);

	return path;
}
