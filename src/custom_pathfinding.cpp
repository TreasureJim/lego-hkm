#include "eigen_kinematics.hpp"
#include <Eigen/Dense>
#include <vector>

static const Eigen::Vector3d DEFAULT_LIM_MIN = {-1, -1, 0};
static const Eigen::Vector3d DEFAULT_LIM_MAX = {1, 1, 1};

static const Eigen::Vector3d UP_VEC = {0, 0, 1};

class PathFinding {
	Eigen::Vector3d limits_min;
	Eigen::Vector3d limits_max;

	inline bool pos_within_bounding_box(Eigen::Vector3d pos);
	Eigen::Vector3d ray_box_intersection(Eigen::Vector3d pos, Eigen::Vector3d dir);
	std::optional<Eigen::Vector3d> find_last_valid_pos(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& end_pos);

  public:
	PathFinding(Eigen::Vector3d limits_min = DEFAULT_LIM_MIN, Eigen::Vector3d limits_max = DEFAULT_LIM_MAX);
	std::optional<std::vector<Eigen::Vector3d>> find_path(Eigen::Vector3d start_pos, Eigen::Vector3d goal_pos);
};

PathFinding::PathFinding(Eigen::Vector3d limits_min, Eigen::Vector3d limits_max) {
	this->limits_min = limits_min;
	this->limits_max = limits_max;
}

inline bool PathFinding::pos_within_bounding_box(Eigen::Vector3d pos) {
	return pos.x() < this->limits_min.x() || pos.x() > this->limits_max.x() ||
		pos.y() < this->limits_min.y() || pos.y() > this->limits_max.y() ||
		pos.z() < this->limits_min.z() || pos.z() > this->limits_max.z();
}

std::optional<Eigen::Vector3d> PathFinding::find_last_valid_pos(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& end_pos) {
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

/// Computes the intersection for a bounding box and a ray given that the ray starts inside the box 
Eigen::Vector3d PathFinding::ray_box_intersection(Eigen::Vector3d pos, Eigen::Vector3d dir) {
    double t_min_x = (this->limits_min.x() - pos.x()) / dir.x();
    double t_max_x = (this->limits_max.x() - pos.x()) / dir.x();

    double t_min_y = (this->limits_min.y() - pos.y()) / dir.y();
    double t_max_y = (this->limits_max.y() - pos.y()) / dir.y();

    double t_min_z = (this->limits_min.z() - pos.z()) / dir.z();
    double t_max_z = (this->limits_max.z() - pos.z()) / dir.z();

    // Ensure t_min is the entry point and t_max is the exit point for each axis
    if (t_min_x > t_max_x) std::swap(t_min_x, t_max_x);
    if (t_min_y > t_max_y) std::swap(t_min_y, t_max_y);
    if (t_min_z > t_max_z) std::swap(t_min_z, t_max_z);

    // Find the largest t_min, which is the first intersection with the cube's surface
    double t_hit = std::max({t_min_x, t_min_y, t_min_z});

    // Compute the intersection point
    return pos + t_hit * dir;
}

std::optional<std::vector<Eigen::Vector3d>> PathFinding::find_path(Eigen::Vector3d start_pos, Eigen::Vector3d goal_pos) {
	// check if within limits
	if (!pos_within_bounding_box(start_pos)) return std::nullopt;
	if (!pos_within_bounding_box(goal_pos)) return std::nullopt;

	std::vector<Eigen::Vector3d> path = {start_pos};
	
	while(true) {
		auto pos_opt = find_last_valid_pos(path.back(), goal_pos);
		// check if path from last node to goal is clear
		if (!pos_opt.has_value()) break;
		path.push_back(pos_opt.value());

		auto vec_cross = (path.end()[-2] - path.back()).cross(UP_VEC).normalized();
		auto intersection_point = this->ray_box_intersection(path.back(), vec_cross);
		path.push_back(path.back() + vec_cross * (intersection_point - path.back()).norm() / 2);
	}

	path.push_back(goal_pos);

	return path;
}
