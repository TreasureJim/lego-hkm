#pragma once

#include <Eigen/Dense>
#include <optional>

extern "C" {
#include "kinematics.h"
}
#include "pathfinding.hpp"

class PathFinding {
	agile_pkm_model* model;

	Eigen::Vector3d limits_min;
	Eigen::Vector3d limits_max;

	inline bool pos_within_bounding_box(Eigen::Vector3d pos);
	Eigen::Vector3d ray_box_intersection(Eigen::Vector3d pos, Eigen::Vector3d dir);
	std::optional<Eigen::Vector3d> find_last_valid_pos(const Eigen::Vector3d &start_pos,
	                                                   const Eigen::Vector3d &end_pos);

  public:
	PathFinding(Eigen::Vector3d limits_min = DEFAULT_LIM_MIN, Eigen::Vector3d limits_max = DEFAULT_LIM_MAX);
	std::optional<std::vector<Eigen::Vector3d>> find_path(Eigen::Vector3d start_pos, Eigen::Vector3d goal_pos);
};
