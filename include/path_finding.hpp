#pragma once

#include <ompl/geometric/PathGeometric.h>
#include <optional>

void path_finding_setup();
std::optional<std::vector<Eigen::Vector3d>> find_path(Eigen::Vector3d start_pos, Eigen::Vector3d goal_pos);
