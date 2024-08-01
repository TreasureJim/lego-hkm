#pragma once

#include <optional>
#include <vector>
#include <Eigen/Dense>

static const Eigen::Vector3d DEFAULT_LIM_MIN = {-1, -1, 0};
static const Eigen::Vector3d DEFAULT_LIM_MAX = {1, 1, 1};

static const Eigen::Vector3d UP_VEC = {0, 0, 1};

#ifdef USE_OMPL
#include "ompl_pathfinding.hpp"
#else
#include "custom_pathfinding.hpp"
#endif 
