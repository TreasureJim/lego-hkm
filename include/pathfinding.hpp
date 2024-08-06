#pragma once

#include <Eigen/Dense>

static const Eigen::Vector3d DEFAULT_LIM_MIN = {-2, -2, 0};
static const Eigen::Vector3d DEFAULT_LIM_MAX = {2, 2, 2};

static const Eigen::Vector3d UP_VEC = {0, 0, 1};

#ifdef USE_OMPL
#include "ompl_pathfinding.hpp"
#else
#include "custom_pathfinding.hpp"
#endif 
