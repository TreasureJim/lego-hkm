#pragma once

#include <memory>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/goals/GoalLazySamples.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/GeneticSearch.h>
#include <ompl/geometric/SimpleSetup.h>
#include <optional>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;

#include "pathfinding.hpp"

class PathFinding {
private:
	std::shared_ptr<ompl::base::SE3StateSpace> space = std::make_shared<ob::SE3StateSpace>();
	og::SimpleSetup ss;

	static bool check_state(const ob::State *state);

  public:
	PathFinding(Eigen::Vector3d limits_min = DEFAULT_LIM_MIN, Eigen::Vector3d limits_max = DEFAULT_LIM_MAX);
	std::optional<std::vector<Eigen::Vector3d>> find_path(Eigen::Vector3d start_pos, Eigen::Vector3d goal_pos);
};
