#include <ompl/base/goals/GoalLazySamples.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/GeneticSearch.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <optional>
#include <vector>

#include "kinematics.h"
#include "mark2_0_fixed.hpp"
#include "ompl_pathfinding.hpp"

PathFinding::PathFinding(Eigen::Vector3d limits_min, Eigen::Vector3d limits_max): ss(space) {
	ob::RealVectorBounds bounds(3);
	bounds.setLow(0, limits_min.x());
	bounds.setLow(1, limits_min.y());
	bounds.setLow(2, limits_min.z());

	bounds.setHigh(0, limits_max.x());
	bounds.setHigh(1, limits_max.y());
	bounds.setHigh(2, limits_max.z());

	space->setBounds(bounds);

	this->ss = og::SimpleSetup(space);
	this->ss.setStateValidityChecker(PathFinding::check_state);
}

 bool PathFinding::check_state(const ob::State *state) {
	const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

	const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

	double pos_arr[3] = {se3state->getX(), se3state->getY(), se3state->getZ()};
	double _[4];
	if (inv(&mark2_0_fixed, pos_arr, 0.0, _) < 0) {
		return false;
	}

	return true;
}

std::optional<std::vector<Eigen::Vector3d>> PathFinding::find_path(Eigen::Vector3d start_pos, Eigen::Vector3d goal_pos) {
	ob::ScopedState<ob::SE3StateSpace> start(space);
	start->setXYZ(start_pos.x(), start_pos.y(), start_pos.z());
	start->rotation().setIdentity();

	ob::ScopedState<ob::SE3StateSpace> goal(space);
	goal->setXYZ(goal_pos.x(), goal_pos.y(), goal_pos.z());
	goal->rotation().setIdentity();

	ss.setStartAndGoalStates(start, goal);

	ob::PlannerStatus solved = ss.solve();

	if (solved) {
		// std::cout << "Found solution:" << std::endl;
		ss.simplifySolution();
	} else {
		std::cout << "No solution found\n";
		return std::nullopt;
	}
	ompl::geometric::PathGeometric path = ss.getSolutionPath();
	;

	std::vector<Eigen::Vector3d> path_coords;
	path_coords.reserve(path.getStateCount());

	for (const ompl::base::State *state : path.getStates()) {
		std::vector<double> coord_vec;
		space->copyToReals(coord_vec, state);
		Eigen::Vector3d cart_coord = {coord_vec[0], coord_vec[1], coord_vec[2]};

		path_coords.push_back(cart_coord);
	}

	return path_coords;
}
