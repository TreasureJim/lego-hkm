#include <memory>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/goals/GoalLazySamples.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/GeneticSearch.h>
#include <ompl/geometric/SimpleSetup.h>
#include <optional>
#include <vector>

#include "kinematics.h"
#include "mark2_0_fixed.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool check_state(const ob::State *state) {
  const ob::SE3StateSpace::StateType *se3state =
      state->as<ob::SE3StateSpace::StateType>();

  const ob::RealVectorStateSpace::StateType *pos =
      se3state->as<ob::RealVectorStateSpace::StateType>(0);

  // if (se3state->getX() < ROBOT_INVALID_RADIUS) {
  //   return false;
  // }

  double pos_arr[3] = {se3state->getX(), se3state->getY(), se3state->getZ()};
  double _[4];
  if (inv(&mark2_0_fixed, pos_arr, 0.0, _) < 0) {
    return false;
  }

  return true;
}

auto space = std::make_shared<ob::SE3StateSpace>();
og::SimpleSetup ss(space);

void path_finding_setup() {
  ob::RealVectorBounds bounds(3);
  bounds.setLow(-100);
  bounds.setHigh(100);

  space->setBounds(bounds);

  ss.setStateValidityChecker(check_state);
}

std::optional<std::vector<Eigen::Vector3d>> find_path(Eigen::Vector3d start_pos, Eigen::Vector3d goal_pos) {
  ob::ScopedState<ob::SE3StateSpace> start(space);
  start->setXYZ(start_pos.x(), start_pos.y(),start_pos.z());
  start->rotation().setIdentity();

  ob::ScopedState<ob::SE3StateSpace> goal(space);
  goal->setXYZ(goal_pos.x(), goal_pos.y(),goal_pos.z());
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
  ompl::geometric::PathGeometric path = ss.getSolutionPath();;


  std::vector<Eigen::Vector3d> path_coords;
  path_coords.reserve(path.getStateCount());

  for (const ompl::base::State* state :path.getStates()) {
    std::vector<double> coord_vec;
    space->copyToReals(coord_vec, state);
    Eigen::Vector3d cart_coord = {coord_vec[0], coord_vec[1], coord_vec[2]};

    path_coords.push_back(cart_coord);
  }
}
