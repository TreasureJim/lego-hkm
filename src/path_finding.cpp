#include <array>
#include <memory>
#include <ompl-1.6/ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl-1.6/ompl/geometric/PathGeometric.h>
#include <ompl/base/goals/GoalLazySamples.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/GeneticSearch.h>
#include <ompl/geometric/SimpleSetup.h>
#include <vector>

#include "kinematics.h"
#include "robot_config.h"
#include "spacial_conv.h"
#include "mark2_0_fixed.h"

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

  ss.setStateValidityChecker(&check_state);
}

ompl::geometric::PathGeometric *find_path(double start_pos[3],
                                          double goal_pos[3]) {
  ob::ScopedState<ob::SE3StateSpace> start(space);
  start->setXYZ(start_pos[0], start_pos[1], start_pos[2]);
  start->rotation().setIdentity();

  ob::ScopedState<ob::SE3StateSpace> goal(space);
  goal->setXYZ(goal_pos[0], goal_pos[1], goal_pos[2]);
  goal->rotation().setIdentity();

  ss.setStartAndGoalStates(start, goal);

  ob::PlannerStatus solved = ss.solve();

  if (solved) {
    std::cout << "Found solution:" << std::endl;

    ss.simplifySolution();

    // std::ofstream matrix_file("../matrix.txt");
    // ss.getSolutionPath().printAsMatrix(matrix_file);

    return &ss.getSolutionPath();
  } else {
    std::cout << "No solution found\n";
    return nullptr;
  }
}

void geometric_paths_to_cart_coords(ompl::geometric::PathGeometric* path, std::vector<std::array<double, 3>>& path_cart_coords) {
  for (const ompl::base::State* state :path->getStates()) {
    std::vector<double> coord_vec;
    space->copyToReals(coord_vec, state);
    
    std::array<double, 3> cart_coord;
    cyl_pol_to_cart(coord_vec.data(), cart_coord.data());

    path_cart_coords.push_back(cart_coord);
  }
}
