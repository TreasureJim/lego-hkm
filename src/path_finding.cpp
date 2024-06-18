#include <fstream>
#include <memory>
#include <ompl-1.6/ompl/geometric/PathGeometric.h>
#include <ompl/base/goals/GoalLazySamples.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/GeneticSearch.h>
#include <ompl/geometric/SimpleSetup.h>

#include "robot_config.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool check_state(const ob::State *state) {
  const ob::SE3StateSpace::StateType *se3state =
      state->as<ob::SE3StateSpace::StateType>();

  const ob::RealVectorStateSpace::StateType *pos =
      se3state->as<ob::RealVectorStateSpace::StateType>(0);

  if (se3state->getZ() < ROBOT_INVALID_RADIUS) {
    fprintf(stderr, "State: invalid confine.\n");
    return false;
  }

  return true;
}

auto space = std::make_shared<ob::SE3StateSpace>();
og::SimpleSetup ss(space);

void path_finding_setup() {
  ob::RealVectorBounds bounds(3);
  bounds.setLow(0);
  bounds.setHigh(100);

  space->setBounds(bounds);

  ss.setStateValidityChecker(&check_state);
}

ompl::geometric::PathGeometric* find_path(double start_pos[3], double goal_pos[3]) {
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
    // ss.getSolutionPath().print(std::cout);
    //
    // std::ofstream matrix_file("../matrix.txt");
    // ss.getSolutionPath().printAsMatrix(matrix_file);

		return &ss.getSolutionPath();
  } else {
    std::cout << "No solution found\n";
		return nullptr;
  }
}
