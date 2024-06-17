#include <cstdio>

#include <fstream>
#include <memory>
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

int main() {
  auto space = std::make_shared<ob::SE3StateSpace>();

  ob::RealVectorBounds bounds(3);
  bounds.setLow(0);
  bounds.setHigh(100);

  space->setBounds(bounds);

  og::SimpleSetup ss(space);
  ss.setStateValidityChecker(&check_state);

  ob::ScopedState<ob::SE3StateSpace> start(space);
  start->setXYZ(1.0, 1.0, 10.0);
  start->rotation().setIdentity();

  ob::ScopedState<ob::SE3StateSpace> goal(space);
  goal->setXYZ(2.0, 3.0, 5.0);
  goal->rotation().setIdentity();

  ss.setStartAndGoalStates(start, goal);

  ob::PlannerStatus solved = ss.solve();

  if (solved) {
    std::cout << "Found solution:" << std::endl;

    ss.simplifySolution();
    ss.getSolutionPath().print(std::cout);
    ss.getSolutionPath().printAsMatrix(std::cout);

    std::ofstream matrix_file("../matrix.txt");
    ss.getSolutionPath().printAsMatrix(matrix_file);
  } else {
    std::cout << "No solution found\n";
  }
}
