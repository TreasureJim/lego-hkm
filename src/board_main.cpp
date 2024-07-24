#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <random>

#define RAD_TO_DEG 57.295779513

#include "kinematics.h"
#include "mark2_0_fixed.h"
#include "robot.hpp"

void random_valid_cart_pos(double cart_pos[3]) {
  double joint_angles[4];
  for (int i = 0; i < 4; i++) {
    std::uniform_real_distribution<double> unif (mark2_0_fixed.joint_lims[i][0] + 0.0001, mark2_0_fixed.joint_lims[i][1]);
    std::default_random_engine re;
    joint_angles[i] = unif(re);
  }

  double matrix[4][4];
  double angle = 0.0;
  fwd(&mark2_0_fixed, joint_angles, matrix, &angle);

  cart_pos[0] = matrix[0][3];
  cart_pos[1] = matrix[1][3];
  cart_pos[2] = matrix[2][3];
}

int main (int argc, char *argv[]) {
  Robot robot;
  if (robot.error) return 0;

  double goal_pos[3];
  random_valid_cart_pos(goal_pos);
  robot.move_linear(robot.get_current_cart_loc().data(), goal_pos);
}
