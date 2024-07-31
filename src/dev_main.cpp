#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <random>

#include "kinematics.h"
#include "mark2_0_fixed.hpp"
#include "robot.hpp"

Eigen::Vector3d random_valid_cart_pos(void) {
  double joint_angles[4];
  for (int i = 0; i < 4; i++) {
    std::uniform_real_distribution<double> unif(mark2_0_fixed.joint_lims[i][0] +
                                                    0.0001,
                                                mark2_0_fixed.joint_lims[i][1]);
    std::default_random_engine re;
    joint_angles[i] = unif(re);
  }

  double matrix[4][4];
  double angle = 0.0;
  fwd(&mark2_0_fixed, joint_angles, matrix, &angle);

  return Eigen::Vector3d(matrix[0][3], matrix[1][3], matrix[2][3]);
}

int main (int argc, char *argv[]) {
  Robot robot;
  if (robot.error)
    return 0;

  robot.move_linear( random_valid_cart_pos());
}
