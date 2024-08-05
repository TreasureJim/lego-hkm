#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/thread/pthread/thread_data.hpp>
#include <chrono>
#include <ctime>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <random>
#include <stdio.h>

#define RAD_TO_DEG 57.295779513

#include "kinematics.h"
#include "mark2_0_fixed.hpp"
#include "robot.hpp"

std::chrono::system_clock::rep time_since_epoch(){
    static_assert(
        std::is_integral<std::chrono::system_clock::rep>::value,
        "Representation of ticks isn't an integral value."
    );
    auto now = std::chrono::system_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::seconds>(now).count();
}

std::default_random_engine re(time_since_epoch());
Eigen::Vector3d random_valid_cart_pos(void) {
  double joint_angles[4];
  for (int i = 0; i < 4; i++) {
    std::uniform_real_distribution<double> unif(mark2_0_fixed.joint_lims[i][0] +
                                                    0.0001,
                                                mark2_0_fixed.joint_lims[i][1]);
    joint_angles[i] = unif(re);
  }

  double matrix[4][4];
  double angle = 0.0;
  fwd(&mark2_0_fixed, joint_angles, matrix, &angle);

  return Eigen::Vector3d(matrix[0][3], matrix[1][3], matrix[2][3]);
}

int main(int argc, char *argv[]) {
  Robot robot;
  if (robot.error) {
    fprintf(stderr, "[ERROR] Could not initialise robot.\n");
    return 0;
  }

  boost::this_thread::sleep(boost::posix_time::seconds(1));

  robot.move_linear( random_valid_cart_pos());
}
