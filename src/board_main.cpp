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
#include "test_helpers.hpp"


int main(int argc, char *argv[]) {
  Robot robot;
  if (robot.error) {
    fprintf(stderr, "[ERROR] Could not initialise robot.\n");
    return 0;
  }

  boost::this_thread::sleep(boost::posix_time::seconds(1));

  robot.move_linear( random_valid_cart_pos());
}
