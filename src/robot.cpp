#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <ompl-1.6/ompl/geometric/PathGeometric.h>

#define RAD_TO_DEG 57.295779513

#include "kinematics.h"
#include "mark2_0_fixed.h"
#include "path_finding.h"
#include "motors.h"

/// `path_finding_setup` needs to be run first before running this
int move_linear(double start_pos[3], double goal_pos[3]) {
  ompl::geometric::PathGeometric *path = find_path(start_pos, goal_pos);
  if (!path) {
    fprintf(stderr, "ERROR: Could not plan path.\n");
    return 0;
  }
  std::vector<std::array<double, 3>> cart_coords;
  // geometric_paths_to_cart_coords(path, cart_coords);

  // path->print(std::cout);

  for (const auto &coord : cart_coords) {
    double servo_angles[4];
    int result = cart_to_drive(&mark2_0_fixed, coord.data(), 0.0, servo_angles);
    if (result < 0) {
      fprintf(stderr,
              "Error generating motor position for x: %f, y: %f, z: %f.\n",
              coord[0], coord[1], coord[2]);
      continue;
    }

    printf("Cart: x: %f, y: %f, z: %f. Drive positions: %f, %f, %f\n", coord[0],
           coord[1], coord[2], servo_angles[0] * RAD_TO_DEG,
           servo_angles[1] * RAD_TO_DEG, servo_angles[2] * RAD_TO_DEG);
    motor_transition_angle(double *start_angle, double *goal_angle)
  }

  return 0;
}
