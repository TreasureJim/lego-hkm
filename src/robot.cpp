#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <ompl-1.6/ompl/geometric/PathGeometric.h>

#define RAD_TO_DEG 57.295779513

#include "kinematics.h"
#include "mark2_0_fixed.h"
#include "motors.h"
#include "path_finding.h"
#include "robot.h"

int robot_setup() {
  if (!motor_setup()) return false;
  motor_reset_angle();
  return true;
}

void robot_shutdown() {

  double* default_angles = DEFAULT_JOINT_ANGLES;
  move_linear(get_current_cart_loc().data(),
              joint_angle_to_cart_loc(default_angles).data());
  motor_shutdown();
}

std::array<double, 3> joint_angle_to_cart_loc(const double angles[4]) {
  std::array<double, 3> cart_pos;
  double mat[4][4];
  fwd(&mark2_0_fixed, angles, mat, NULL);
  cart_pos[0] = mat[0][3];
  cart_pos[1] = mat[1][3];
  cart_pos[2] = mat[2][3];
  return cart_pos;
}

std::array<double, 3> get_current_cart_loc() {
  return joint_angle_to_cart_loc(current_joint_angles);
}

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

  for (int i = 1; i < cart_coords.size(); i++) {
    const auto &coord = cart_coords[i];
    double servo_angles[4];
    if (cart_to_drive(&mark2_0_fixed, coord.data(), 0.0, servo_angles) < 0) {
      fprintf(stderr,
              "Error generating motor position for x: %f, y: %f, z: %f.\n",
              coord[0], coord[1], coord[2]);
      continue;
    }

    printf("Cart: x: %f, y: %f, z: %f. Drive positions: %f, %f, %f\n", coord[0],
           coord[1], coord[2], servo_angles[0] * RAD_TO_DEG,
           servo_angles[1] * RAD_TO_DEG, servo_angles[2] * RAD_TO_DEG);
    motor_transition_angle(cart_coords[i - 1].data(), cart_coords[i].data());
  }

  return 0;
}
