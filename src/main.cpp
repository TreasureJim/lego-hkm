#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <ompl-1.6/ompl/geometric/PathGeometric.h>
#include <random>
#include <string>

#define RAD_TO_DEG 57.295779513

#include "kinematics.h"
#include "mark2_0_fixed.h"
#include "path_finding.h"
#include "spacial_conv.h"

void print_usage() {
  printf("===== USAGE =====\n"
         "./LegoHKM x y z\n\n"
         "x, y, z are doubles which represent the cartesian goal co-ordinates "
         "for the TCP.\n");

  exit(-1);
}

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

/// `path_finding_setup` needs to be run first before running this
int move_linear(double start_pos[3], double goal_pos[3]) {
  // if (argc < 4)
  //   print_usage();
  // // read in goal position
  // double goal_pos_cart[3];
  // try {
  //   goal_pos_cart[0] = std::stod(argv[1]);
  //   goal_pos_cart[1] = std::stod(argv[2]);
  //   goal_pos_cart[2] = std::stod(argv[3]);
  // } catch (int e) {
  //   print_usage();
  // }
  //
  // double goal_pos_cyl[3];
  // cart_to_cyl_pol(goal_pos_cart, goal_pos_cyl);

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
  }

  return 0;
}


