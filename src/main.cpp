#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ompl-1.6/ompl/geometric/PathGeometric.h>
#include <string>

#define RAD_TO_DEG 57.295779513

#include "kinematics.h"
#include "mark2_0.h"
#include "path_finding.h"
#include "spacial_conv.h"

void print_usage() {
  printf("===== USAGE =====\n"
         "./LegoHKM x y z\n\n"
         "x, y, z are doubles which represent the cartesian goal co-ordinates "
         "for the TCP.\n");

  exit(-1);
}

int main(int argc, char *argv[]) {
  if (argc < 4)
    print_usage();
  // read in goal position
  double goal_pos_cart[3];
  try {
    goal_pos_cart[0] = std::stod(argv[1]);
    goal_pos_cart[1] = std::stod(argv[2]);
    goal_pos_cart[2] = std::stod(argv[3]);
  } catch (int e) {
    print_usage();
  }

  double goal_pos_cyl[3];
  cart_to_cyl_pol(goal_pos_cart, goal_pos_cyl);

  path_finding_setup();

  double start_pos[3] = {0.02, 0.0, 0.0};

  ompl::geometric::PathGeometric *path = find_path(start_pos, goal_pos_cart);
  if (!path) {
    return 0;
  }
  std::vector<std::array<double, 3>> cart_coords;
  geometric_paths_to_cart_coords(path, cart_coords);

  // path->print(std::cout);

  for (const auto &coord : cart_coords) {
    double servo_angles[4];
    int result = cart_to_drive(&mark2_0, coord.data(), 0.0, servo_angles);
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

  for (double j1 = 0.0; j1 < 2 * M_PI; j1 += 0.1) {
    for (double j2 = 0.0; j2 < 2 * M_PI; j2 += 0.1) {
      for (double j3 = 0.0; j3 < 2 * M_PI; j3 += 0.1) {
        for (double j4 = 0.0; j4 < 2 * M_PI; j4 += 0.1) {
          double j_angles[4] = {j1, j2, j3, j4};
          double matrix[4][4];
          double orient_angle = 0.0;
          int result = fwd(&mark2_0, j_angles, matrix, &orient_angle);

          if (result < 0) {
            fprintf(stderr, "ERROR: gen forward kin for joint angles: %f, %f, %f, %f\n", j1, j2, j3, j4);
            continue;
          } else {
            double coords[3] = {matrix[0][3], matrix[1][3], matrix[2][3]};
            double servo_angles[4];
            if (cart_to_drive(&mark2_0, coords, 0.0, servo_angles) < 0) {
              fprintf(stderr, "ERROR: couldn't perform inv kin.\n");
              printf("Drive positions: %f, %f, %f, %f\t Cart: x: %f, y: %f, z: %f.\n", j1, j2, j3, j4, coords[0], coords[1], coords[2]);
            }
          }
        }
      }
    }
  }

  return 0;
}
