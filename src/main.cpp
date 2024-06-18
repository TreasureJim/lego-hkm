#include <cstdio>
#include <cstdlib>
#include <ompl-1.6/ompl/geometric/PathGeometric.h>
#include <string>

#define RAD_TO_DEG 57.295779513

#include "kinematics.h"
#include "mark2_0.h"
#include "path_finding.h"

void print_usage() {
  printf("===== USAGE =====\n"
         "./LegoHKM x y z\n\n"
         "x, y, z are doubles which represent the goal co-ordinates for the "
         "TCP.\n");

  exit(-1);
}

int main(int argc, char *argv[]) {
  if (argc < 4)
    print_usage();
  // read in goal position
  double goal_pos[3];
  try {
    goal_pos[0] = std::stod(argv[1]);
    goal_pos[1] = std::stod(argv[2]);
    goal_pos[2] = std::stod(argv[3]);
  } catch (int e) {
    print_usage();
  }

  path_finding_setup();

  double start_pos[3] = {0.0};

  ompl::geometric::PathGeometric *path = find_path(start_pos, goal_pos);
  if (!path) {
    return 0;
  }

  double tool_offset[3] = {0.0};

  double servo_angles[4];
  cart_to_drive_posonly(&mark2_0, goal_pos, 0.0, tool_offset, servo_angles);

  printf("Drive positions: %f, %f, %f", servo_angles[0] * RAD_TO_DEG,
         servo_angles[1] * RAD_TO_DEG, servo_angles[2] * RAD_TO_DEG);

  return 0;
}
