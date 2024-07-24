#pragma once

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <ompl/geometric/PathGeometric.h>

#define RAD_TO_DEG 57.295779513

#include "kinematics.h"
#include "math/3d_circle.hpp"
#include "motors.hpp"
#include "path_finding.hpp"

#include "robot.hpp"

Eigen::Vector3d joint_angle_to_cart_loc(const double angles[4]) {
  double mat[4][4];
  fwd(&mark2_0_fixed, angles, mat, NULL);

  Eigen::Vector3d cart_pos = {mat[0][3], mat[1][3], mat[2][3]};

  return cart_pos;
}

int Robot::robot_setup() {
  if (!motor_setup())
    return false;
  motor_reset_angle();
  this->cart_pos =
      Eigen::Vector3d(joint_angle_to_cart_loc(current_joint_angles));
  return true;
}

int Robot::go_to(Eigen::Vector3d pos) {
  double goal_servo_angles[4];
  if (cart_to_drive(&mark2_0_fixed, pos.data(), 0.0, goal_servo_angles) < 0) {
    fprintf(stderr,
            "ERROR: generating motor position for x: %f, y: %f, z: %f.\n",
            pos.x(), pos.y(), pos.z());
    return 0;
  }

  motor_transition_angle(current_joint_angles, goal_servo_angles);
}

void Robot::robot_shutdown() {
  double *default_angles = DEFAULT_JOINT_ANGLES;
  this->go_to(joint_angle_to_cart_loc(default_angles));
  motor_shutdown();
}

Robot::Robot() { error = robot_setup(); }

Robot::~Robot() { robot_shutdown(); }

Eigen::Vector3d Robot::get_current_cart_loc() {
  return joint_angle_to_cart_loc(current_joint_angles);
}

int Robot::move_linear(Eigen::Vector3d start_pos, Eigen::Vector3d goal_pos) {
  auto path = find_path(start_pos, goal_pos);
  if (!path.has_value()) {
    fprintf(stderr, "ERROR: Could not plan path.\n");
    return 0;
  }

  for (const auto &coord : *path) {
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
    this->go_to(coord);
  }

  return 0;
}

int Robot::move_radial(double p1[3], double p2[3], double p3[3]) {
  Eigen::Vector3d v1(p1[0], p1[1], p1[2]);
  Eigen::Vector3d v2(p2[0], p2[1], p2[2]);
  Eigen::Vector3d v3(p3[0], p3[1], p3[2]);

  Circle_3D circle(v1, v2, v3);
  if (!circle)
    return 0;

  for (float p = 0.0; p <= 1.0; p += 0.5) {
    this->go_to(circle.get_arc_coord(p));
  }

  return 1;
}
