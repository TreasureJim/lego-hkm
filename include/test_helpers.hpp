#pragma once

extern "C" {
#include "kinematics.h"
}

#include <Eigen/Dense>

Eigen::Vector3d random_valid_cart_pos(agile_pkm_model* model);
