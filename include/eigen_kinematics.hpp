#pragma once

#include "kinematics.h"
#include <Eigen/Dense>
#include <array>
#include <optional>

std::array<double, 3> pos_to_array(const Eigen::Vector3d &vec);
std::optional<std::array<double, 4>> inverse(Eigen::Vector3d pos, agile_pkm_model* model);
