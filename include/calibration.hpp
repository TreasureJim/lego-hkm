#pragma once

#include <array>
#include <string>

std::array<std::array<double, 2>, 3> read_calibration_file(const std::string& file_name);
