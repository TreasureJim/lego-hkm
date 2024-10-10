#include "calibration.hpp"
#include <fstream>
#include <iostream>
#include <stdexcept>

std::array<std::array<double, 2>, 3> read_calibration_file(const std::string& file_name) {
    std::ifstream file(file_name);

    if (!file.is_open()) {
        throw std::runtime_error("Failed to open calibration file: " + file_name);
    }

    std::array<std::array<double, 2>, 3> data;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 2; ++j) {
            if (!(file >> data[i][j])) {
                throw std::runtime_error("Failed to read a double value from file.");
            }
        }
    }

    file.close();
    return data;
}
