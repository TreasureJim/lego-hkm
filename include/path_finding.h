#pragma once

#include <ompl/geometric/PathGeometric.h>

void path_finding_setup();
ompl::geometric::PathGeometric* find_path(double start_pos[3], double goal_pos[3]);
void geometric_paths_to_cart_coords(ompl::geometric::PathGeometric* path, std::vector<std::array<double, 3>>& cart_coords);
