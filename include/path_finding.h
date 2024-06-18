#pragma once

#include <ompl-1.6/ompl/geometric/PathGeometric.h>

void path_finding_setup();
ompl::geometric::PathGeometric* find_path(double start_pos[3], double goal_pos[3]);
