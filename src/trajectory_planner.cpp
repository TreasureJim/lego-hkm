#include "IMotion.hpp"
#include <Eigen/Dense>
extern "C" {
#include "motion_types.h"
}

Eigen::Vector3d interpolate_points(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3, double t) {
	Eigen::Vector3d v1 = p2 - p1;
	Eigen::Vector3d v2 = p3 - p2;

	Eigen::Vector3d a = p1 + t * v1;
	Eigen::Vector3d b = p2 + t * v2;

	Eigen::Vector3d v3 = b - a;
	return a + t * v3;
}

Eigen::Vector3d blend_movements(double t, IMotion &current_movement, IMotion &next_movement, blendvalue &blend) {
	// TODO
	assert(false);
	return Eigen::Vector3d(0,0,0);
}
