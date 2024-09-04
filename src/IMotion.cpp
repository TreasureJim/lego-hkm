#include "juliet_comms.hpp"
#include "math/3d_circle.hpp"
#include "pathfinding.hpp"
#include <Eigen/Dense>
#include <cassert>
#include <vector>

#include "IMotion.hpp"

Eigen::Vector3d robtarget_to_vector(robtarget &r) { return Eigen::Vector3d(r.x, r.y, r.z); }

IMotion::IMotion(Eigen::Vector3d origin_pos) : origin_pos(origin_pos) {}

// MotionCircle

MotionCircle::MotionCircle(Eigen::Vector3d origin_pos, movecircular circle)
	: IMotion(origin_pos), circle_3d(origin_pos, robtarget_to_vector(circle.apos), robtarget_to_vector(circle.target)) {
}

Eigen::Vector3d MotionCircle::GetPoint(float t) { return this->circle_3d.get_circle_coord(t * 2 * M_PI); }

bool MotionCircle::is_valid() { return this->circle_3d.check_circle_valid_path(); }

// MotionArc

MotionArc::MotionArc(Eigen::Vector3d origin_pos, movecircular circle)
	: IMotion(origin_pos), circle_3d(origin_pos, robtarget_to_vector(circle.apos), robtarget_to_vector(circle.target)) {
}

Eigen::Vector3d MotionArc::GetPoint(float t) { return this->circle_3d.get_arc_coord(t); }

bool MotionArc::is_valid() { return this->circle_3d.check_arc_valid_path(); }

// MotionLinear

MotionLinear::MotionLinear(Eigen::Vector3d origin_pos, Eigen::Vector3d target) : IMotion(origin_pos), target(target) {}

Eigen::Vector3d MotionLinear::GetPoint(float t) { return this->origin_pos + (this->target - this->origin_pos) * t; }

bool MotionLinear::is_valid() {
	for (float p = 0.0; p <= 1.0; p += 0.05) {
		auto vec = this->GetPoint(p);
		double pos[3] = {vec.x(), vec.y(), vec.z()};
		if (inv(&lego_model, pos, 0.0, NULL) < 0) {
			return false;
		}
	}

	return true;
}

// MotionPath

MotionPath::MotionPath(Eigen::Vector3d origin_pos, Eigen::Vector3d target)
	: IMotion(origin_pos), path(PathFinding().find_path(origin_pos, target).value()) {
	for (int i = 1; i < this->path.size(); i++) {
		this->total_path_size += (this->path[i] - this->path[i - 1]).norm();
	}

	float so_far = 0.0;
	for (int i = 1; i < this->path.size(); i++) {
		float length = (this->path[i] - this->path[i - 1]).norm();
		so_far += length;
		this->point_percentages.push_back(so_far / this->total_path_size);
	}
}

Eigen::Vector3d MotionPath::GetPoint(float t) {
	float distance = this->total_path_size * t;

	// Find how far along in the path
	int path_i;
	for (path_i = 0; path_i < this->point_percentages.size(); path_i++) {
		if (this->point_percentages[path_i] > distance)
			continue;

		break;
	}

	return this->path[path_i] + (this->path[path_i + 1] - this->path[path_i]) * t;
}

bool MotionPath::is_valid() { return !this->path.empty(); } 
