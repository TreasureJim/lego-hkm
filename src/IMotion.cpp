#include "juliet_comms.hpp"
#include "kinematics.h"
#include "math/3d_circle.hpp"
#include "motion_types.h"
#include "pathfinding.hpp"
#include <Eigen/Dense>
#include <array>
#include <cassert>
#include <cstring>
#include <vector>
#include "eigen_kinematics.hpp"

#include "IMotion.hpp"

Eigen::Vector3d robtarget_to_vector(robtarget &r) { return Eigen::Vector3d(r.x, r.y, r.z); }

std::array<double, 4> hkmpos_to_array(hkmpos pos) {
	std::array<double, 4> arr;
	arr[0] = pos.j1;
	arr[1] = pos.j2;
	arr[2] = pos.j3;
	arr[3] = pos.j4;
	return arr;
}

Eigen::Vector3d array_to_pos(std::array<double, 4> joints) {
	double pos[3];
	cart_to_drive(&lego_model, pos, 0.0, joints.data());
	Eigen::Vector3d vec(pos);
	return vec;
}

IMotion::IMotion(uint8_t uuid[16], Eigen::Vector3d origin_pos, Eigen::Vector3d target_pos)
	: origin_pos(origin_pos), target_pos(target_pos) {
	memcpy(this->uuid, uuid, sizeof(this->uuid));
}

IMotion *IMotion::motion_com_to_IMotion(Eigen::Vector3d origin_pos, motion_command command) {
	IMotion *motion;

	switch (command.type) {
	case MOVE_POS: {
		motion = new MotionPath(origin_pos, command.motion.pos);
	}
	case MOVE_LIN: {
		motion = new MotionLinear(origin_pos, command.motion.linear);
	}
	case MOVE_ARC: {
		motion = new MotionArc(origin_pos, command.motion.arc);
	}
	case MOVE_CIRC: {
		motion = new MotionCircle(origin_pos, command.motion.circular);
	}
	case MOVE_JOINT: {
		motion = new MotionJoint(origin_pos, command.motion.joint);
	}
	default: {
		throw "[ERROR] Unrecognised motion command. ";
	}
	}

	return motion;
}

Eigen::Vector3d IMotion::get_target_pos() {
	return this->target_pos;
}

// MotionCircle

MotionCircle::MotionCircle(Eigen::Vector3d origin_pos, movecircular circle)
	: IMotion(circle.motion_id, origin_pos, robtarget_to_vector(circle.target)),
	  circle_3d(origin_pos, robtarget_to_vector(circle.apos), robtarget_to_vector(circle.target)) {}

Eigen::Vector3d MotionCircle::GetPoint(float t) { return this->circle_3d.get_circle_coord(t * 2 * M_PI); }

bool MotionCircle::is_valid() { return this->circle_3d.check_circle_valid_path(); }

// MotionArc

MotionArc::MotionArc(Eigen::Vector3d origin_pos, movearc arc)
	: IMotion(arc.motion_id, origin_pos, robtarget_to_vector(arc.target)),
	  circle_3d(origin_pos, robtarget_to_vector(arc.apos), robtarget_to_vector(arc.target)) {}

Eigen::Vector3d MotionArc::GetPoint(float t) { return this->circle_3d.get_arc_coord(t); }

bool MotionArc::is_valid() { return this->circle_3d.check_arc_valid_path(); }

// MotionLinear

MotionLinear::MotionLinear(Eigen::Vector3d origin_pos, movelinear linear_com) : IMotion(linear_com.motion_id, origin_pos, robtarget_to_vector(linear_com.target)) {}

Eigen::Vector3d MotionLinear::GetPoint(float t) { return this->origin_pos + (this->target_pos - this->origin_pos) * t; }

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

MotionPath::MotionPath(Eigen::Vector3d origin_pos, movepos path_com)
	: IMotion(path_com.motion_id, origin_pos, robtarget_to_vector(path_com.target)), path(PathFinding().find_path(origin_pos, robtarget_to_vector(path_com.target)).value()) {
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

// MotionJoint

MotionJoint::MotionJoint(Eigen::Vector3d origin_pos, movejoint joint_com)
	: IMotion(joint_com.motion_id, origin_pos, array_to_pos(hkmpos_to_array(joint_com.target))), angles(hkmpos_to_array(joint_com.target)) {}

std::array<double, 4> MotionJoint::get_angles() { return this->angles; };

bool MotionJoint::is_valid() {
	return inverse(this->target_pos).has_value();
}

Eigen::Vector3d MotionJoint::GetPoint(float t) {
 return this->origin_pos + (this->target_pos - this->origin_pos) * t; 
}
