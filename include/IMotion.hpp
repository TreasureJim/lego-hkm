#pragma once

#include "math/3d_circle.hpp"
#include <Eigen/Eigen>
#include <Eigen/src/Core/Matrix.h>

extern "C" {
#include "motion_types.h"
}

std::array<double, 4> hkmpos_to_array(hkmpos pos);
Eigen::Vector3d array_to_pos(std::array<double, 4> joints, agile_pkm_model* model);

enum motiontype_e {
	MOVE_POS,
	MOVE_LIN,
	MOVE_ARC,
	MOVE_CIRC,
	MOVE_JOINT,
};

union motion_u {
	movepos pos;
	movelinear linear;
	movearc arc;
	movecircular circular;
	movejoint joint;
};

struct motion_command {
	motiontype_e type;
	motion_u motion;
};

class IMotion {
  protected:
	struct agile_pkm_model* model;

	const Eigen::Vector3d origin_pos;
	const Eigen::Vector3d target_pos;

	IMotion(uint8_t uuid[16], Eigen::Vector3d origin_pos, Eigen::Vector3d target_pos, agile_pkm_model* model);

  public:
	uint8_t uuid[16];

	static IMotion *motion_com_to_IMotion(Eigen::Vector3d origin_pos, motion_command command, agile_pkm_model* model);
	Eigen::Vector3d get_target_pos();

	virtual Eigen::Vector3d GetPoint(float t) = 0;
	virtual bool is_valid() = 0;
};

class MotionCircle : public IMotion {
	Circle_3D circle_3d;

  public:
	MotionCircle(Eigen::Vector3d origin_pos, movecircular circle, agile_pkm_model* model);
	Eigen::Vector3d GetPoint(float t) override;
	bool is_valid() override;
};

class MotionArc : public IMotion {
	Circle_3D circle_3d;

  public:
	MotionArc(Eigen::Vector3d origin_pos, movearc arc, agile_pkm_model* model);
	Eigen::Vector3d GetPoint(float t) override;
	bool is_valid() override;
};

class MotionLinear : public IMotion {
  public:
	MotionLinear(Eigen::Vector3d origin_pos, movelinear linear_com, agile_pkm_model* model);
	Eigen::Vector3d GetPoint(float t) override;
	bool is_valid() override;
};

class MotionPath : public IMotion {
	std::vector<Eigen::Vector3d> path;
	float total_path_size = 0.0;
	// Starts at the first path point. Eg. one after the origin (path[[1]])
	std::vector<float> point_percentages;

  public:
	MotionPath(Eigen::Vector3d origin_pos, movepos path_com, agile_pkm_model* model);
	Eigen::Vector3d GetPoint(float t) override;
	bool is_valid() override;
};

class MotionJoint : public IMotion {
	std::array<double, 4> angles;

  public:
	MotionJoint(Eigen::Vector3d origin_pos, movejoint joint_com, agile_pkm_model* model);
	Eigen::Vector3d GetPoint(float t) override;
	std::array<double, 4> get_angles();
	bool is_valid() override;
};
