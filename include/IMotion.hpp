#pragma once

#include "math/3d_circle.hpp"
#include <Eigen/Eigen>

extern "C" {
#include "motion_types.h"
}

class IMotion {
  protected:
	const Eigen::Vector3d origin_pos;

	IMotion(Eigen::Vector3d origin_pos);

  public:
	virtual Eigen::Vector3d GetPoint(float t) = 0;
	virtual bool is_valid() = 0;
};

class MotionCircle : public IMotion {
	Circle_3D circle_3d;

  public:
	MotionCircle(Eigen::Vector3d origin_pos, movecircular circle);
	Eigen::Vector3d GetPoint(float t) override;
	bool is_valid() override;
};

class MotionArc : public IMotion {
	Circle_3D circle_3d;

  public:
	MotionArc(Eigen::Vector3d origin_pos, movecircular circle);
	Eigen::Vector3d GetPoint(float t) override;
	bool is_valid() override;
};

class MotionLinear : public IMotion {
	Eigen::Vector3d target;

  public:
	MotionLinear(Eigen::Vector3d origin_pos, Eigen::Vector3d target);
	Eigen::Vector3d GetPoint(float t) override;
	bool is_valid() override;
};

class MotionPath : public IMotion {
	std::vector<Eigen::Vector3d> path;
	float total_path_size = 0.0;
	// Starts at the first path point. Eg. one after the origin (path[[1]])
	std::vector<float> point_percentages;

  public:
	MotionPath(Eigen::Vector3d origin_pos, Eigen::Vector3d target);
	Eigen::Vector3d GetPoint(float t) override;
	bool is_valid() override;
};
