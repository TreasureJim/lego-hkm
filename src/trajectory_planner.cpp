#include "pathfinding.hpp"
#include "juliet_comms.hpp"
#include "math/3d_circle.hpp"
#include <Eigen/Dense>
#include <vector>

Eigen::Vector3d robtarget_to_vector(robtarget &r) { return Eigen::Vector3d(r.x, r.y, r.z); }

class IMotion {
protected:
	const Eigen::Vector3d origin_pos;

public:
	IMotion(Eigen::Vector3d origin_pos);
	virtual Eigen::Vector3d GetPoint(float t);
};

IMotion::IMotion(Eigen::Vector3d origin_pos): origin_pos(origin_pos) {}

class MotionCircle : public IMotion {
	Circle_3D circle_3d;

  public:
	MotionCircle(Eigen::Vector3d origin_pos, movecircular circle);
	Eigen::Vector3d GetPoint(float t) override;
};

MotionCircle::MotionCircle(Eigen::Vector3d origin_pos, movecircular circle)
	: IMotion(origin_pos), circle_3d(origin_pos, robtarget_to_vector(circle.apos), robtarget_to_vector(circle.target)) {
}

Eigen::Vector3d MotionCircle::GetPoint(float t) { return this->circle_3d.get_circle_coord(t); }

class MotionArc : public IMotion {
	Circle_3D circle_3d;

  public:
	MotionArc(Eigen::Vector3d origin_pos, movecircular circle);
  Eigen::Vector3d GetPoint(float t) override;
};

MotionArc::MotionArc(Eigen::Vector3d origin_pos, movecircular circle)
	: IMotion(origin_pos), circle_3d(origin_pos, robtarget_to_vector(circle.apos), robtarget_to_vector(circle.target)) {
}

Eigen::Vector3d MotionArc::GetPoint(float t) { return this->circle_3d.get_arc_coord(t); }

class MotionLinear : public IMotion {
  Eigen::Vector3d target;

public:
  MotionLinear(Eigen::Vector3d origin_pos, Eigen::Vector3d target);
  Eigen::Vector3d GetPoint(float t) override;
};

MotionLinear::MotionLinear(Eigen::Vector3d origin_pos, Eigen::Vector3d target): IMotion(origin_pos), target(target) {}

Eigen::Vector3d MotionLinear::GetPoint(float t) {
  return this->origin_pos + (this->target - this->origin_pos) * t;
}

class MotionPath : public IMotion {
  std::vector<Eigen::Vector3d> path;
  std::vector<float> path_percentages;

public:
  MotionPath(Eigen::Vector3d origin_pos, Eigen::Vector3d target);
  Eigen::Vector3d GetPoint(float t) override;
};

MotionPath::MotionPath(Eigen::Vector3d origin_pos, Eigen::Vector3d target): IMotion(origin_pos), path(PathFinding().find_path(origin_pos, target).value()) {
  if (this->path.empty()) throw "No path found.";

  float total_size = 0.0; 
  for (int i = 1; i < this->path.size(); i++) {
    this->total_size += (this->path[i] - this->path[i - 1]).norm();
    this->path_percentages.push_back(this->total_size);
  }
}

Eigen::Vector3d MotionPath::GetPoint(float t) {
  float distance = this->total_size * t;
  
  // Find how far along in the path
  int path_i;
  for (path_i = 0; path_i < this->path_percentages.size(); path_i++) {
    if (this->path_percentages[path_i] > distance) continue;
    
    break;
  }

  return (this->path[i + 1] - this->path[i]) * 
}
