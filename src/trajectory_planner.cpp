#include "juliet_comms.hpp"
#include "math/3d_circle.hpp"
#include <Eigen/Dense>

Eigen::Vector3d robtarget_to_vector(robtarget &r) { return Eigen::Vector3d(r.x, r.y, r.z); }

class IMotion {
  Eigen::Vector3d origin_pos;
  motion_u motion;

public:
  IMotion(Eigen::Vector3d origin_pos);
  virtual Eigen::Vector3d GetPoint(float t);
};

IMotion::IMotion(Eigen::Vector3d origin_pos) {
  this->origin_pos = origin_pos;
}

class MotionCircle: public IMotion {
Circle_3D circle_3d;

public:
  MotionCircle(movecircular circle);
};

MotionCircle::MotionCircle(Eigen::Vector3d origin_pos, movecircular circle): IMotion(origin_pos) {
  this->circle_3d = Circle_3D(origin_pos, circle.apos, circle.target);
}

Eigen::Vector3d MotionCircle::GetPoint(float t) {
  return this->circle_3d.get_circle_coord(t);
}

class MotionArc: public IMotion {
Circle_3D circle_3d;

public:
  MotionCircle(movearc circle);
};

MotionCircle::MotionCircle(Eigen::Vector3d origin_pos, movecircular circle): IMotion(origin_pos) {
  this->circle_3d = Circle_3D(origin_pos, circle.apos, circle.target);
}

Eigen::Vector3d MotionCircle::GetPoint(float t) {
  return this->circle_3d.get_arc_coord(t);
}

class MotionLinear: public IMotion {

};

class MotionPath: public IMotion {

};
