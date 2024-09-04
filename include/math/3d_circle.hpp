#pragma once

#include "kinematics.h"
#include "lego_model.hpp"
#include <Eigen/Geometry>
#include <Eigen/src/Core/Matrix.h>
#include <array>
#include <cmath>

class Circle_3D {
  private:
	std::array<Eigen::Vector3d, 3> points;

	Eigen::Vector3d origin_3d;
	double radius;
	Eigen::Vector3d normal;

	double arc_start_angle;
	double arc_end_angle;
	// Normalised vector on the plane
	Eigen::Vector3d plane_vec1;
	// Another normalised vector on the plane that is orthogonal to the first
	// (basis1)
	Eigen::Vector3d plane_vec2;

	Eigen::Vector3d find_plane_normal(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3) {
		Eigen::Vector3d v1 = p2 - p1;
		Eigen::Vector3d v2 = p3 - p1;
		Eigen::Vector3d normal = v1.cross(v2);

		normal /= normal.norm();
		return normal;
	}

	/// Given 3 coordinates of points on a circle
	/// Finds radius of circle and assigns it to self.radius
	/// Returns the 2D centre of the the circle
	Eigen::Vector2d find_circle_params(Eigen::Vector2d p1, Eigen::Vector2d p2, Eigen::Vector2d p3) {
		double x1 = p1.x(), y1 = p1.y();
		double x2 = p2.x(), y2 = p2.y();
		double x3 = p3.x(), y3 = p3.y();

		auto a1 = 2 * (x1 - x2);
		auto b1 = 2 * (y1 - y2);
		auto c1 = pow(x1, 2) + pow(y1, 2) - pow(x2, 2) - pow(y2, 2);

		auto a2 = 2 * (x1 - x3);
		auto b2 = 2 * (y1 - y3);
		auto c2 = pow(x1, 2) + pow(y1, 2) - pow(x3, 2) - pow(y3, 2);

		auto denominator = a1 * b2 - a2 * b1;
		auto h = (c1 * b2 - c2 * b1) / denominator;
		auto k = (a1 * c2 - a2 * c1) / denominator;

		this->radius = sqrt(pow((x1 - h), 2) + pow(y1 - k, 2));
		return Eigen::Vector2d(h, k);
	}

	void find_circle_centre_radius_3d() {
		Eigen::Vector3d p1 = this->points[0];
		Eigen::Vector3d p2 = this->points[1];
		Eigen::Vector3d p3 = this->points[2];

		this->normal = find_plane_normal(this->points[0], this->points[1], this->points[2]);

		/// origin of the plane
		/// can use one of the points
		auto origin = p1;
		this->plane_vec1 = (p2 - p1) / (p2 - p1).norm();
		this->plane_vec2 = normal.cross(this->plane_vec1) / normal.cross(this->plane_vec1).norm();

		// desmos visualisation: https://www.desmos.com/3d/chdszznbfw
		auto to_2d = [&](Eigen::Vector3d point) -> Eigen::Vector2d {
			return Eigen::Vector2d((point - origin).dot(this->plane_vec1), (point - origin).dot(this->plane_vec2));
		};

		Eigen::Vector2d p1_2d(0, 0);
		Eigen::Vector2d p2_2d = to_2d(p2);
		Eigen::Vector2d p3_2d = to_2d(p3);

		// get 2d centre of circle
		Eigen::Vector2d centre_2d = find_circle_params(p1_2d, p2_2d, p3_2d);

		// convert 2d to 3d coordinate
		this->origin_3d = origin + centre_2d[0] * this->plane_vec1 + centre_2d[1] * this->plane_vec2;
	}

	void paramtric_arc_3d() {
		Eigen::Vector3d v1 = this->points[0] - this->origin_3d;
		Eigen::Vector3d v2 = this->points[1] - this->origin_3d;
		Eigen::Vector3d v3 = this->points[2] - this->origin_3d;

		auto angle_between = [](Eigen::Vector3d v1, Eigen::Vector3d v2) {
			return atan2(v1.cross(v2).norm(), v1.dot(v2));
		};

		double angle_12 = angle_between(v1, v2);
		double angle_23 = angle_between(v2, v3);
		double angle_31 = angle_between(v3, v1);

		std::array<Eigen::Vector3d, 3> order;

		if (angle_12 + angle_23 > angle_23 + angle_31 and angle_12 + angle_23 > angle_31 + angle_12)
			order = {v1, v2, v3};
		else if (angle_23 + angle_31 > angle_12 + angle_23)
			order = {v2, v3, v1};
		else
			order = {v3, v1, v2};

		auto start_angle = atan2(order[0].dot(this->plane_vec2), order[0].dot(this->plane_vec1));
		auto mid_angle = atan2(order[1].dot(this->plane_vec2), order[1].dot(this->plane_vec1));
		auto end_angle = atan2(order[2].dot(this->plane_vec2), order[2].dot(this->plane_vec1));

		// Ensure the arc goes the longer way around
		if (mid_angle < start_angle)
			mid_angle += 2 * M_PI;
		if (end_angle < mid_angle)
			end_angle += 2 * M_PI;

		this->arc_start_angle = start_angle;
		this->arc_end_angle = end_angle;
	}

  public:
	Circle_3D(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3) {
		this->points = {p1, p2, p3};
		this->find_circle_centre_radius_3d();
		this->paramtric_arc_3d();
	}

	/// float `t` between 0.0 and (2 * M_PI) - represents how far along the arc in radians
	/// Returns: gives the coordinate of a point on the circle given `t`
	Eigen::Vector3d get_circle_coord(float t) {
		return this->origin_3d + this->radius * (cos(t) * this->plane_vec1 + sin(t) * this->plane_vec2);
	}

	/// float `p` between 0.0 and 1.0 - represents how far along the arc
	/// Returns: gives the coordinate of the arc given `t`
	Eigen::Vector3d get_arc_coord(float p) {
		return this->get_circle_coord(this->arc_start_angle + (arc_end_angle - arc_start_angle) * p);
	}

	bool check_circle_valid_path() {
		for (float p = 0.0; p <= 1.0; p += 0.05) {
			auto vec = this->get_circle_coord(p);
			double pos[3] = {vec.x(), vec.y(), vec.z()};
			if (inv(&lego_model, pos, 0.0, NULL) < 0) {
				return false;
			}
		}

		return true;
	}

	bool check_arc_valid_path() {
		for (float p = 0.0; p <= 1.0; p += 0.05) {
			auto vec = this->get_arc_coord(p);
			double pos[3] = {vec.x(), vec.y(), vec.z()};
			if (inv(&lego_model, pos, 0.0, NULL) < 0) {
				return false;
			}
		}

		return true;
	}
};
