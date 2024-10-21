#include "kinematics.h"
#include <Eigen/Dense>
#include <array>
#include <cstring>
#include <optional>

std::array<double, 3> pos_to_array(const Eigen::Vector3d &vec) {
	std::array<double, 3> arr;
	arr[0] = vec.x();
	arr[1] = vec.y();
	arr[2] = vec.z();
	return arr;
}

std::optional<std::array<double, 4>> inverse(Eigen::Vector3d pos, agile_pkm_model* model) {
	std::array<double, 4> joints;
	auto pos_arr = pos_to_array(pos);

	if (inv(model, pos_arr.data(), 0.0, joints.data()) < 0)
		return std::nullopt;

	return joints;
}

static void matrix_to_pos(double matrix[4][4], double pos[3]) {
	pos[0] = matrix[0][3];
	pos[1] = matrix[1][3];
	pos[2] = matrix[2][3];
}

Eigen::Vector3d forward(double joints[3], const struct agile_pkm_model *rob) {
	double joint_angles[4];
	memcpy(joint_angles, joints, sizeof(double) * 3);

	double matrix[4][4];
	double angle = 0.0;
	fwd(rob, joint_angles, matrix, &angle);

	double cart_pos[3];
	matrix_to_pos(matrix, cart_pos);

	return Eigen::Vector3d(cart_pos[0], cart_pos[1], cart_pos[2]);
}
