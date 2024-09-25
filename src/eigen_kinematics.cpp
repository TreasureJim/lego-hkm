#include "kinematics.h"
#include <Eigen/Dense>
#include <array>
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
