#include "calibration.hpp"
#include "eigen_kinematics.hpp"
#include "lego_model.hpp"
#include "motors.hpp"
#include <iostream>

int main(int argc, char *argv[]) {
	motor_offset_values = read_calibration_file("./calibration.data");

	// to motor
	std::array<double, 3> initial_coords;
	std::array<double, 4> initial_joint_vals;
	double motor_positions[3];
	{
		initial_coords[0] = std::stod(argv[1]);
		initial_coords[1] = std::stod(argv[2]);
		initial_coords[2] = std::stod(argv[3]);
		auto j = inverse(Eigen::Vector3d(initial_coords[0], initial_coords[1], initial_coords[2]), &lego_model);

		if (!j.has_value()) {
			std::cout << "Could not IK initial value!" << std::endl;
			exit(1);
		} else
			initial_joint_vals = j.value();

		for (int i = 0; i < 3; i++) {
			motor_positions[i] = joint_angle_to_libservo_value(initial_joint_vals[i], i);
		}
	}

	// to cartesian
	std::array<double, 3> output_coords;
	std::array<double, 4> output_joint_vals;
	{
		for (int i = 0; i < 3; i++) {
			output_joint_vals[i] = motor_angle_to_joint_angle(i, motor_positions[i]);
		}

		// x y z
		double matrix[4][4];
		double angle = 0.0;
		fwd(&lego_model, motor_positions, matrix, &angle);

		matrix_to_pos(matrix, output_coords.data());
	}

	if (initial_joint_vals != output_joint_vals) {
		std::cout << "FAILURE JOINTS: DO NOT MATCH!" << std::endl;
		std::cout << "Initial Joint angles: " << initial_joint_vals[0] << ", " << initial_joint_vals[1] << ", "
				  << initial_joint_vals[2] << std::endl;
		std::cout << "Output Joint angles:  " << output_joint_vals[0] << ", " << output_joint_vals[1] << ", "
				  << output_joint_vals[2] << std::endl;
	}

	if (initial_coords != output_coords) {
		std::cout << "FAILURE COORDS: DO NOT MATCH!" << std::endl;
		std ::cout << "INPUT X Y Z:  " << initial_coords[0] << ", " << initial_coords[1] << ", " << initial_coords[2] << std::endl;
		std ::cout << "OUTPUT X Y Z: " << output_coords[0] << ", " << output_coords[1] << ", " << output_coords[2] << std::endl;
		return 1;
	} else {
		std::cout << "SUCCESS!" << std::endl;
		return 0;
	}

	return 0;
}
