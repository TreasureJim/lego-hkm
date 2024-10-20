#include "calibration.hpp"
#include "eigen_kinematics.hpp"
#include "lego_model.hpp"
#include "motors.hpp"
#include "rc/servo.h"
#include "rc/time.h"
#include "robot.hpp"
#include <array>
#include <atomic>
#include <fcntl.h>
#include <iostream>
#include <mutex>
#include <queue>
#include <termios.h>
#include <thread>
#include <unistd.h>

#define MOTOR_CHANGE_AMOUNT 0.05

Robot *robot;
std::queue<MotionLinear> motion_queue;

double speed = 0.01;

std::mutex printing_mut;
std::mutex data_mut;
std::array<double, 3> motor_positions;

std::atomic<bool> stop;

void enableRawMode() {
	termios term;
	tcgetattr(STDIN_FILENO, &term);
	term.c_lflag &= ~(ICANON | ECHO); // Disable canonical mode and echo
	tcsetattr(STDIN_FILENO, TCSANOW, &term);
}
void disableRawMode() {
	termios term;
	tcgetattr(STDIN_FILENO, &term);
	term.c_lflag |= (ICANON | ECHO); // Enable canonical mode and echo
	tcsetattr(STDIN_FILENO, TCSANOW, &term);
}
void setNonBlockingInput() {
	int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
}

void displayValues() {
	std::cout << "\033[H";              // Set cursor to start position
	std::cout << "\033[J" << std::endl; // Clear screen

	// Write the xyz, angle and motor angle

	// motor angles
	std::cout << "Motor angles: " << motor_positions[0] << ", " << motor_positions[1] << ", " << motor_positions[2]
			  << std::endl;

	// joint angles
	double joint_angles[4];
	for (int i = 0; i < 3; i++) {
		joint_angles[i] = motor_angle_to_joint_angle(i, motor_positions[i]);
	}

	std::cout << "Joint angles: " << joint_angles[0] << ", " << joint_angles[1] << ", " << joint_angles[2] << std::endl;

	// x y z
	double matrix[4][4];
	double angle = 0.0;
	fwd(&lego_model, joint_angles, matrix, &angle);

	double cart_pos[3];
	matrix_to_pos(matrix, cart_pos);

	std::cout << "X Y Z: " << cart_pos[0] << ", " << cart_pos[1] << ", " << cart_pos[2] << std::endl;

	std::cout << "\033[" << 200 << ";" << 0 << "H"; // Move cursor to last line

	{
		// show bad IK
		auto j = inverse(Eigen::Vector3d(cart_pos[0], cart_pos[1], cart_pos[2]), &lego_model);
		if (!j.has_value())
			std::cout << "BAD" << std::endl;
	}
}

double bound_value(double x, double a, double b) {
	// Get the minimum and maximum between a and b
	double min_val = std::min(a, b);
	double max_val = std::max(a, b);

	// Return x bounded between min_val and max_val
	if (x < min_val) {
		return min_val;
	} else if (x > max_val) {
		return max_val;
	} else {
		return x;
	}
}

void user_input() {
	char input;
	while (true) {
		input = getchar();

		if (input == 'q') {
			stop = true;
		}

		{
			auto lock = std::lock_guard(data_mut);

			// Handle X-axis control
			if (input == 'w')
				motor_positions[0] += MOTOR_CHANGE_AMOUNT; // Increase speed in X
			if (input == 's')
				motor_positions[0] -= MOTOR_CHANGE_AMOUNT; // Decrease speed in X

			// Handle Y-axis control
			if (input == 'e')
				motor_positions[1] += MOTOR_CHANGE_AMOUNT; // Increase speed in Y
			if (input == 'd')
				motor_positions[1] -= MOTOR_CHANGE_AMOUNT; // Decrease speed in Y

			// Handle Z-axis control
			if (input == 'r')
				motor_positions[2] += MOTOR_CHANGE_AMOUNT; // Increase speed in Z
			if (input == 'f')
				motor_positions[2] -= MOTOR_CHANGE_AMOUNT; // Decrease speed in Z

			// limit motor values
			for (int motor = 0; motor < 3; motor++) {
				motor_positions[motor] =
					bound_value(motor_positions[motor], motor_offset_values[motor][0], motor_offset_values[motor][1]);
			}

			auto display_lock = std::lock_guard(printing_mut);
			displayValues();
		}

		usleep(100000); // Small delay for smooth input
	}
}

// Thread function to control the robot and update the position
void robotControlThread() {

	// TODO: Make robot execute motions from here
	assert(false);
}

int main(int argc, char *argv[]) {
	if (argc < 2) {
		fprintf(stderr, "Usage: %s <robot type> <calibration file>", argv[0]);
	}

	if (std::string("lego") == argv[1]) {
		robot = new LegoRobot(&lego_model);

		std::string calibration_file = "./calibration.data";
		if (argc >= 3) {
			calibration_file = argv[2];
		}
		// Load calibration data
		motor_offset_values = read_calibration_file(calibration_file);
	} else if (std::string("sim") == argv[1]) {
		robot = new FakeVisRobot();
	} else {
		fprintf(stderr, "Invalid robot picked: %s", argv[1]);
		exit(1);
	}

	// Enable RAW mode for terminal input
	enableRawMode();
	// setNonBlockingInput();

	displayValues();
	std::thread control_thread(robotControlThread);

	user_input();

	// Disable RAW mode when done
	disableRawMode();

	motor_shutdown();
	std::cout << "\nRobot control terminated." << std::endl;
	return 0;
}
