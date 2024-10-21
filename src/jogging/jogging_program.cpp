#include "calibration.hpp"
#include "lego_model.hpp"
#include "motors.hpp"
#include "robot.hpp"
#include <array>
#include <atomic>
#include <condition_variable>
#include <csignal>
#include <fcntl.h>
#include <iostream>
#include <mutex>
#include <queue>
#include <termios.h>
#include <thread>
#include <unistd.h>

extern "C" {
#include "motion_types.h"
}

agile_pkm_model *model;
double speed = 0.01;

enum E_MODE { MODE_JOINT, MODE_COORD, MODE_END };
enum E_MODE mode = MODE_COORD;

std::mutex robot_mut;

std::mutex data_mut;
std::queue<MotionLinear> motion_queue;
std::condition_variable motion_queue_trigger;
Eigen::Vector3d last_target_pos;

std::atomic<bool> last_motion_valid = true;
std::atomic<bool> stop;

void handle_sig(int sig) {
	stop = true;
	motion_queue_trigger.notify_all();
}

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

void displayValues(Robot *robot) {
	std::cout << "\033[H"; // Set cursor to start position
	std::cout << "\033[J" << std::endl; // Clear screen

	// joint angles
	auto joint_angles = robot->get_current_joint_angles();
	std::cout << "Joint angles: " << joint_angles[0] << ", " << joint_angles[1] << ", " << joint_angles[2] << std::endl;

	// x y z
	auto cart_pos = robot->get_current_cart_loc();
	std::cout << "X Y Z: " << cart_pos.x() << ", " << cart_pos.y() << ", " << cart_pos.z() << std::endl;

	std::cout << "\033[" << 200 << ";" << 0 << "H"; // Move cursor to last line
	if (!last_motion_valid) 
		std::cout << "Motion Invalid!" << std::endl;
}

char valid_keys[] = "q-_=+mwsedrf";

void user_input() {
	while (!stop) {
		char input = getchar();

		if (input == -1 || strchr(valid_keys, input) == NULL) {
			usleep(50000); 
			continue;
		}

		if (input == 'q') {
			handle_sig(1);
			break;
		}

		if (input == '-' || input == '_') {
			speed -= 0.01;
			continue;
		}
		if (input == '=' || input == '+') {
			speed += 0.01;
			continue;
		}

		if (input == 'm') {
			mode = (E_MODE)((mode + 1) % MODE_END);
			std::cout << "Mode: " << std::to_string(mode) << std::endl;
			continue;
		}

		// create motion
		movejog jog = {0};
		switch (mode) {
		case MODE_COORD: {

			if (input == 'w')
				jog.x += speed;
			if (input == 's')
				jog.x -= speed;

			if (input == 'e')
				jog.y += speed;
			if (input == 'd')
				jog.y -= speed;

			if (input == 'r')
				jog.z += speed;
			if (input == 'f')
				jog.z -= speed;

			break;
		}

		case MODE_JOINT: {
			if (input == 'w')
				jog.j1 += speed;
			if (input == 's')
				jog.j1 -= speed;

			if (input == 'e')
				jog.j2 += speed;
			if (input == 'd')
				jog.j2 -= speed;

			if (input == 'r')
				jog.j3 += speed;
			if (input == 'f')
				jog.j3 -= speed;

			break;
		}
		default:
			assert(false);
		}

		MotionLinear motion = MotionLinear::jog_to_linear(last_target_pos, jog, model);
		bool valid_motion = motion.is_valid();
		last_motion_valid = valid_motion;
		if (!valid_motion) {
			continue;
		}

		last_target_pos = motion.GetPoint(1.0);

		{
			auto lock = std::lock_guard(data_mut);
			motion_queue.push(motion);
		}
		motion_queue_trigger.notify_one();

		usleep(100000); // Small delay for smooth input
	}
}

bool execute_command(Robot &robot, IMotion &motion) {
	robot.execute_motion(motion);
	displayValues(&robot);

	return true;
}

void robot_thread_func(Robot *robot) {
	while (!stop) {
		std::optional<MotionLinear> motion;

		// check if queue is empty
		{
			std::unique_lock queue_lock(data_mut);

			// Wait until the queue is not empty or stop_robot_thread becomes true
			motion_queue_trigger.wait(queue_lock, [&]() { return !motion_queue.empty() || stop; });

			// If the thread is requested to stop, exit the loop
			if (stop) {
				return;
			}

			motion = motion_queue.front();
			motion_queue.pop();

			// get_commands(); // Get commands after the queue is populated
		}

		// Execute the command and handle the result
		execute_command(*robot, motion.value());
		displayValues(robot);
	}
}

int main(int argc, char *argv[]) {
	signal(SIGINT, handle_sig);

	if (argc < 2) {
		fprintf(stderr, "Usage: %s <robot type> <calibration file>", argv[0]);
		exit(1);
	}

	Robot *robot;

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

	model = robot->get_model();
	last_target_pos = robot->get_current_cart_loc();

	// Enable RAW mode for terminal input
	enableRawMode();
	setNonBlockingInput();

	displayValues(robot);
	std::thread control_thread(robot_thread_func, robot);

	user_input();

	control_thread.join();

	// Disable RAW mode when done
	disableRawMode();

	delete robot;

	return 0;
}
