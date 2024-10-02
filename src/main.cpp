#include "lego_model.hpp"
#include "juliet_comms.hpp"
#include "motors.hpp"
#include "robot.hpp"
#include <arpa/inet.h>
#include <asm-generic/socket.h>
#include <cstdio>
#include <cstdlib>
#include <netinet/in.h>
#include <signal.h>
#include <sys/socket.h>
#include <thread>
#include "client.hpp"

Robot* robot;
std::thread* robot_thread;

void handle_sig(int sig) {
	cleanup_juliet_comms();

	stop_robot_thread = true;
	robot_thread->join();

	if (robot) delete robot;
	exit(0);
}

int main(int argc, char *argv[]) {
	signal(SIGINT, handle_sig);

	if (argc > 1 && strcmp(argv[1], "--calibrate") == 0) {
		motor_setup();
		motor_reset_angle(); // Call the calibration function
		motor_shutdown();
		return 0;           // Exit after calibration
	}

	if (argc < 3) {
		fprintf(stderr, "Usage: %s IPv4_addr port\n", argv[0]);
		exit(EXIT_FAILURE);
	}

	int jl_socket;
	if ((jl_socket = connect_to_server(argv[1], argv[2])) < 0)
		exit(1);

	robot = new LegoRobot(lego_model);
	if (robot->error) {
		fprintf(stderr, "[ERROR] Could not initialise robot.\n");
		exit(EXIT_FAILURE);
	}

	robot_thread = new std::thread(robot_thread_func, robot);

	juliet_communication(jl_socket, robot->get_current_cart_loc(), &robot->get_model());

	return 0;
}
