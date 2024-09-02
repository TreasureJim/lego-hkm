#include "juliet_comms.hpp"
#include "motors.hpp"
#include "robot.hpp"
#include <arpa/inet.h>
#include <asm-generic/socket.h>
#include <cstdio>
#include <cstdlib>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <thread>
#include "client.hpp"

int main(int argc, char *argv[]) {
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

	std::thread robot_thread(robot_thread_func);

	juliet_communication(jl_socket);

	return 0;
}
