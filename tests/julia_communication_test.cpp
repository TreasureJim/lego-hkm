#include "IMotion.hpp"
#include "juliet_comms.hpp"
#include "options.hpp"
#include <cstdio>
#include <thread>
#include <client.hpp>

extern "C" {
	#include "mark2_0.h"
}

using namespace std::chrono_literals;

void print_command(FILE *out, IMotion &command) {
	fprintf(out, "ID: ");
	for (int i = 0; i < 16; i++) {
		fprintf(out, "%02x", command.uuid[i]);
	}
	fprintf(out, "\n");
}

std::unique_ptr<IMotion> command;

// assumes there are commands in queue and queue is locked for this thread
void get_commands() {
	command = std::move(motion_queue.front());
	motion_queue.pop();
}


void robot_thread_func_test() {
	while (true) {
		// check if queue is empty
		{
			std::unique_lock queue_lock(motion_queue_mutex);

			if (motion_queue.empty()) {
				queue_lock.unlock();
				// wait for notification
				motion_queue_trigger.wait(queue_lock);
				get_commands();
			} else {
				get_commands();
			}
		}

		std::this_thread::sleep_for(1000ms);
		print_command(stdout, *command);
		send_command_status(*command, CMD_COMPLETED);
	}
}

int main(int argc, char *argv[]) {
	if (argc < 3) {
		fprintf(stderr, "Usage: %s IPv4_addr port\n", argv[0]);
		exit(EXIT_FAILURE);
	}

	int jl_socket;
	if ((jl_socket = connect_to_server(argv[1], argv[2])) < 0)
		exit(1);

	std::thread robot_thread(robot_thread_func_test);

	juliet_communication(jl_socket, calibration_location, &mark2_0);

	return 0;
}
