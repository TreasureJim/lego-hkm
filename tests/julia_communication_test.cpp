#include "juliet_comms.hpp"
#include <cstdio>
#include <thread>
#include <client.hpp>

using namespace std::chrono_literals;

void print_command(FILE *out, motion_command &command) {
	fprintf(out, "Type: %d, ID: ", command.type);
	for (int i = 0; i < 16; i++) {
		fprintf(out, "%02x", command.motion.pos.motion_id[i]);
	}
	fprintf(out, "\n");
}

motion_command command;

// assumes there are commands in queue and queue is locked for this thread
void get_commands() {
	command = motion_queue.front();
	motion_queue.pop();
}


void robot_thread_func() {
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
		print_command(stdout, command);
		send_command_status(command, CMD_COMPLETED);
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

	std::thread robot_thread(robot_thread_func);

	juliet_communication(jl_socket);

	return 0;
}
