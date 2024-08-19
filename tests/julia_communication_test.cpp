#include "juliet_comms.hpp"
#include <cstdio>
#include <thread>

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
