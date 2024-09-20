#include "IMotion.hpp"
#include "juliet_comms.hpp"
#include "robot.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <cassert>
#include <cstdlib>
#include <mutex>
#include <optional>
#include <stdio.h>

std::optional<std::unique_ptr<IMotion>> previous_command;
std::unique_ptr<IMotion> current_command;
std::optional<std::unique_ptr<IMotion>> next_command;

void print_command(FILE *out, motion_command &command) {
	fprintf(out, "Type: %d, ID: ", command.type);
	for (int i = 0; i < 16; i++) {
		fprintf(out, "%02x", command.motion.pos.motion_id[i]);
	}
}

// assumes there are commands in queue and queue is locked for this thread
void get_commands() {
	current_command = std::move(motion_queue.front());
	motion_queue.pop();
	if (!motion_queue.empty())
		next_command = std::move(motion_queue.front());
}

bool execute_command(Robot &robot) {
	robot.execute_motion(*current_command);

	return true;
}

void robot_thread_func(Robot* robot) {
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

		if (!execute_command(*robot)) {
			send_command_status(*current_command, CMD_FAILED);
			return;
		}
		send_command_status(*current_command, CMD_COMPLETED);

		previous_command = std::move(current_command);
	}
}
