#include "IMotion.hpp"
#include "juliet_comms.hpp"
#include "robot.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <cassert>
#include <cstdlib>
#include <mutex>
#include <optional>
#include <stdio.h>

std::atomic_bool stop_robot_thread = false;

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
	if (next_command.has_value()) {
		current_command = std::move(next_command.value());
		next_command = std::nullopt;
	} else {
		current_command = std::move(motion_queue.front());
		motion_queue.pop();
	}

	if (!motion_queue.empty()) {
		next_command = std::move(motion_queue.front());
		motion_queue.pop();
	}
}

bool execute_command(Robot &robot) {
	robot.execute_motion(*current_command);

	return true;
}

void robot_thread_func(Robot *robot) {
	while (!stop_robot_thread) {
		// check if queue is empty
		{
			std::unique_lock queue_lock(motion_queue_mutex);

			// Wait until the queue is not empty or stop_robot_thread becomes true
			motion_queue_trigger.wait(queue_lock, [&]() { return !motion_queue.empty() || stop_robot_thread; });

			// If the thread is requested to stop, exit the loop
			if (stop_robot_thread) {
				return;
			}

			get_commands(); // Get commands after the queue is populated
		}

		// Execute the command and handle the result
		if (!execute_command(*robot)) {
			send_command_status(*current_command, CMD_FAILED);
			return;
		}
		send_command_status(*current_command, CMD_COMPLETED);

		previous_command = std::move(current_command);
	}
}
