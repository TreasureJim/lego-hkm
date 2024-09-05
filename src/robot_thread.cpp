#include "IMotion.hpp"
#include "juliet_comms.hpp"
#include "robot.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <cassert>
#include <cstdlib>
#include <mutex>
#include <optional>
#include <stdio.h>

std::optional<motion_command> previous_command;
motion_command current_command;
std::optional<motion_command> next_command;

void print_command(FILE *out, motion_command &command) {
	fprintf(out, "Type: %d, ID: ", command.type);
	for (int i = 0; i < 16; i++) {
		fprintf(out, "%02x", command.motion.pos.motion_id[i]);
	}
}

// assumes there are commands in queue and queue is locked for this thread
void get_commands() {
	current_command = motion_queue.front();
	motion_queue.pop();
	if (!motion_queue.empty())
		next_command = motion_queue.front();
}

bool execute_command(Robot &robot) {
	auto motion_s = &current_command.motion;

	IMotion *motion = nullptr;
	switch (current_command.type) {
	case MOVE_POS: {
		motion = new MotionPath();
		break;
	}
	case MOVE_LIN: {
		robot.move_linear(robtarget_to_vector(current_command.motion.linear.target));
		break;
	}
	case MOVE_ARC: {
		robot.move_arc(robtarget_to_vector(motion_s->arc.apos), robtarget_to_vector(motion_s->arc.target));
		break;
	}
	case MOVE_CIRC: {
		robot.move_circular(robtarget_to_vector(motion_s->circular.apos),
		                    robtarget_to_vector(motion_s->circular.target));
		break;
	}
		case MOVE_JOINT: {

		}
	default: {
		fprintf(stderr, "[ERROR] Unrecognised motion command. ");
		print_command(stderr, current_command);
		fprintf(stderr, ".\n");
		return false;
	}
	}

	robot.execute_motion(motion_s);

	return true;
}

void robot_thread_func() {
	Robot robot;
	if (robot.error) {
		fprintf(stderr, "[ERROR] Could not initialise robot.\n");
		exit(EXIT_FAILURE);
	}

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

		if (!execute_command(robot)) {
			send_command_status(current_command, CMD_FAILED);
			return;
		}
		send_command_status(current_command, CMD_COMPLETED);

		previous_command = current_command;
	}
}
