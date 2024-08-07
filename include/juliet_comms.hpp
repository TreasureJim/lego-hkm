#pragma once

#include <cstdio>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <queue>


extern "C" {
#include "motion_types.h"
}

enum motiontype_e {
	MOVE_POS,
	MOVE_LIN,
	MOVE_ARC,
	MOVE_CIRC,
	MOVE_JOINT,
};

union motion_u {
	movepos pos;
	movelinear linear;
	movearc arc;
	movecircular circular;
	movejoint joint;
};

struct motion_command {
	motiontype_e type;
	motion_u motion;
};

extern std::mutex motion_queue_mutex;
extern std::queue<motion_command> motion_queue;
extern std::condition_variable motion_queue_trigger;

void push_to_queue(motion_command command);
void juliet_communication(int juliet_socket);
