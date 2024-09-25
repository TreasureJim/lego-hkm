#pragma once

#include <cstdio>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <queue>
#include "IMotion.hpp"


extern std::mutex motion_queue_mutex;
extern std::queue<std::unique_ptr<IMotion>> motion_queue;
extern std::condition_variable motion_queue_trigger;

void push_to_queue(motion_command command);
void juliet_communication(int juliet_socket, Eigen::Vector3d initial_location, agile_pkm_model* c_model);

enum command_status_e {
	CMD_FAILED = -1,
	CMD_INPROGRESS,
	CMD_COMPLETED
};
void send_command_status(const IMotion &command, command_status_e status);
