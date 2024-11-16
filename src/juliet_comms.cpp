#include "juliet_comms.hpp"
#include "IMotion.hpp"
#include "motors.hpp"
#include "eigen_kinematics.hpp"
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <stdio.h>
#include <sys/socket.h>
#include <unistd.h>

extern "C" {
#include "chan/decoder.h"
#include "chan/encoder.h"
#include "chan/fd_reader.h"
#include "chan/fd_writer.h"
#include "motion_types.h"
}

agile_pkm_model* model;

std::mutex motion_queue_mutex;
std::queue<std::unique_ptr<IMotion>> motion_queue;
std::condition_variable motion_queue_trigger;
Eigen::Vector3d last_target_pos;

Eigen::Vector3d last_completed_pos;

struct chan_encoder *encoder = nullptr;
struct chan_decoder *decoder = nullptr;
char decoder_ctx[] = "decoder context";

int socket_fd;


void push_to_queue(motion_command command) {
	std::cout << "RECIEVED MOTION" << std::endl;

	IMotion* motion = IMotion::motion_com_to_IMotion(last_target_pos, command, model);
	last_target_pos = motion->get_target_pos();
	motion_queue.push(std::unique_ptr<IMotion>(motion));
	motion_queue_trigger.notify_one();
}

#define GEN_MOTION_TYPE_CALLBCK(MOTION_TYPE, U_NAME, M_E)        \
	void MOTION_TYPE##_callbck_func(MOTION_TYPE *m, void *ctx) { \
		motion_u motion;                                         \
		motion.U_NAME = *m;                                      \
		motion_command command = {                               \
			.type = M_E,                                         \
			.motion = motion,                                    \
		};                                                       \
		push_to_queue(command);                                  \
	}

GEN_MOTION_TYPE_CALLBCK(movepos, pos, MOVE_POS);
GEN_MOTION_TYPE_CALLBCK(movelinear, linear, MOVE_LIN);
GEN_MOTION_TYPE_CALLBCK(movearc, arc, MOVE_ARC);
GEN_MOTION_TYPE_CALLBCK(movecircular, circular, MOVE_CIRC);
GEN_MOTION_TYPE_CALLBCK(movejoint, joint, MOVE_JOINT);
GEN_MOTION_TYPE_CALLBCK(movejog, jog, MOVE_JOG);

void send_robotstatus() {
	auto pos = last_completed_pos;
	auto joints = inverse(pos, model).value_or(std::array<double, 4>());

	robotstatus status {
		.x = pos.x(),
		.y = pos.y(),
		.z = pos.z(),
		.j1 = joints[0],
		.j2 = joints[1],
		.j3 = joints[2],
		.j4 = joints[3],
	};

	int result = encode_robotstatus(encoder, &status);
	if (result < 0) {
		fprintf(stderr, "[ERROR] sending robotstatus. Status: %d.\n", result);
	}
}

void robotrequeststatus_callbck_func(robotrequeststatus* m, void* ctx) {
	send_robotstatus();
}

void unknown_type_error_func(uint8_t *sig, uint32_t sig_len, void *ctx) {
	fprintf(stderr, "[ERROR] parsing type from juliet queue.\n");
}

void cleanup_juliet_comms() {
	fd_writer_free(encoder->writer);
	chan_encoder_free(encoder);

	fd_reader_free(decoder->reader);
	chan_decoder_free(decoder);

	close(socket_fd);
}

void juliet_communication(int juliet_socket, Eigen::Vector3d initial_location, agile_pkm_model* c_model) {
	model = c_model;
	last_completed_pos = last_target_pos = initial_location;

	socket_fd = juliet_socket;

	auto j_writer = fd_writer_new(juliet_socket);
	auto j_reader = fd_reader_new(juliet_socket);
	encoder = chan_encoder_new(j_writer);
	decoder = chan_decoder_new(j_reader, unknown_type_error_func, decoder_ctx);

	// register encoder types
	chan_enc_register_motionid(encoder);
	chan_enc_register_robotstatus(encoder);

	// register decoder types
	chan_dec_register_movepos(decoder, movepos_callbck_func, decoder_ctx);
	chan_dec_register_movelinear(decoder, movelinear_callbck_func, decoder_ctx);
	chan_dec_register_movearc(decoder, movearc_callbck_func, decoder_ctx);
	chan_dec_register_movecircular(decoder, movecircular_callbck_func, decoder_ctx);
	chan_dec_register_movejoint(decoder, movejoint_callbck_func, decoder_ctx);

	chan_dec_register_movejog(decoder, movejog_callbck_func, decoder_ctx);
	chan_dec_register_robotrequeststatus(decoder, robotrequeststatus_callbck_func, decoder_ctx);

	// decode
	int status;
	/* while ((status = chan_decode(decoder)) == 0)
		; */
	while(true) chan_decode(decoder);

	cleanup_juliet_comms();
	printf("Received non-zero status from chan: %d.\n", status);
}

void send_command_status(const IMotion &command, command_status_e status) {
	motionid motion_status = {.status = (int8_t)status};
	memcpy(motion_status.id, command.uuid, 16);

	int result = encode_motionid(encoder, &motion_status);
	if (result < 0) {
		fprintf(stderr, "[ERROR] sending motiondid. Status: %d.\n", result);
	}

	std::cout << "MOTION COMPLETED, ID: " << std::to_string(status) << std::endl;

	if (status == CMD_COMPLETED) {
		last_completed_pos = command.get_target_pos();
	}

	send_robotstatus();
}
