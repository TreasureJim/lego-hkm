#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <queue>

#include "juliet_comms.hpp"

extern "C" {
#include "chan/decoder.h"
#include "chan/encoder.h"
#include "chan/fd_reader.h"
#include "chan/fd_writer.h"
#include "motion_types.h"
}

std::mutex motion_queue_mutex;
std::queue<motion_command> motion_queue;
std::condition_variable motion_queue_trigger;

void push_to_queue(motion_command command) {
	motion_queue.push(command);
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

void unknown_type_error_func(uint8_t *sig, uint32_t sig_len, void *ctx) {
	fprintf(stderr, "[ERROR] parsing type from juliet queue.\n");
}

struct chan_encoder* encoder = nullptr;
void juliet_communication(int juliet_socket) {
	auto j_writer = fd_writer_new(juliet_socket);
	auto j_reader = fd_reader_new(juliet_socket);
	auto j_enc = chan_encoder_new(j_writer);
	auto j_dec = chan_decoder_new(j_reader, unknown_type_error_func, NULL);

	// register encoder types
	chan_enc_register_motionid(j_enc);

	// register decoder types
	chan_dec_register_movepos(j_dec, movepos_callbck_func, NULL);
	chan_dec_register_movelinear(j_dec, movelinear_callbck_func, NULL);
	chan_dec_register_movearc(j_dec, movearc_callbck_func, NULL);
	chan_dec_register_movecircular(j_dec, movecircular_callbck_func, NULL);
	chan_dec_register_movejoint(j_dec, movejoint_callbck_func, NULL);

	// decode
	while (chan_decode(j_dec))
		;

	printf("Reached EOF in socket.\n");
}

void send_command_status(const motion_command& command, command_status_e status) {
	motionid motion_status = {
		.status = (int8_t)status
	};
	memcpy(motion_status.id, command.motion.linear.motion_id, 16);

	chan_encode_motionid(encoder, &motion_status);
} 
