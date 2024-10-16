#include <arpa/inet.h>
#include <atomic>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <condition_variable>
#include <csignal>
#include <iostream>
#include <mutex>
#include <thread>
#include <unistd.h>

extern "C" {
#include "chan/decoder.h"
#include "chan/encoder.h"
#include "chan/fd_reader.h"
#include "chan/fd_writer.h"
#include "motion_types.h"
}

std::atomic<bool> stop = false;

void generateUUID(uint8_t *motion_id) {
	boost::uuids::uuid uuid = boost::uuids::random_generator()();
	// Copy UUID into motion_id array (16 bytes)
	std::memcpy(motion_id, uuid.data, 16);
}

struct chan_encoder *encoder;
void mlin(double x, double y, double z) {
	robtarget target = {x, y, z, 1.0, 1.0, 1.0, 1.0};

	movelinear motion = {.target = target};
	generateUUID(motion.motion_id);

	int result = encode_movelinear(encoder, &motion);
	if (result < 0) {
		fprintf(stderr, "[ERROR] sending motiondid. Status: %d.\n", result);
	}
}

std::mutex mut;
std::condition_variable commandCompleted;

int server_sock, client_sock;

int startServer(uint16_t port) {
	int server_sock = socket(AF_INET, SOCK_STREAM, 0);
	if (server_sock == -1) {
		std::cerr << "Failed to create socket." << std::endl;
		return -1;
	}

	int opt = 1;
	if (setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) == -1) {
		std::cerr << "Failed to set socket options." << std::endl;
		close(server_sock);
		return -1;
	}

	sockaddr_in serverAddr;
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(port);
	serverAddr.sin_addr.s_addr = INADDR_ANY;

	if (bind(server_sock, (sockaddr *)&serverAddr, sizeof(serverAddr)) == -1) {
		std::cerr << "Failed to bind to port." << std::endl;
		close(server_sock);
		return -1;
	}

	if (listen(server_sock, 1) == -1) {
		std::cerr << "Failed to listen." << std::endl;
		close(server_sock);
		return -1;
	}

	std::cout << "Listening on port " << port << std::endl;

	int client_sock = accept(server_sock, nullptr, nullptr);
	if (client_sock == -1) {
		std::cerr << "Failed to accept connection." << std::endl;
		close(server_sock);
		return -1;
	}

	std::cout << "Connected to robot" << std::endl;

	return 0;
}

void stopServer(int serverSocket, int clientSocket) {
	std::cout << "Closing server sockets" << std::endl;
	close(clientSocket);
	close(serverSocket);
}

void motionid_cb(struct motionid *m, void *ctx) { commandCompleted.notify_one(); }

void waitForCommandCompletion() {
	std::unique_lock lock(mut);
	commandCompleted.wait(lock);
}

void unknown_type_error_func(uint8_t *sig, uint32_t sig_len, void *ctx) {
	fprintf(stderr, "[ERROR] parsing type from juliet queue.\n");
}
char decoder_ctx[] = "decoder context";

struct chan_decoder *decoder;
void decoding_thread_f() {
	if (stop)
		return;
	chan_decode(decoder);
}

void handle_sig(int sig) {
	stop = true;
	stopServer(server_sock, client_sock);
	exit(0);
}

int main() {
	signal(SIGINT, handle_sig);
	uint16_t port = 9000;
	int result = startServer(port);
	if (result < 0) {
		return EXIT_FAILURE;
	}

	auto j_writer = fd_writer_new(client_sock);
	auto j_reader = fd_reader_new(client_sock);
	encoder = chan_encoder_new(j_writer);
	decoder = chan_decoder_new(j_reader, unknown_type_error_func, decoder_ctx);
	chan_enc_register_movelinear(encoder);
	chan_dec_register_motionid(decoder, motionid_cb, decoder_ctx);

	std::thread decoding_thread(decoding_thread_f);

	std::cout << "Starting commands now!" << std::endl;
	try {
		while (true) {
			mlin(0.210862, 0.0692802, 0.0278155);
			mlin(0.163194, 0.0995267, 0.112009);
			mlin(0.132179, 0.233252, 0.112009);
			mlin(0.13639, 0.222761, 0.0348802);

			waitForCommandCompletion();
			waitForCommandCompletion();
			waitForCommandCompletion();
			waitForCommandCompletion();
		}
	} catch (const std::exception &e) {
		std::cerr << "Error: " << e.what() << std::endl;
	}

	stopServer(result, result);
	return 0;
}
