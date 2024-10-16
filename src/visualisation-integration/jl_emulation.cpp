#include <arpa/inet.h>
#include <atomic>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <condition_variable>
#include <csignal>
#include <iostream>
#include <mutex>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

extern "C" {
#include "chan/decoder.h"
#include "chan/encoder.h"
#include "chan/fd_reader.h"
#include "chan/fd_writer.h"
#include "motion_types.h"
}

// GLOBALS
std::atomic<bool> stop = false;
std::thread* decoding_thread;

int server_sock = 0, client_sock = 0;

struct chan_writer * j_writer;
struct chan_reader * j_reader;

struct chan_encoder *encoder;
struct chan_decoder *decoder;

std::mutex mut;
std::condition_variable commandCompleted;


void generateUUID(uint8_t *motion_id) {
	boost::uuids::uuid uuid = boost::uuids::random_generator()();
	// Copy UUID into motion_id array (16 bytes)
	std::memcpy(motion_id, uuid.data, 16);
}

int startServer(uint16_t port) {
	server_sock = socket(AF_INET, SOCK_STREAM, 0);
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
		std::cerr << "Failed to bind to port. " << strerror(errno) << std::endl;
		close(server_sock);
		return -1;
	}

	if (listen(server_sock, 1) == -1) {
		std::cerr << "Failed to listen." << strerror(errno) << std::endl;
		close(server_sock);
		return -1;
	}

	std::cout << "Listening on port " << port << std::endl;

	client_sock = accept(server_sock, nullptr, nullptr);
	if (client_sock == -1) {
		std::cerr << "Failed to accept connection." << strerror(errno) << std::endl;
		close(server_sock);
		return -1;
	}

	std::cout << "Connected to robot" << std::endl;

	return 0;
}

void stopServer() {
	std::cout << "Closing server sockets" << std::endl;
	if (client_sock)
		close(client_sock);
	if (server_sock)
		close(server_sock);
	std::cout << "Closed sockets" << std::endl;
}

void handle_sig(int sig) {
	stop = true;
	decoding_thread->join();
	stopServer();
	exit(0);
}

void mlin(double x, double y, double z) {
	movelinear motion = {.target = {x, y, z, 1.0, 1.0, 1.0, 1.0}};
	// generateUUID(motion.motion_id);

	int result = encode_movelinear(encoder, &motion);
	if (result < 0) {
		std::cerr << "[ERROR] sending motiondid. Status: " << std::to_string(result) << std::endl;
		handle_sig(1);
	}
}

void motionid_cb(struct motionid *m, void *ctx) {
	std::lock_guard<std::mutex> lock(mut);
	commandCompleted.notify_one();
	std::cout << "Finished command" << std::endl;
}

void waitForCommandCompletion() {
	std::unique_lock lock(mut);
	commandCompleted.wait(lock);
}

void unknown_type_error_func(uint8_t *sig, uint32_t sig_len, void *ctx) {
	fprintf(stderr, "[ERROR] parsing type from juliet queue.\n");
}
char decoder_ctx[] = "decoder context";

void decoding_thread_f() {
	while (!stop)
		if (chan_decode(decoder) != 0) {
			std::cerr << "Error decoding" << std::endl;
			handle_sig(1);
	}
}

int main() {
	signal(SIGINT, handle_sig);
	uint16_t port = 9000;
	int result = startServer(port);
	if (result < 0) {
		return EXIT_FAILURE;
	}

	j_writer = fd_writer_new(client_sock);
	j_reader = fd_reader_new(client_sock);
	encoder = chan_encoder_new(j_writer);
	decoder = chan_decoder_new(j_reader, unknown_type_error_func, decoder_ctx);
	chan_enc_register_movelinear(encoder);
	chan_dec_register_motionid(decoder, motionid_cb, decoder_ctx);

	decoding_thread = new std::thread(decoding_thread_f);

	std::cout << "Starting commands now!" << std::endl;
	try {
		while (true) {
			mlin(0.945, 0.839, 0.67);
			mlin(0.52, -1.5, 0.2);

			waitForCommandCompletion();
			waitForCommandCompletion();
		}
	} catch (const std::exception &e) {
		std::cerr << "Error: " << e.what() << std::endl;
	}

	stopServer();
	return 0;
}
