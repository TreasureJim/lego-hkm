#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <netinet/in.h>
#include <ostream>
#include <strings.h>
#include <thread>
#include <unistd.h>

extern "C" {
#include "chan/decoder.h"
#include "chan/encoder.h"
#include "chan/fd_reader.h"
#include "chan/fd_writer.h"
#include "motion_types.h"
}

using namespace std::chrono_literals;

void print_hex(const char *data, size_t length) {
	std::stringstream ss;
	ss << std::hex << std::setfill('0');
	for (size_t i = 0; i < length; ++i) {
		ss << std::setw(2) << static_cast<unsigned int>(static_cast<unsigned char>(data[i]));
	}
	std::cout << ss.str();
}

void print_motionid(motionid &motion) {
	std::cout << "Motion: ";
	print_hex((char*)&motion.id, sizeof(motion.id));
	std::cout << " status: " << (int)motion.status << std::endl;
}

char ctx[] = "someshi";

int main(int argc, char *argv[]) {
	int port = 9000;

	// setup a socket and connection tools
	sockaddr_in servAddr;
	bzero((char *)&servAddr, sizeof(servAddr));
	servAddr.sin_family = AF_INET;
	servAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	servAddr.sin_port = htons(port);

	// open stream oriented socket with internet address
	// also keep track of the socket descriptor
	int server_sock = socket(AF_INET, SOCK_STREAM, 0);
	if (server_sock < 0) {
		std::cerr << "Error establishing the server socket" << std::endl;
		exit(1);
	}

	int opt = 1;
	if (setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)) < 0) {
		perror("[ERROR] Setting SO_REUSEADDR");
		exit(1);
	}

	// bind the socket to its local address
	int bindStatus = bind(server_sock, (struct sockaddr *)&servAddr, sizeof(servAddr));
	if (bindStatus < 0) {
		std::cerr << "Error binding socket to local address. status: " << bindStatus << std::endl;
		perror("port binding failed");
		exit(0);
	}
	std::cout << "Waiting for a client to connect..." << std::endl;

	// listen for up to 5 requests at a time
	listen(server_sock, 1);

	// receive a request from client using accept
	// we need a new address to connect with the client
	sockaddr_in newSockAddr;
	socklen_t newSockAddrSize = sizeof(newSockAddr);
	// accept, create a new socket descriptor to
	// handle the new connection with client
	int robot_socket = accept(server_sock, (sockaddr *)&newSockAddr, &newSockAddrSize);
	if (robot_socket < 0) {
		std::cerr << "Error accepting request from client!" << std::endl;
		exit(1);
	}
	std::cout << "Connected with client!" << std::endl;

	struct chan_writer *j_writer = fd_writer_new(robot_socket);
	struct chan_reader *j_reader = fd_reader_new(robot_socket);
	if (j_reader == NULL) {
		fprintf(stderr, "fd_reader_new failed");
		exit(1);
	}

	struct chan_encoder *encoder = chan_encoder_new(j_writer);
	struct chan_decoder *decoder = chan_decoder_new(
		j_reader,
		[](unsigned char *, unsigned int, void *) {
			fprintf(stderr, "Couldn't decode message.\n");
			exit(1);
		},
		ctx);
	if (decoder == NULL) {
		fprintf(stderr, "chan_decoder_new failed");
		exit(1);
	}

	chan_enc_register_movelinear(encoder);
	chan_dec_register_motionid(
		decoder,
		[](struct motionid *m, void *ctx) {
			print_motionid(*m);
			std::this_thread::sleep_for(3000ms);
		},
		ctx);

	movelinear move;
	memset(&move, 0x12, sizeof(move.motion_id));

	printf("Sending command.\n");
	encode_movelinear(encoder, &move);

	if (chan_decode(decoder)) {
		printf("Decoding error.\n");
	}

	close(server_sock);
	printf("Ended decoding.\n");

	return 0;
}
