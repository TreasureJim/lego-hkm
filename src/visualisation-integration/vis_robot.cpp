#include <thread>
#include <unistd.h>

#include "robot.hpp"
#include <netinet/in.h>

extern "C" {
#include "mark2_0.h"

#include "chan/encoder.h"
#include "chan/fd_writer.h"
#include "motion_types.h"
}

using namespace std::chrono_literals;

FakeVisRobot::FakeVisRobot(int fake_delay, int port, Eigen::Vector3d starting_pos)
	: Robot(mark2_0), port(port), fake_delay(fake_delay), current_loc(starting_pos) {
	assert(port >= 0 && port < 65535);
	if (this->robot_setup() < 0) {
		this->error = true;
	}
}

FakeVisRobot::FakeVisRobot() : Robot(mark2_0) {
	assert(port >= 0 && port < 65535);
	if (this->robot_setup() < 0) {
		this->error = true;
	}
}

FakeVisRobot::~FakeVisRobot() { this->robot_shutdown(); }

int FakeVisRobot::robot_setup() {
	sockaddr_in servAddr;
	bzero((char *)&servAddr, sizeof(servAddr));
	servAddr.sin_family = AF_INET;
	servAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	servAddr.sin_port = htons(this->port);

	// open stream oriented socket with internet address
	// also keep track of the socket descriptor
	int sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock < 0) {
		fprintf(stderr, "Error establishing the server socket\n");
		exit(0);
	}

	int reuse = 1;
	int result = setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (void *)&reuse, sizeof(reuse));
	if (result < 0) {
		perror("ERROR SO_REUSEADDR:");
	}

	// bind the socket to its local address
	int bindStatus = bind(sock, (struct sockaddr *)&servAddr, sizeof(servAddr));
	if (bindStatus < 0) {
		fprintf(stderr, "Error binding socket to local address\n");
		exit(0);
	}

	printf("Listening on port: %d\n", this->port);
	printf("Waiting for a visualisation client to connect...\n");

	// listen for up to 5 requests at a time
	listen(sock, 1);

	// receive a request from client using accept
	// we need a new address to connect with the client
	sockaddr_in newSockAddr;
	socklen_t newSockAddrSize = sizeof(newSockAddr);

	// accept, create a new socket descriptor to
	// handle the new connection with client
	int vis_conn = accept(sock, (sockaddr *)&newSockAddr, &newSockAddrSize);
	if (vis_conn < 0) {
		fprintf(stderr, "Error accepting request from client!\n");
		exit(1);
	}

	printf("Connected to visualisation!\n");

	auto writer = fd_writer_new(vis_conn);
	this->encoder = chan_encoder_new(writer);

	chan_enc_register_hkmpos(encoder);

	this->s_socket = sock;
	this->vis_conn = vis_conn;

	return 0;
}

void FakeVisRobot::robot_shutdown() {
	chan_encoder_free(this->encoder);

	close(this->vis_conn);
	close(this->s_socket);
}

int FakeVisRobot::go_to(Eigen::Vector3d pos) {
	// Send Command
	hkmpos hkm_pos;
	hkm_pos.j1 = pos.x() * 100;
	hkm_pos.j2 = pos.y() * 100;
	hkm_pos.j3 = pos.z() * 100;
	hkm_pos.j4 = 0;

	encode_hkmpos(this->encoder, &hkm_pos);

	this->current_loc = pos;

	// Delay
	std::this_thread::sleep_for(std::chrono::milliseconds(this->fake_delay));
	return 0;
}

Eigen::Vector3d FakeVisRobot::get_current_cart_loc() { return this->current_loc; }
