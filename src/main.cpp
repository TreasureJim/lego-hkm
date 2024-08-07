#include "juliet_comms.hpp"
#include <arpa/inet.h>
#include <cstdio>
#include <cstdlib>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <thread>

int connect_to_server(char **argv) {
	sockaddr_in vis_addr;
	vis_addr.sin_family = AF_INET;
	vis_addr.sin_addr.s_addr = INADDR_ANY;

	int s = inet_pton(AF_INET, argv[1], (char *)&vis_addr.sin_addr.s_addr);
	if (s <= 0) {
		if (s == 0)
			fprintf(stderr, "[ERROR] Address not in valid format.\n");
		else
			perror("inet_pton");
		exit(EXIT_FAILURE);
	}
	try {
		int port = std::stoi(argv[2]);
		if (port < 0 || port > 65535) {
			fprintf(stderr, "[ERROR] Port argument must be a valid port number (<= 65535).\n");
			exit(EXIT_FAILURE);
		}
		vis_addr.sin_port = htons(port);
	} catch (...) {
		fprintf(stderr, "[ERROR] Port argument must be a valid port number.\n");
		exit(EXIT_FAILURE);
	}

	int vis_socket = socket(AF_INET, SOCK_STREAM, 0);
	if (connect(vis_socket, (struct sockaddr *)&vis_addr, sizeof(vis_addr))) {
		fprintf(stderr, "[ERROR] Couldn't connect to juliet on %s:%s.\n", argv[1], argv[2]);
		return -1;
	}

	return vis_socket;
}

int main(int argc, char *argv[]) {
	if (argc < 3) {
		fprintf(stderr, "Usage: %s IPv4_addr port\n", argv[0]);
		exit(EXIT_FAILURE);
	}

	int jl_socket;
	if ((jl_socket = connect_to_server(argv)) < 0)
		;

	// start robot thread
	// TODO
	// std::thread robot_thread()

	juliet_communication(jl_socket);

	return 0;
}
