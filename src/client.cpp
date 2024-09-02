#include <arpa/inet.h>
#include <cstdio>
#include <cstdlib>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>

int connect_to_server(char* address_s, char* port_s) {
	sockaddr_in vis_addr;
	vis_addr.sin_family = AF_INET;
	vis_addr.sin_addr.s_addr = INADDR_ANY;

	int s = inet_pton(AF_INET, address_s, (char *)&vis_addr.sin_addr.s_addr);
	if (s <= 0) {
		if (s == 0)
			fprintf(stderr, "[ERROR] Address not in valid format.\n");
		else
			perror("inet_pton");
		exit(EXIT_FAILURE);
	}
	try {
		int port = std::stoi(port_s);
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
	if (vis_socket < 0) {
		perror("[ERROR] Creating socket");
		return -1;
	}

	if (connect(vis_socket, (struct sockaddr *)&vis_addr, sizeof(vis_addr))) {
		fprintf(stderr, "[ERROR] Couldn't connect to juliet on %s:%s.\n", address_s, port_s);
		return -1;
	}

	return vis_socket;
}
