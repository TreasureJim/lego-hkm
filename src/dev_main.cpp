#include <Eigen/Dense>
#include <boost/chrono/duration.hpp>
#include <boost/thread/pthread/thread_data.hpp>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <math.h>
#include <netinet/in.h>
#include <random>
#include <sys/socket.h>
#include <unistd.h>

#include "kinematics.h"
#include "mark2_0_fixed.hpp"

extern "C" {
#include "chan/encoder.h"
#include "chan/fd_writer.h"
#include "motion_types.h"
}

#define RADIAN_TO_DEGREE 180/M_PI

std::default_random_engine re(std::random_device().entropy());

hkmpos random_hkm_pos(void) {
	hkmpos pos;

	std::uniform_real_distribution<double> unif1(mark2_0_fixed.joint_lims[0][0] + 0.0001,
	                                             mark2_0_fixed.joint_lims[0][1]);
	pos.j1 = unif1(re) * RADIAN_TO_DEGREE;

	std::uniform_real_distribution<double> unif2(mark2_0_fixed.joint_lims[1][0] + 0.0001,
	                                             mark2_0_fixed.joint_lims[1][1]);
	pos.j2 = unif2(re) * RADIAN_TO_DEGREE;

	std::uniform_real_distribution<double> unif3(mark2_0_fixed.joint_lims[2][0] + 0.0001,
	                                             mark2_0_fixed.joint_lims[2][1]);
	pos.j3 = unif3(re) * RADIAN_TO_DEGREE;

	std::uniform_real_distribution<double> unif4(mark2_0_fixed.joint_lims[3][0] + 0.0001,
	                                             mark2_0_fixed.joint_lims[3][1]);
	pos.j4 = unif4(re) * RADIAN_TO_DEGREE;

	return pos;
}

Eigen::Vector3d random_valid_cart_pos(void) {
	double joint_angles[4];
	for (int i = 0; i < 4; i++) {
		std::uniform_real_distribution<double> unif(mark2_0_fixed.joint_lims[i][0] + 0.0001,
		                                            mark2_0_fixed.joint_lims[i][1]);
		joint_angles[i] = unif(re);
	}

	double matrix[4][4];
	double angle = 0.0;
	fwd(&mark2_0_fixed, joint_angles, matrix, &angle);

	return Eigen::Vector3d(matrix[0][3], matrix[1][3], matrix[2][3]);
}

int main(int argc, char *argv[]) {
	int vis_socket = socket(AF_INET, SOCK_STREAM, 0);
	sockaddr_in vis_addr;
	vis_addr.sin_family = AF_INET;
	vis_addr.sin_port = htons(4445);
	vis_addr.sin_addr.s_addr = INADDR_ANY;

	if (connect(vis_socket, (struct sockaddr *)&vis_addr, sizeof(vis_addr))) {
		fprintf(stderr, "Couldn't connect to mock kernel on port 4445.\n");
		return 0;
	}

	printf("Connected to mock kernel on port 4445.\n");

	struct chan_writer *vis_writer = fd_writer_new(vis_socket);
	struct chan_encoder *vis_enc = chan_encoder_new(vis_writer);
	// if (chan_enc_register_hkmpos(vis_enc)) {
	if (chan_enc_register_hkmpos(vis_enc)) {
		fprintf(stderr, "Failed to register hkmpos to channel encoder.\n");
		return 1;
	}

	while (true) {
		struct hkmpos pos = random_hkm_pos();
		printf("Sending: %f, %f, %f, %f\n", pos.j1, pos.j2, pos.j3, pos.j4);
		encode_hkmpos(vis_enc, &pos);

		boost::this_thread::sleep(boost::posix_time::milliseconds(500));
	}

	fd_writer_free(vis_writer);
}
