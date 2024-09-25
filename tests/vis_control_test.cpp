#include <thread>
extern "C" {
#include "mark2_0.h"
}

#include "IMotion.hpp"
#include "robot.hpp"
#include "test_helpers.hpp"
#include "mark2_0.h"

int main (int argc, char *argv[]) {
	auto robot = FakeVisRobot(50);
	if (robot.error) {
		fprintf(stderr, "Error starting robot.\n");
		return 1;
	}

	while (true) {
		auto goal_pos = random_valid_cart_pos(&mark2_0);
		auto motion = MotionLinear(robot.get_current_cart_loc(), goal_pos, &mark2_0);
		printf("Executing motion.\n");

		if (robot.execute_motion(motion) < 0) {
			fprintf(stderr, "Invalid motion. Exiting!\n");
			exit(1);
		}

		// std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}

	return 0;
}
