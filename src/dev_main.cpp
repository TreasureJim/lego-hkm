#include "robot.hpp"

Robot *robot;

void handle_sig(int sig) {
	delete robot;
	exit(0);
}

int main(int argc, char *argv[]) {
	robot = new FakeVisRobot(50);
	delete robot;
}
