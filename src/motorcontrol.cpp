#include "lego_model.hpp"
#include "rc/dsm.h"
#include "rc/servo.h"
#include "rc/time.h"
#include <cerrno>
#include <cmath> // For M_PI and the conversion
#include <cstdio>
#include <motors.hpp>
#include <thread>

using namespace std::chrono_literals;

int main(int argc, char *argv[]) {
	if (argc != 5) {
		// Error handling: Expecting 4 angles as input
		printf("Usage: %s angle1 angle2 angle3 angle4\n", argv[0]);
		return -1;
	}

	motor_setup();

	double joint_angles[4]; // Array to store angles in degrees

	// Convert the command-line arguments to degrees
	for (int i = 0; i < 4; i++) {
		char *e;
		joint_angles[i] = strtod(argv[i + 1], &e); // Convert string to double
		if (*e != '\0' || errno != 0) {
			fprintf(stderr, "[ERROR] angle arguments need to be valid numbers!");
			exit(1);
		}
	}

	printf("Moving motors to %f, %f, %f, %f.\n", joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3]);

	for (int i = 0; i < 30; i++) {
		// Send pulse with the converted radian values
		for (int m = 0; m < 4; m++) {
			if (rc_servo_send_pulse_normalized(m + 1, joint_angles[m]) == -1) {
				fprintf(stderr, "[ERROR] sending move command to servo: %d\n", m);
				return -1;
			}
		}
		rc_usleep(1000000 / 50);
	}

	rc_usleep(50000);
	// turn off power rail and cleanup
	rc_servo_power_rail_en(0);
	rc_servo_cleanup();
	rc_dsm_cleanup();

	return 0;
}
