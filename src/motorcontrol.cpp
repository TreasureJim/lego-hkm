#include "rc/servo.h"
#include <motors.hpp>
#include <cmath> // For M_PI and the conversion
#include <cstdio>
#include "mark2_0_fixed.hpp"

int main(int argc, char *argv[]) {
    if (argc != 5) {
        // Error handling: Expecting 4 angles as input
        printf("Usage: %s angle1 angle2 angle3 angle4\n", argv[0]);
        return -1;
    }

    motor_setup();

    double joint_angles_deg[4]; // Array to store angles in degrees

    // Convert the command-line arguments to degrees
    for (int i = 0; i < 4; i++) {
        joint_angles_deg[i] = atof(argv[i + 1]); // Convert string to double
    }

    // Convert degrees to radians
    double joint_angles_rad[4];
    for (int i = 0; i < 4; i++) {
        joint_angles_rad[i] = joint_angles_deg[i] * (M_PI / 180.0);
    }

    // Send pulse with the converted radian values
    rc_servo_send_pulse_normalized(1, joint_angle_to_libservo_value(joint_angles_rad[0]));
    rc_servo_send_pulse_normalized(2, joint_angle_to_libservo_value(joint_angles_rad[1]));
    rc_servo_send_pulse_normalized(3, joint_angle_to_libservo_value(joint_angles_rad[2]));
    rc_servo_send_pulse_normalized(4, joint_angle_to_libservo_value(joint_angles_rad[3]));

    return 0;
}
