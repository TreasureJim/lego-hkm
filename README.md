# Lego HKM Robot

This project contains the server for controlling the robot (using juliet) and the motion kernel that controls the servos. 

## Program Arguments

\<robot type>

- either "lego" or "sim"

--calibrate

- puts the motors in their contracted extreme positions. eg. 0 degrees rotation
- maybe we will generate a calibration file in future. Holds custom extreme rotations

\<IPv4 address and port>

- for connecting to juliet server

## Building

This project uses Docker to compile. 

Run `./build_docker_image.sh` to build the required docker image.

Then run `./cross-compile.sh` to compile for the Beagle Board Bone Black.
