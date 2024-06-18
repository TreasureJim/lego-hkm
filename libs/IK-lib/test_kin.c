#include "kinematics.h"
#include "mark2_0.h"
#include <stdio.h>

#define RAD_TO_DEG 57.295779513

int main(){
    // Cartesian position {x, y, z} in meters.
    double pos[3] = {1.0, 1.0, 0.2};

    // Cartesian rotation of TCP around Z-axis.
    double q4 = 0.0; 

    // Offset of tool 
    double tool[3] = {0.0, 0.0, 0.0};

    // Output drive positions
    double q[4];

    // Dp the inverse kinematic transform
    inv_posonly(&mark2_0, pos, q4, tool, q);

    // Print the values of drive position 1, 2, 3 & 4 (radians)
    printf("Drive positions: %f, %f, %f, %f", q[0] * RAD_TO_DEG, q[1] * RAD_TO_DEG, q[2] * RAD_TO_DEG, q[3] * RAD_TO_DEG);

    return 0;
}