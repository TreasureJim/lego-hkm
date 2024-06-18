#include <stdio.h>
#include <math.h>
#include <assert.h>
#ifndef _POSIX_C_SOURCE
#define M_PI		3.14159265358979323846	/* pi */
#define M_PI_2		1.57079632679489661923	/* pi/2 */
#define M_PI_4		0.78539816339744830962	/* pi/4 */
#endif
#include "kinematics.h"
#include "mark1_5.h"

int main(void)
{
	size_t times;
	for (times = 0; times < 1; times++) {
		double q[] = {0.1, 1.5 - M_PI_2, -0.3, 2.0};
		printf("Drive to cartesian for q: {%f, %f, %f, %f}\n", q[0], q[1], q[2], q[3]);

		double out_tcp[4][4];
		double orient_angle;

		if (drive_to_cart(&mark1_5, q, out_tcp, &orient_angle) < 0) {
			printf("drive_to_cart() failed\n");
			return -1;
		}

		printf("pos: {%.8f, %.8f, %.8f}\n", out_tcp[0][3], out_tcp[1][3], out_tcp[2][3]);
		printf("ori: %.16f\n", orient_angle);

		double inv_in[] = {out_tcp[0][3], out_tcp[1][3], out_tcp[2][3]};
		double q_out[4];

		if (cart_to_drive(&mark1_5, inv_in, orient_angle, q_out) < 0) {
			printf("cart_to_drive() failed\n");
			return -1;
		}

		printf("Cartesian to drive:\n");
		printf("q_out[0]: {%f, %f, %f, %f}\n", q_out[0], q_out[1], q_out[2], q_out[3]);
		for (int i = 0; i < 4; i++) {
			assert(fabs(q[i] - q_out[i]) < 1e-10);
		}
	}

	return 0;
}
