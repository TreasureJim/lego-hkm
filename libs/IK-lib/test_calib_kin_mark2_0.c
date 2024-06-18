#include <stdio.h>
#include <math.h>
#include <assert.h>
#ifndef _POSIX_C_SOURCE
#define M_PI		3.14159265358979323846	/* pi */
#define M_PI_2		1.57079632679489661923	/* pi/2 */
#define M_PI_4		0.78539816339744830962	/* pi/4 */
#endif
#include "kinematics.h"
#include "ckin/common.h"
#include "mark2_0.h"
#include "mark2_0_perturbed.h"

int main(void)
{
	size_t times;
	double qs[][4] = {{1.0, 0.5, 0.2, 2.0},
					{0.0, 0.0, 0.0, 0.0},
					{-0.5, -0.8, 0.2, 1.5}};
	for (times = 0; times < 3; times++)
	{
		double q[4];
		for (size_t i = 0; i < 4; i++) {
			q[i] = qs[times][i];
		}
		printf("Fwd for q: {%f, %f, %f, %f}\n", q[0], q[1], q[2], q[3]);

		double out_tcp[4][4];
		double out_elbow[4][4];
		double orient_angle;

		if (fwd(&mark2_0, q, out_tcp, &orient_angle, out_elbow, 0.0, 0) < 0)
		{
			printf("fwd() failed\n");
			return -1;
		}

		printf("pos: {%.8f, %.8f, %.8f}\n", out_tcp[0][3], out_tcp[1][3], out_tcp[2][3]);
		printf("elbow pos: {%.8f, %.8f, %.8f}\n", out_elbow[0][3], out_elbow[1][3], out_elbow[2][3]);
		printf("ori: %.16f\n", orient_angle);

		double inv_in[] = {out_tcp[0][3], out_tcp[1][3], out_tcp[2][3]};
		double q_out[4];

		if (inv(&mark2_0, inv_in, orient_angle, q_out, 0.0, 0) < 0)
		{
			printf("inv() failed\n");
			return -1;
		}

		printf("Inv:\n");
		printf("q_out: {%f, %f, %f, %f}\n", q_out[0], q_out[1], q_out[2], q_out[3]);
		printf("\n");
	}
	printf("--------Perturbed--------\n\n");
	for (times = 0; times < 3; times++)
	{
		double q[4];
		for (size_t i = 0; i < 4; i++) {
			q[i] = qs[times][i];
		}
		printf("Fwd for q: {%f, %f, %f, %f}\n", q[0], q[1], q[2], q[3]);

		double out_tcp[4][4];
		double out_elbow[4][4];
		double orient_angle;

		if (fwd(&mark2_0_perturbed, q, out_tcp, &orient_angle, out_elbow, 0.0, 0) < 0)
		{
			printf("fwd() failed\n");
			//return -1;
		}

		printf("pos: {%.8f, %.8f, %.8f}\n", out_tcp[0][3], out_tcp[1][3], out_tcp[2][3]);
		printf("elbow pos: {%.8f, %.8f, %.8f}\n", out_elbow[0][3], out_elbow[1][3], out_elbow[2][3]);
		printf("ori: %.16f\n", orient_angle);

		double inv_in[] = {out_tcp[0][3], out_tcp[1][3], out_tcp[2][3]};
		double q_out[4];

		if (inv(&mark2_0_perturbed, inv_in, orient_angle, q_out, 0.0, 0) < 0)
		{
			printf("inv() failed\n");
			//return -1;
		}
		printf("Inv:\n");
		printf("q_out: {%f, %f, %f, %f}\n", q_out[0], q_out[1], q_out[2], q_out[3]);
		printf("\n");
	}

	return 0;
}
