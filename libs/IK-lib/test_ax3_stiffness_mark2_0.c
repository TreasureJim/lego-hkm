#include <stdio.h>
#include <math.h>
#include <assert.h>
#ifndef _POSIX_C_SOURCE
#define M_PI		3.14159265358979323846	/* pi */
#define M_PI_2		1.57079632679489661923	/* pi/2 */
#define M_PI_4		0.78539816339744830962	/* pi/4 */
#endif
#include "kinematics.h"
#include "dyn.h"
#include "ckin/common.h"
#include "mark2_0.h"
#include "mark2_0_lpoe.h"

int main(void)
{
	size_t times;
	double qs[][4] = {{1.1, 0.1, -0.3, 2.0},
					{0.0, 0.0, 0.0, 0.0},
					{-0.5, -0.8, 0.2, 1.5},
					{1.0, 0.5, 0.2, 2.0}};
	struct dyn_params_link tool = {
                .m = 4,
                .r = {0, 0, 0},
                .ival = {{0, 0, 0},
                         {0, 0, 0},
                         {0, 0, 0}},
    };

	for (times = 0; times < 4; times++)
	{
		double q[4];
		for (size_t i = 0; i < 4; i++) {
			q[i] = qs[times][i];
		}
		printf("Fwd for q: {%f, %f, %f, %f}\n", q[0], q[1], q[2], q[3]);

		double out_tcp[4][4];
		double out_elbow[4][4];
		double orient_angle;

		if (fwd_full(&mark2_0, &mark2_0_lpoe, mark2_0_dynpars, &tool, q, out_tcp, &orient_angle, out_elbow, 0.0, 0) < 0)
		{
			printf("fwd() failed\n");
			return -1;
		}

		printf("pos: {%.8f, %.8f, %.8f}\n", out_tcp[0][3], out_tcp[1][3], out_tcp[2][3]);
		printf("elbow pos: {%.8f, %.8f, %.8f}\n", out_elbow[0][3], out_elbow[1][3], out_elbow[2][3]);
		printf("ori: %.16f\n", orient_angle);

		double inv_in[] = {out_tcp[0][3], out_tcp[1][3], out_tcp[2][3]};
		double q_out[4];

		if (inv_full(&mark2_0, &mark2_0_lpoe, mark2_0_dynpars, &tool, inv_in, orient_angle, q_out, 0.0, 0) < 0)
		{
			printf("inv() failed\n");
			return -1;
		}

		printf("Inv:\n");
		printf("q_out: {%f, %f, %f, %f}\n", q_out[0], q_out[1], q_out[2], q_out[3]);
		printf("\n");


        double v[4] = {0.5, 0.5, 0.2, 23.0};
        double acc[4] = {-1.0, 1.0, -1.0, 1.0};
		double qd[4];
		double qdd[4];
		if (inv_with_vel_acc_full(&mark2_0, &mark2_0_lpoe, mark2_0_dynpars, &tool, inv_in, orient_angle, v, acc, q_out, qd, qdd) < 0)
		{
			printf("inv_vel_acc() failed\n");
			return -1;
		}

		printf("Inv:\n");
		printf("q_out: {%f, %f, %f, %f}\n", q_out[0], q_out[1], q_out[2], q_out[3]);
		printf("qd: {%f, %f, %f, %f}\n", qd[0], qd[1], qd[2], qd[3]);
		printf("qdd: {%f, %f, %f, %f}\n", qdd[0], qdd[1], qdd[2], qdd[3]);
		printf("\n");
	}
	// printf("--------Perturbed--------\n\n");
	// for (times = 0; times < 3; times++)
	// {
	// 	double q[4];
	// 	for (size_t i = 0; i < 4; i++) {
	// 		q[i] = qs[times][i];
	// 	}
	// 	printf("Fwd for q: {%f, %f, %f, %f}\n", q[0], q[1], q[2], q[3]);
	//
	// 	double out_tcp[4][4];
	// 	double orient_angle;
	//
	// 	if (fwd(&mark2_0_perturbed, q, out_tcp, &orient_angle, 0.0, 0) < 0)
	// 	{
	// 		printf("fwd() failed\n");
	// 		//return -1;
	// 	}
	//
	// 	printf("pos: {%.8f, %.8f, %.8f}\n", out_tcp[0][3], out_tcp[1][3], out_tcp[2][3]);
	// 	printf("ori: %.16f\n", orient_angle);
	//
	// 	double inv_in[] = {out_tcp[0][3], out_tcp[1][3], out_tcp[2][3]};
	// 	double q_out[4];
	//
	// 	if (inv(&mark2_0_perturbed, inv_in, orient_angle, q_out, 0.0, 0) < 0)
	// 	{
	// 		printf("inv() failed\n");
	// 		//return -1;
	// 	}
	// 	printf("Inv:\n");
	// 	printf("q_out: {%f, %f, %f, %f}\n", q_out[0], q_out[1], q_out[2], q_out[3]);
	// 	printf("\n");
	// }

	return 0;
}
