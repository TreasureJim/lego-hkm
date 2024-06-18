/*
 * Copyright (C) 2020 Cognibotics
 *
 * Use or copying of this file or any part of it, without explicit
 * permission from Cognibotics is not allowed in any way.
 */
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <time.h>

#include "c_lpoe/lpoe.h"
#include "kinematics.h"
#include "kin_model.h"
#include "mark1_5.h"
#include "mark2_0.h"

#define NBR_ITE 10000

static void bench_vel_acc_calc(void)
{
	double q[4] = {0.1, 1.5 - M_PI_2, -0.3, 45};
	double orient_angle;
	double out_tcp[4][4];
	double v[4] = {0.5, 0.5, 0.2, 23.0};
	double acc[4] = {-1.0, 1.0, -1.0, 1.0};
	double q_inv[4];
	double qd_inv[4];
	double qdd_inv[4];
	int ret;

	ret = drive_to_cart(&kin_model, q, out_tcp, &orient_angle);

	double pos[3] = {out_tcp[0][3], out_tcp[1][3], out_tcp[2][3]};

	printf("\nBench of calculation of qd and qdd:\n");

	double start, stop, t1;

	start = clock();
	for (int i = 0; i < NBR_ITE; ++i) {
		ret = cart_to_drive_with_vel_acc(&kin_model, pos, orient_angle, v, acc, q_inv, qd_inv, qdd_inv);
	}
	printf("ret: %d\n", ret);
	stop = clock();
	t1 = (double)(stop - start) / CLOCKS_PER_SEC / NBR_ITE * 1e6;
	printf("%d iterations took on average %f microsec per call\n",
	       NBR_ITE, t1);
}

static void bench_vel_acc_calc_mark1_5(void)
{
	double q[4] = {0.1, 1.5 - M_PI_2, -0.3, 2.5};
	double orient_angle;
	double out_tcp[4][4];
	res_q_extra_t res_q_extra;
	ax4_fwd_ret_t ax4_fwd_ret;
	double v[4] = {0.5, 0.5, 0.2, 2.3};
	double acc[4] = {-1.0, 1.0, -1.0, 1.0};
	double q_inv[4];
	double qd_inv[4];
	double qdd_inv[4];
	int ret;
	double start, stop, t1;

	printf("\nBench of drive_to_cart for Mark1_5:\n");
	start = clock();
	for (int i = 0; i < NBR_ITE; ++i) {
		ret = drive_to_cart(&mark1_5, q, out_tcp, &orient_angle);
	}
	printf("ret: %d\n", ret);
	stop = clock();
	t1 = (double)(stop - start) / CLOCKS_PER_SEC / NBR_ITE * 1e6;
	printf("%d iterations took on average %f microsec per call\n",
	       NBR_ITE, t1);

	double pos[3] = {out_tcp[0][3], out_tcp[1][3], out_tcp[2][3]};

	printf("\nBench of calculation of qd and qdd for Mark1_5:\n");
	start = clock();
	for (int i = 0; i < NBR_ITE; ++i) {
		ret = cart_to_drive_with_vel_acc(&mark1_5, pos, orient_angle, v, acc, q_inv, qd_inv, qdd_inv);
	}
	printf("ret: %d\n", ret);
	stop = clock();
	t1 = (double)(stop - start) / CLOCKS_PER_SEC / NBR_ITE * 1e6;
	printf("%d iterations took on average %f microsec per call\n",
	       NBR_ITE, t1);
}

static void bench_vel_acc_calc_mark2_0(void)
{
	double q[4] = {0.1, 1.5 - M_PI_2, -0.3, 2.5};
	double orient_angle;
	double out_tcp[4][4];
	res_q_extra_t res_q_extra;
	ax4_fwd_ret_t ax4_fwd_ret;
	double v[4] = {0.5, 0.5, 0.2, 2.3};
	double acc[4] = {-1.0, 1.0, -1.0, 1.0};
	double q_inv[4];
	double qd_inv[4];
	double qdd_inv[4];
	int ret;
	double start, stop, t1;

	printf("\nBench of drive_to_cart for Mark2_0:\n");
	start = clock();
	for (int i = 0; i < NBR_ITE; ++i) {
		ret = drive_to_cart(&mark2_0, q, out_tcp, &orient_angle);
	}
	printf("ret: %d\n", ret);
	stop = clock();
	t1 = (double)(stop - start) / CLOCKS_PER_SEC / NBR_ITE * 1e6;
	printf("%d iterations took on average %f microsec per call\n",
	       NBR_ITE, t1);

	double th[4];
	printf("\nBench of drive_to_joint_full for Mark2_0:\n");
	start = clock();
	for (int i = 0; i < NBR_ITE; ++i) {
		ret = drive_to_joint_full(&mark2_0, q, th, &res_q_extra, &ax4_fwd_ret);
	}
	printf("ret: %d\n", ret);
	stop = clock();
	t1 = (double)(stop - start) / CLOCKS_PER_SEC / NBR_ITE * 1e6;
	printf("%d iterations took on average %f microsec per call\n",
	       NBR_ITE, t1);

	double pos[3] = {out_tcp[0][3], out_tcp[1][3], out_tcp[2][3]};

	printf("\nBench of calculation of qd and qdd for Mark2_0:\n");
	start = clock();
	for (int i = 0; i < NBR_ITE; ++i) {
		ret = cart_to_drive_with_vel_acc(&mark2_0, pos, orient_angle, v, acc, q_inv, qd_inv, qdd_inv);
	}
	printf("ret: %d\n", ret);
	stop = clock();
	t1 = (double)(stop - start) / CLOCKS_PER_SEC / NBR_ITE * 1e6;
	printf("%d iterations took on average %f microsec per call\n",
	       NBR_ITE, t1);
}

/*
static void bench_calib_kin_mark2_0(void)
{
	double q[4] = {0.1, 1.5 - M_PI_2, -0.3, 2.5};
	double q[4] = {0.1, 1.5, -0.3, 2.5};
	double orient_angle;
	double out_tcp[4][4];
	double v[4] = {0.5, 0.5, 0.2, 2.3};
	double acc[4] = {-1.0, 1.0, -1.0, 1.0};
	double q_inv[4];
	double qd_inv[4];
	double qdd_inv[4];
	res_q_extra_t res_q_extra;
	ax4_fwd_ret_t ax4_fwd_ret;
	int ret;
	double start, stop, t1;

	struct dyn_params_link tool = {
		.m = 4,
		.r = {0, 0, 0},
		.ival = {{0, 0, 0},
			 {0, 0, 0},
			 {0, 0, 0}},
	};

	printf("\nBench of fwd for Mark2_0:\n");
	start = clock();
	for (int i = 0; i < NBR_ITE; ++i) {
		ret = drive_to_cart(&mark2_0, q, out_tcp, &orient_angle);
	}
	printf("ret: %d\n", ret);
	stop = clock();
	t1 = (double)(stop-start)/CLOCKS_PER_SEC/NBR_ITE*1e6;
	printf("%d iterations took on average %f microsec per call\n",
			NBR_ITE, t1);

	double pos[3] = {out_tcp[0][3], out_tcp[1][3], out_tcp[2][3]};

	printf("\nBench of nominal calculation of qd and qdd for Mark2_0:\n");
	start = clock();
	for (int i = 0; i < NBR_ITE; ++i) {
		ret = inv_with_vel_acc(&mark2_0, pos, orient_angle, v, acc, q_inv, qd_inv, qdd_inv);
	}
	printf("ret: %d\n", ret);
	stop = clock();
	t1 = (double)(stop-start)/CLOCKS_PER_SEC/NBR_ITE*1e6;
	printf("%d iterations took on average %f microsec per call\n",
			NBR_ITE, t1);

	printf("\nBench of fwd for Mark2_0:\n");
	start = clock();
	for (int i = 0; i < NBR_ITE; ++i) {
		ret = fwd(&mark2_0, q, out_tcp, &orient_angle, 0.0, 0);
	}
	printf("ret: %d\n", ret);
	stop = clock();
	t1 = (double)(stop-start)/CLOCKS_PER_SEC/NBR_ITE*1e6;
	printf("%d iterations took on average %f microsec per call\n",
			NBR_ITE, t1);

	double pos2[3] = {out_tcp[0][3], out_tcp[1][3], out_tcp[2][3]};

	printf("\nBench of inv for Mark2_0:\n");
	start = clock();
	for (int i = 0; i < NBR_ITE; ++i) {
		ret = inv(&mark2_0, pos2, orient_angle, q_inv, 0.0, 0);
	}
	printf("ret: %d\n", ret);
	stop = clock();
	t1 = (double)(stop-start)/CLOCKS_PER_SEC/NBR_ITE*1e6;
	printf("%d iterations took on average %f microsec per call\n",
			NBR_ITE, t1);

	printf("\nBench of calculation of qd and qdd for Mark2_0:\n");
	start = clock();
	for (int i = 0; i < NBR_ITE; ++i) {
		ret = inv_with_vel_acc(&mark2_0, pos2, orient_angle, v, acc, q_inv, qd_inv, qdd_inv);
	}
	printf("ret: %d\n", ret);
	stop = clock();
	t1 = (double)(stop-start)/CLOCKS_PER_SEC/NBR_ITE*1e6;
	printf("%d iterations took on average %f microsec per call\n",
			NBR_ITE, t1);


	printf("\nBench of fwd_full for Mark2_0:\n");
	start = clock();
	for (int i = 0; i < NBR_ITE; ++i) {
		ret = fwd_full(&mark2_0, &mark2_0_lpoe, mark2_0_dynpars, &tool, q, out_tcp, &orient_angle, 0.0, 0);
	}
	printf("ret: %d\n", ret);
	stop = clock();
	t1 = (double)(stop-start)/CLOCKS_PER_SEC/NBR_ITE*1e6;
	printf("%d iterations took on average %f microsec per call\n",
			NBR_ITE, t1);


	double pos3[3] = {out_tcp[0][3], out_tcp[1][3], out_tcp[2][3]};

	printf("\nBench of inv_full for Mark2_0:\n");
	start = clock();
	for (int i = 0; i < NBR_ITE; ++i) {
		ret = inv_full(&mark2_0, &mark2_0_lpoe, mark2_0_dynpars, &tool, pos3, orient_angle, q_inv, 0.0, 0);
	}
	printf("ret: %d\n", ret);
	stop = clock();
	t1 = (double)(stop-start)/CLOCKS_PER_SEC/NBR_ITE*1e6;
	printf("%d iterations took on average %f microsec per call\n",
			NBR_ITE, t1);

	printf("\nBench of full calculation of qd and qdd for Mark2_0:\n");
	start = clock();
	for (int i = 0; i < NBR_ITE; ++i) {
		ret = inv_with_vel_acc_full(&mark2_0, &mark2_0_lpoe, mark2_0_dynpars, &tool, pos3, orient_angle, v, acc, q_inv, qd_inv, qdd_inv);
	}
	printf("ret: %d\n", ret);
	stop = clock();
	t1 = (double)(stop-start)/CLOCKS_PER_SEC/NBR_ITE*1e6;
	printf("%d iterations took on average %f microsec per call\n",
			NBR_ITE, t1);
}*/

int main(void)
{
	bench_vel_acc_calc();
	bench_vel_acc_calc_mark1_5();
	bench_vel_acc_calc_mark2_0();
	// bench_calib_kin_mark2_0();
}
