/*
 * Copyright (C) 2020 Cognibotics
 *
 * Use or copying of this file or any part of it, without explicit
 * permission from Cognibotics is not allowed in any way.
 */
#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <time.h>

#include "dyn.h"
#include "concept_prototype.h"
#include "kinematics.h"
#include "kin_model.h"
#include "mark1_5.h"
#include "mark1_5_lpoe.h"
#include "mark2_0.h"
#include "mark2_0_lpoe.h"

#define NBR_ITE 100000

static void bench_dyn_inv_agile(void)
{
	struct dyn_params_link tool = {
	    .m = 4,
	    .r = {0, 0, 0},
	    .ival = {{0, 0, 0},
		     {0, 0, 0},
		     {0, 0, 0}},
	};
	double q[4] = {0.1, 1.5 - M_PI / 2, -0.3, 45};
	double qd[4] = {0.5, -0.5, 0.5, 30};
	double qdd[4] = {2.3, 4.5, 6.23, -301.4};
	double g[3] = {0.0, 0.0, -9.82};

	int ret;
	double trq[4];

	printf("\nBench of dyn_inv_agile without coriolis/centrifugal terms:\n");

	double start, stop, t1;

	start = clock();
	for (int i = 0; i < NBR_ITE; ++i) {
		ret = dyn_inv_agile(&kin_model, &concept_prototype,
				    concept_prototype_dynpars,
				    &tool, q, qd, qdd, g, 0, trq);
	}
	printf("ret: %d\n", ret);
	stop = clock();
	t1 = (double)(stop - start) / CLOCKS_PER_SEC / NBR_ITE * 1e6;
	printf("\n%d iterations took on average %f microsec per call\n",
	       NBR_ITE, t1);

	printf("\n\nBench of dyn_inv_agile including coriolis/centrifugal terms:\n");

	start = clock();
	for (int i = 0; i < NBR_ITE; ++i) {
		ret = dyn_inv_agile(&kin_model, &concept_prototype,
				    concept_prototype_dynpars,
				    &tool, q, qd, qdd, g, 1, trq);
	}
	printf("ret: %d\n", ret);
	stop = clock();
	t1 = (double)(stop - start) / CLOCKS_PER_SEC / NBR_ITE * 1e6;
	printf("\n%d iterations took on average %f microsec per call\n",
	       NBR_ITE, t1);
}

static void bench_dyn_inv_agile_mark1_5(void)
{
	struct dyn_params_link tool = {
	    .m = 4,
	    .r = {0, 0, 0},
	    .ival = {{0, 0, 0},
		     {0, 0, 0},
		     {0, 0, 0}},
	};
	double q[4] = {0.1, 1.5 - M_PI / 2, -0.3, 2.5};
	double qd[4] = {0.5, -0.5, 0.5, 3.0};
	double qdd[4] = {2.3, 4.5, 6.23, -31.4};
	double g[3] = {0.0, 0.0, -9.82};

	int ret;
	double trq[4];

	printf("\nBench of dyn_inv Mark1_5 without coriolis/centrifugal terms:\n");

	double start, stop, t1;

	start = clock();
	for (int i = 0; i < NBR_ITE; ++i) {
		ret = dyn_inv_agile(&mark1_5, &mark1_5_lpoe, mark1_5_dynpars,
				    &tool, q, qd, qdd, g, 0, trq);
	}
	printf("ret: %d\n", ret);
	stop = clock();
	t1 = (double)(stop - start) / CLOCKS_PER_SEC / NBR_ITE * 1e6;
	printf("\n%d iterations took on average %f microsec per call\n",
	       NBR_ITE, t1);

	printf("\n\nBench of dyn_inv Mark1_5 including coriolis/centrifugal terms:\n");

	start = clock();
	for (int i = 0; i < NBR_ITE; ++i) {
		ret = dyn_inv_agile(&mark1_5, &mark1_5_lpoe, mark1_5_dynpars,
				    &tool, q, qd, qdd, g, 1, trq);
	}
	printf("ret: %d\n", ret);
	stop = clock();
	t1 = (double)(stop - start) / CLOCKS_PER_SEC / NBR_ITE * 1e6;
	printf("\n%d iterations took on average %f microsec per call\n",
	       NBR_ITE, t1);
}

static void bench_inertia_matrix_mark2_0(void)
{
	struct dyn_params_link tool = {
	    .m = 4,
	    .r = {0, 0, 0},
	    .ival = {{0, 0, 0},
		     {0, 0, 0},
		     {0, 0, 0}},
	};
	double q[4] = {0.0, 0.0, 0.0, 0.0};
	double imat[4][4];
	int ret;

	printf("\nBench of inertia_matrix for Mark2_0:\n");

	double start, stop, t1;

	start = clock();
	for (int i = 0; i < NBR_ITE; ++i) {
		ret = inertia_matrix(&mark2_0, &mark2_0_lpoe, mark2_0_dynpars,
				     &tool, q, imat);
	}
	stop = clock();
	t1 = (double)(stop - start) / CLOCKS_PER_SEC / NBR_ITE * 1e6;
	printf("\n%d iterations took on average %f microsec per call\n",
	       NBR_ITE, t1);

	printf("\nBench of inertia_matrix in joint space for Mark2_0:\n");

	start = clock();
	for (int i = 0; i < NBR_ITE; ++i) {
		ret = inertia_matrix_new_joint_space(&mark2_0, &mark2_0_lpoe, mark2_0_dynpars,
						     &tool, q, imat);
	}
	stop = clock();
	t1 = (double)(stop - start) / CLOCKS_PER_SEC / NBR_ITE * 1e6;
	printf("\n%d iterations took on average %f microsec per call\n",
	       NBR_ITE, t1);
}

static void bench_dyn_inv_agile_mark2_0(void)
{
	struct dyn_params_link tool = {
	    .m = 4,
	    .r = {0, 0, 0},
	    .ival = {{0, 0, 0},
		     {0, 0, 0},
		     {0, 0, 0}},
	};
	double q[4] = {0.1, 1.5 - M_PI / 2, -0.3, 2.5};
	double qd[4] = {0.5, -0.5, 0.5, 3.0};
	double qdd[4] = {2.3, 4.5, 6.23, -31.4};
	double g[3] = {0.0, 0.0, -9.82};

	int ret;
	double trq[4];

	double start, stop, t1;

	double qe[9] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7 - M_PI_2, 0.8, 0.9};
	double qde[9] = {0.1, -0.1, 0.1, -0.1, 0.1, -0.1, 0.1, -0.1, 0.1};
	double qdde[9] = {-0.1, 0.1, -0.1, 0.1, -0.1, 0.1, -0.1, 0.1, -0.1};

	printf("\nBench of dyn_inv_agile_extended Mark2_0 without coriolis/centrifugal terms:\n");
	start = clock();
	for (int i = 0; i < NBR_ITE; ++i) {
		ret = dyn_inv_agile_extended(&mark2_0_lpoe, mark2_0_dynpars, &tool, qe, qde, qdde, g, trq);
	}
	printf("ret: %d\n", ret);
	stop = clock();
	t1 = (double)(stop - start) / CLOCKS_PER_SEC / NBR_ITE * 1e6;
	printf("\n%d iterations took in average %f microsec per call\n",
	       NBR_ITE, t1);

	printf("\nBench of dyn_inv Mark2_0 without coriolis/centrifugal terms:\n");
	start = clock();
	for (int i = 0; i < NBR_ITE; ++i) {
		ret = dyn_inv_agile(&mark2_0, &mark2_0_lpoe, mark2_0_dynpars,
				    &tool, q, qd, qdd, g, 0, trq);
	}
	printf("ret: %d\n", ret);
	stop = clock();
	t1 = (double)(stop - start) / CLOCKS_PER_SEC / NBR_ITE * 1e6;
	printf("\n%d iterations took on average %f microsec per call\n",
	       NBR_ITE, t1);

	printf("\n\nBench of dyn_inv Mark2_0 including coriolis/centrifugal terms:\n");

	start = clock();
	for (int i = 0; i < NBR_ITE; ++i) {
		ret = dyn_inv_agile(&mark2_0, &mark2_0_lpoe, mark2_0_dynpars,
				    &tool, q, qd, qdd, g, 1, trq);
	}
	printf("ret: %d\n", ret);
	stop = clock();
	t1 = (double)(stop - start) / CLOCKS_PER_SEC / NBR_ITE * 1e6;
	printf("\n%d iterations took on average %f microsec per call\n",
	       NBR_ITE, t1);
}

static void bench_dyn_inv_agile_grav_mark2_0(void)
{
	struct dyn_params_link tool = {
	    .m = 4,
	    .r = {0, 0, 0},
	    .ival = {{0, 0, 0},
		     {0, 0, 0},
		     {0, 0, 0}},
	};
	double q[4] = {0.1, 1.5 - M_PI / 2, -0.3, 2.5};
	double g[3] = {0.0, 0.0, -9.82};

	int ret;
	double trq[4];

	printf("\nBench of dyn_inv_grav Mark2_0:\n");

	double start, stop, t1;

	start = clock();
	for (int i = 0; i < NBR_ITE; ++i) {
		ret = dyn_inv_agile_grav(&mark2_0, &mark2_0_lpoe, mark2_0_dynpars,
					 &tool, q, g, trq);
	}
	printf("ret: %d\n", ret);
	stop = clock();
	t1 = (double)(stop - start) / CLOCKS_PER_SEC / NBR_ITE * 1e6;
	printf("\n%d iterations took on average %f microsec per call\n",
	       NBR_ITE, t1);
}

static void bench_dyn_inv_agile_granular_mark2_0(void)
{
	struct dyn_params_link tool = {
	    .m = 4,
	    .r = {0, 0, 0},
	    .ival = {{0, 0, 0},
		     {0, 0, 0},
		     {0, 0, 0}},
	};

	double q[4] = {0.1, 0.2, 0.3, 0.4};
	double qd[4] = {1.0, -1.0, 1.0, -1.0};
	double qdd[4] = {-10.0, 10.0, -10.0, 10.0};
	double g[3] = {0.0, 0.0, -9.82};

	int ret;
	double trq_grav[4];
	double trq_vel[4];
	double trq_acc[4];
	double trq_ref[4];

	double start, stop, t1;

	printf("\nBench of dyn_inv_granular Mark2_0 without coriolis/centrifugal terms:\n");

	start = clock();
	for (int i = 0; i < NBR_ITE; ++i) {
		ret = dyn_inv_agile_granular(&mark2_0, &mark2_0_lpoe, mark2_0_dynpars,
					     &tool, q, qd, qdd, g, 0, trq_grav, trq_vel, trq_acc);
	}
	printf("ret: %d\n", ret);
	stop = clock();
	t1 = (double)(stop - start) / CLOCKS_PER_SEC / NBR_ITE * 1e6;
	printf("\n%d iterations took on average %f microsec per call\n",
	       NBR_ITE, t1);

	printf("\nBench of dyn_inv_granular Mark2_0 including coriolis/centrifugal terms:\n");

	start = clock();
	for (int i = 0; i < NBR_ITE; ++i) {
		ret = dyn_inv_agile_granular(&mark2_0, &mark2_0_lpoe, mark2_0_dynpars,
					     &tool, q, qd, qdd, g, 0, trq_grav, trq_vel, trq_acc);
	}
	printf("ret: %d\n", ret);
	stop = clock();
	t1 = (double)(stop - start) / CLOCKS_PER_SEC / NBR_ITE * 1e6;
	printf("\n%d iterations took on average %f microsec per call\n",
	       NBR_ITE, t1);
}

int main(void)
{
	bench_dyn_inv_agile();
	bench_dyn_inv_agile_mark1_5();
	bench_inertia_matrix_mark2_0();
	bench_dyn_inv_agile_mark2_0();
	bench_dyn_inv_agile_grav_mark2_0();
	bench_dyn_inv_agile_granular_mark2_0();
}
