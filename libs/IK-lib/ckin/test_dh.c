/*
 * Copyright (C) 2018, 2019 Cognibotics
 *
 * Use or copying of this file or any part of it, without explicit
 * permission from Cognibotics is not allowed in any way.
 */
#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

#include "dh.h"

static void test_dhmatrix(void)
{
	struct model_dh example_rob = {
		.a = {0.160, 0.780, 0.150, 0.0, 0.0, 0.0},
		.d = {0.520, 0.0, 0.0, -0.860, 0.0, -0.153},
		.alpha = {-M_PI/2, 0, M_PI/2, -M_PI/2, M_PI/2, 0},
		.rot_dir = {-1, 1, 1, 1, 1, 1},
		.rot_offs = {0, 0, -M_PI/2, 0, 0, 0},
	};
	double th = M_PI/2;
	double dh0_true[4][4] = {
		{0.0, 0.0, -1.0, 0.0},
		{1.0, 0.0, 0.0, 0.16},
		{0.0, -1.0, 0.0, 0.52},
		{0.0, 0.0, 0.0, 1.0},
	};

	double dh0[4][4];
	dhmatrix(example_rob, 0, th, dh0);
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			assert(fabs(dh0[i][j] - dh0_true[i][j])  < 1e-12);
}

static void test_xrot_ytr(void)
{
	double xrot = M_PI/6;
	double ytr = 0.1337;
	double res_true[4][4] = {
		{1.0, 0.0, 0.0, 0.0},
		{0.0, sqrt(3)/2, -0.5, 0.1337*sqrt(3)/2},
		{0.0, 0.5, sqrt(3)/2, 0.1337/2},
		{0.0, 0.0, 0.0, 1.0},
	};

	double res[4][4];
	xrot_ytr(xrot, ytr, res);
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			assert(fabs(res[i][j] - res_true[i][j])  < 1e-12);
}

static void test_inv_pos_dh(void)
{
	struct model_dh example_rob = {
		.a = {0.160, 0.780, 0.150, 0.0, 0.0, 0.0},
		.d = {0.520, 0.0, 0.0, -0.860, 0.0, -0.153},
		.alpha = {-M_PI/2, 0, M_PI/2, -M_PI/2, M_PI/2, 0},
		.rot_dir = {-1, 1, 1, 1, 1, 1},
		.rot_offs = {0, 0, -M_PI/2, 0, 0, 0},
	};
	double wcp[4][4] = {
		{1.0, 0.0, 0.0, 1.020},
		{0.0, 1.0, 0.0, -1e-13},
		{0.0, 0.0, 1.0, 1.45},
		{0.0, 0.0, 0.0, 1.0},
	};
	double th123[4];

	double th123_true[4] = {0.0, -M_PI/2, 0.0, 0.0};
	printf("\nelbow up, no overhead pos\n");
	inv_pos_dh(example_rob, wcp, 0, 0, th123);
	for (int i = 0; i < 4; i++) {
		printf("th123[%d] = %f\n", i, th123[i]);
		assert(fabs(th123[i] - th123_true[i])  < 1e-12);
	}

	double th123_true_ed[4] = {0.0, -0.078172457272897122, -2.7962296703537821, 0.0};
	printf("\nelbow down, no overhead pos\n");
	inv_pos_dh(example_rob, wcp, 0, 1, th123);
	for (int i = 0; i < 4; i++) {
		printf("th123[%d] = %f\n", i, th123[i]);
		assert(fabs(th123[i] - th123_true_ed[i])  < 1e-12);
	}

	double th123_true_op[4] = {M_PI, -2.9308124042539734, -0.53644499274853796, 0.0};
	printf("\nelbow up, overhead pos\n");
	inv_pos_dh(example_rob, wcp, 1, 0, th123);
	for (int i = 0; i < 4; i++) {
		printf("th123[%d] = %f\n", i, th123[i]);
		assert(fabs(th123[i] - th123_true_op[i])  < 1e-12);
	}

	double th123_true_oped[4] = {M_PI, -2.0174437761908202, -2.259784677605245, 0.0};
	printf("\nelbow down, overhead pos\n");
	inv_pos_dh(example_rob, wcp, 1, 1, th123);
	for (int i = 0; i < 4; i++) {
		printf("th123[%d] = %f\n", i, th123[i]);
		assert(fabs(th123[i] - th123_true_oped[i])  < 1e-12);
	}
}

static void test_nom_inv_dh(void)
{
	struct model_dh example_rob = {
		.a = {0.160, 0.780, 0.150, 0.0, 0.0, 0.0},
		.d = {0.520, 0.0, 0.0, -0.860, 0.0, -0.153},
		.alpha = {-M_PI/2, 0, M_PI/2, -M_PI/2, M_PI/2, 0},
		.rot_dir = {-1, 1, 1, 1, 1, 1},
		.rot_offs = {0, 0, -M_PI/2, 0, 0, 0},
	};
	double tcp[4][4] = {
		{-1.0, 0.0, 0.0, 1.02},
		{0.0, 1.0, 0.0, -1e-13},
		{0.0, 0.0, -1.0, 1.297},
		{0.0, 0.0, 0.0, 1.0},
	};
	double ax[7];

	double ax_true[7] = {0.0, -M_PI/2, M_PI/2, 0.0, M_PI/2, 0.0, 0.0};
	printf("\nelbow up, no overhead pos, !neg_a5\n");
	nom_inv_dh(example_rob, tcp, 0, 0, 0, ax);
	for (int i = 0; i < 7; i++) {
		printf("ax[%d] = %f    true = %f\n", i, ax[i], ax_true[i]);
		assert(fabs(ax[i] - ax_true[i])  < 1e-12);
	}

}

static void test_nom_inv_dh2(void)
{
	struct model_dh example_rob = {
		.a = {0.350, 1.15, -0.041, 0.0, 0.0, 0.0},
		.d = {0.675, 0.0, 0.0, -1.0, 0.0, -0.24},
		.alpha = {-M_PI/2, 0, M_PI/2, -M_PI/2, M_PI/2, 0},
		.rot_dir = {-1, 1, 1, 1, 1, 1},
		.rot_offs = {0, 0, -M_PI/2, 0, 0, 0},
	};
	double tcp[4][4] = {
		{-5.28743883e-01, 2.10251906e-02, 8.48520976e-01, 2.57525780e00},
		{-2.51426437e-01, 9.50946755e-01, -1.80236001e-01, -4.61435958e-01},
		{-8.10687765e-01, -3.08639289e-01, -4.97520991e-01, -2.46479830e-02},
		{0.0, 0.0, 0.0, 1.0},
	};
	double ax[7];

	double ax_true[7] = {0.17453293, 0.17453293, 0.17453293, 0.17453293, 0.17453293, 0.17453293, 0.0};
	printf("\nelbow up, no overhead pos, !neg_a5\n");
	nom_inv_dh(example_rob, tcp, 0, 0, 0, ax);
	for (int i = 0; i < 7; i++) {
		printf("ax[%d] = %f    true = %f\n", i, ax[i], ax_true[i]);
		assert(fabs(ax[i] - ax_true[i])  < 1e-7);
	}
}


static void test_nom_inv_dh3(void)
{
	struct model_dh example_rob = {
		.a = {0.400, 1.1, 0.23, 0.0, 0.0, 0.0},
		.d = {0.6985, 0.0, -0.0315, -1.105, 0.0, -0.2685},
		.alpha = {-M_PI/2, 0, M_PI/2, -M_PI/2, M_PI/2, 0},
		.rot_dir = {-1, 1, 1, 1, 1, 1},
		.rot_offs = {0, -M_PI/2, 0, 0, 0, 0},
	};
	double tcp[4][4] = {
		{0.0, 0.0, 1.0, 1.7735},
		{0.0, 1.0, 0.0, -0.0315},
		{-1.0, 0.0, 0.0, 2.0285},
		{0.0, 0.0, 0.0, 1.0},
	};
	double ax[7];

	double ax_true[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	printf("\nelbow up, no overhead pos, !neg_a5\n");
	nom_inv_dh(example_rob, tcp, 0, 0, 0, ax);
	for (int i = 0; i < 7; i++) {
		printf("ax[%d] = %f    true = %f\n", i, ax[i], ax_true[i]);
		assert(fabs(ax[i] - ax_true[i])  < 1e-7);
	}
}

int main(void)
{
	test_dhmatrix();
	test_xrot_ytr();
	test_inv_pos_dh();
	test_nom_inv_dh();
	test_nom_inv_dh2();
	test_nom_inv_dh3();
}
