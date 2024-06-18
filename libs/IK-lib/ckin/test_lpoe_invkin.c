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

#include "example_params.h"
#include "lpoe_invkin.h"

static void test_inv_pos(void)
{
	int ret;
	double wcp[4][4] = {
		{0.0, 0.0, 1.0, 1.05},
		{0.0, 1.0, 0.0, 0},
		{-1.0, 0.0, 0.0, -0.328},
		{0.0, 0.0, 0.0, 1.0},
	};
	double th123[3];

	init_er6();

	double th123_true[] = {0.0, M_PI/2, 0.0};
	printf("\nelbow up, no overhead pos\n");
	ret = inv_pos(&ER6, wcp, 0, 0, th123);
	assert(ret == KIN_OK);
	for (int i = 0; i < 3; i++) {
		printf("th123[%d] = %f   (%f)\n", i, th123[i], th123_true[i]);
		assert(fabs(th123[i] - th123_true[i])  < 1e-10);
	}

	double th123_true_ed[] = {0.0, 2.95806049071, -2.58856476035};
	printf("\nelbow down, no overhead pos\n");
	ret = inv_pos(&ER6, wcp, 0, 1, th123);
	assert(ret == KIN_OK);
	for (int i = 0; i < 3; i++) {
		printf("th123[%d] = %f   (%f)\n", i, th123[i], th123_true_ed[i]);
		assert(fabs(th123[i] - th123_true_ed[i])  < 1e-10);
	}

	double th123_true_op[] = {M_PI, 3.943373747589586, -0.879900334448};
	printf("\nelbow up, overhead pos\n");
	ret = inv_pos(&ER6, wcp, 1, 0, th123);
	assert(ret == KIN_OK);
	for (int i = 0; i < 3; i++) {
		printf("th123[%d] = %f   (%f)\n", i, th123[i], th123_true_op[i]);
		assert(fabs(th123[i] - th123_true_op[i])  < 1e-10);
	}

	double th123_true_oped[] = {M_PI, 4.3836369590995865, -1.70866442591};
	printf("\nelbow down, overhead pos\n");
	ret = inv_pos(&ER6, wcp, 1, 1, th123);
	assert(ret == KIN_OK);
	for (int i = 0; i < 3; i++) {
		printf("th123[%d] = %f   (%f)\n", i, th123[i], th123_true_oped[i]);
		assert(fabs(th123[i] - th123_true_oped[i])  < 1e-10);
	}
}

static void test_nom_inv(void)
{
	int ret;
	double tcp[4][4] = {
		{-1.0, 0.0, 0.0, 1.02},
		{0.0, 1.0, 0.0, -1e-13},
		{0.0, 0.0, -1.0, 1.297},
		{0.0, 0.0, 0.0, 1.0},
	};
	double ax[6];

	init_kr20r1810();

	double ax_true[] = {0.0, -M_PI/2, M_PI/2, 0.0, M_PI/2, 0.0};
	printf("\nelbow up, no overhead pos, !neg_a5\n");
	ret = nom_inv(&KR20R1810, tcp, 0, 0, 0, ax);
	assert(ret == KIN_OK);
	for (int i = 0; i < 6; i++) {
		printf("ax[%d] = %f    true = %f\n", i, ax[i], ax_true[i]);
		assert(fabs(ax[i] - ax_true[i])  < 1e-12);
	}

}

static void test_nom_inv2(void)
{
	int ret;
	double tcp[4][4] = {
		{-5.28743883e-01, 2.10251906e-02, 8.48520976e-01, 2.57525780e00},
		{-2.51426437e-01, 9.50946755e-01, -1.80236001e-01, -4.61435958e-01},
		{-8.10687765e-01, -3.08639289e-01, -4.97520991e-01, -2.46479830e-02},
		{0.0, 0.0, 0.0, 1.0},
	};
	double ax[6];

	init_kr300r2500();

	double ax_true[] = {0.17453293, 0.17453293, 0.17453293, 0.17453293, 0.17453293, 0.17453293};
	printf("\nelbow up, no overhead pos, !neg_a5\n");
	ret = nom_inv(&KR300R2500, tcp, 0, 0, 0, ax);
	assert(ret == KIN_OK);
	for (int i = 0; i < 6; i++){
		printf("ax[%d] = %f    true = %f\n", i, ax[i], ax_true[i]);
		assert(fabs(ax[i] - ax_true[i])  < 1e-7);
	}
}

static void test_nom_inv3(void)
{
	int ret;
	double tcp[4][4] = {
		{0.0, 0.0, 1.0, 1.7735},
		{0.0, 1.0, 0.0, -0.0315},
		{-1.0, 0.0, 0.0, 2.0285},
		{0.0, 0.0, 0.0, 1.0},
	};
	double ax[6];

	init_er300();

	double ax_true[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	printf("\nelbow up, no overhead pos, !neg_a5\n");
	ret = nom_inv(&ER300, tcp, 0, 0, 0, ax);
	assert(ret == KIN_OK);
	for (int i = 0; i < 6; i++) {
		printf("ax[%d] = %f    true = %f\n", i, ax[i], ax_true[i]);
		assert(fabs(ax[i] - ax_true[i])  < 1e-7);
	}
}

static void test_nom_inv4(void)
{
	int ret;
	double tcp[4][4] = {
		{3.43631405e-01, 3.06296780e-01, 8.87749818e-01, 3.01721187e-01},
		{7.60945494e-01, -6.44800084e-01, -7.20750128e-02, -3.53267214e-01},
		{5.50344813e-01, 7.00296462e-01, -4.54648713e-01, 1.05529683},
		{0.0, 0.0, 0.0, 1.0},
	};
	double tcp2[4][4] = {
		{-0.19866933079506122, 0.0, 0.9800665778412416, 0.9872259254278706},
		{0.0, 1.0, 0.0, -1.3877787807814457e-17},
		{-0.9800665778412416, 0.0, -0.19866933079506122, 1.2843184295592396},
		{0.0, 0.0, 0.0, 1.0}};
	double ax[6];
	double ax2[6];

	void init_er6();

	double ax_true[] = {1.0, -0.8, 0.8, -1.0, 1.0, -1.0};
	double ax_true2[] = {0.0, 0.0, 0.0, 0.0, 0.2, 0.0};
	printf("\nelbow up, no overhead pos, !neg_a5\n");
	ret = nom_inv(&ER6, tcp, 0, 0, 0, ax);
	assert(ret == KIN_OK);
	ret = nom_inv(&ER6_offs, tcp, 0, 0, 0, ax2);
	assert(ret == KIN_OK);
	for (int i = 0; i < 6; i++) {
		printf("ax[%d] = %f    true = %f\n", i, ax[i], ax_true[i]);
		assert(fabs(ax[i] - ax_true[i])  < 1e-7);
		assert(fabs(ax2[i] - ax_true[i] + 0.1)  < 1e-7);
	}
	printf("\nelbow up, no overhead pos, !neg_a5\n");
	ret = nom_inv(&ER6, tcp2, 0, 0, 0, ax);
	assert(ret == KIN_OK);
	ret = nom_inv(&ER6_offs, tcp2, 0, 0, 0, ax2);
	assert(ret == KIN_OK);
	for (int i = 0; i < 6; i++) {
		printf("ax[%d] = %f    true = %f\n", i, ax[i], ax_true2[i]);
		assert(fabs(ax[i] - ax_true2[i])  < 1e-7);
		assert(fabs(ax2[i] - ax_true2[i] + 0.1)  < 1e-7);
	}
}

static void test_nom_inv5(void)
{
	int ret;
	double tcp[4][4] = {
		{0.5598411390018071, 0.7872420752814092, 0.25851076184132005, 1.5620668814390153},
		{0.7505132202974292, -0.3495560626524348, -0.5608390724813224, -1.1006620504893334},
		{-0.3511521112565634, 0.5079965294857312, -0.7865314493334075, 1.3609490965706188},
		{0.0, 0.0, 0.0, 1.0},
	};
	double ax[6];

	init_nj130();

	double ax_true[] = {0.5776931264175197, 0.34120144380759454, -2.151022849647937,
			0.7247080101265208, 0.5187273863635297, -2.0806656515790967};
	printf("\nTest NJ130 (parallel bar robot)\n");
	ret = nom_inv(&NJ130, tcp, 0, 0, 0, ax);
	assert(ret == KIN_OK);
	for (int i = 0; i < 6; i++) {
		printf("ax[%d] = %f    true = %f\n", i, ax[i], ax_true[i]);
		assert(fabs(ax[i] - ax_true[i])  < 1e-7);
	}

	printf("\nTest NJ130 (parallel bar robot with coupling matrix)\n");
	ret = nom_inv(&NJ130_2, tcp, 0, 0, 0, ax);
	assert(ret == KIN_OK);
	for (int i = 0; i < 6; i++) {
		printf("ax[%d] = %f    true = %f\n", i, ax[i], ax_true[i]);
		assert(fabs(ax[i] - ax_true[i])  < 1e-7);
	}
}

static void test_tcp2wcp_hw(void)
{
	double tcp1[4][4] = {
		{0.0, 0.0, 1.0, 1.875},
		{0.0, 1.0, 0.0, 0.0},
		{-1.0, 0.0, 0.0, 2.402224318643},
		{0.0, 0.0, 0.0, 1.0},
	};
	double q6_1 = 0.0;
	double wcp_true1[4][4] = {
		{sqrt(3)/2, 0.0, 0.5, 1.565},
		{0.0, 1.0, 0.0, 0.0},
		{-0.5, 0.0, sqrt(3)/2, 2.255},
		{0.0, 0.0, 0.0, 1.0},
	};
	double wcp1[4][4];
	double tcp2[4][4] = {
		{0.0, sqrt(3)/2, -0.5, 1.5375},
		{0.0, 0.5, sqrt(3)/2, 0.3420800344948533},
		{1.0, 0.0, 0.0, 2.255},
		{0.0, 0.0, 0.0, 1.0},
	};
	double q6_2 = -M_PI/2;
	double wcp_true2[4][4] = {
		{-sqrt(3)/2, 0.0, 0.5, 1.565},
		{0.5, 0.0, sqrt(3)/2, 0.0},
		{0.0, 1.0, 0.0, 2.255},
		{0.0, 0.0, 0.0, 1.0},
	};
	double wcp2[4][4];

	init_smart5nj42202_7();

	tcp2wcp_hw(&Smart5NJ42202_7, tcp1, q6_1, wcp1);
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			assert(fabs(wcp1[i][j] - wcp_true1[i][j])  < 1e-6);

	tcp2wcp_hw(&Smart5NJ42202_7, tcp2, q6_2, wcp2);
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			assert(fabs(wcp2[i][j] - wcp_true2[i][j])  < 1e-6);
}

static void test_nom_inv_iteration_hw(void)
{
	double tcp1[4][4] = {
		{0.0, 0.0, 1.0, 1.875},
		{0.0, 1.0, 0.0, 0.0},
		{-1.0, 0.0, 0.0, 2.402224318643},
		{0.0, 0.0, 0.0, 1.0},
	};
	double q6_1 = 0.0;
	int ret1;
	double q_inv1[6];
	double errs1[2];
	double q_inv1_true[6] = {0.0, 0.0, -M_PI/2, 0.0, 0.0, 0.0};

	double tcp2[4][4] = {
		{0.0, sqrt(3)/2, -0.5, 1.5375},
		{0.0, 0.5, sqrt(3)/2, 0.3420800344948533},
		{1.0, 0.0, 0.0, 2.255},
		{0.0, 0.0, 0.0, 1.0},
	};
	double q6_2 = -M_PI/2;
	int ret2;
	double q_inv2[6];
	double errs2[2];
	double q_inv2_true[6] = {0.0, 0.0, -M_PI/2, M_PI/2, M_PI, -M_PI/2};

	double q6_3 = -1.4;
	int ret3;
	double q_inv3[6];
	double errs3[2];
	double q_inv3_true[6] = {-6.83583687e-04, 6.39449698e-03,
			-1.54924498e+00, 1.61744539e+00,
			3.09813533e+00, -1.58788977};

	init_smart5nj42202_7();

	ret1 = nom_inv_iteration_hw(&Smart5NJ42202_7, tcp1, q6_1, 0, 0, 0, q_inv1, errs1);
	assert(ret1 == KIN_OK);
	assert(errs1[0] < 1e-8);
	assert(errs1[1] < 1e-8);
	for (int i = 0; i < 6; i++)
		assert(fabs(q_inv1[i] - q_inv1_true[i])  < 1e-8);

	ret2 = nom_inv_iteration_hw(&Smart5NJ42202_7, tcp2, q6_2, 0, 0, 0, q_inv2, errs2);
	assert(ret2 == KIN_OK);
	assert(errs2[0] < 1e-8);
	assert(errs2[1] < 1e-8);
	for (int i = 0; i < 6; i++)
		assert(fabs(q_inv2[i] - q_inv2_true[i])  < 1e-8);

	ret3 = nom_inv_iteration_hw(&Smart5NJ42202_7, tcp2, q6_3, 0, 0, 0, q_inv3, errs3);
	assert(ret3 == KIN_OK);
	assert(errs3[0] < 0.005);
	assert(errs3[1] < 0.1);
	for (int i = 0; i < 6; i++)
		assert(fabs(q_inv3[i] - q_inv3_true[i])  < 1e-8);
}

static void test_nom_inv_hw(void)
{
	int ret, ret2;
	double tcp[4][4] = {
		{0.0, 0.0, 1.0, 1.875},
		{-0.5, sqrt(3)/2, 0.0, 0.0},
		{-sqrt(3)/2, -0.5, 0.0, 2.4022243186433547},
		{0.0, 0.0, 0.0, 1.0},
	};
	double tcp2[4][4] = {
		{0.75, -sqrt(3)/4, -0.5, 1.5375},
		{0.5, sqrt(3)/2, 0.0, 0.0},
		{sqrt(3)/4, -0.25, sqrt(3)/2, 2.5970800344948533},
		{0.0, 0.0, 0.0, 1.0},
	};
	double q0[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double tol = 1e-8;
	double init_tol = 0.15;
	double ax[6];
	double ax2[6];

	init_smart5nj42202_7();

	double ax_true[] = {0.0, 0.0, -M_PI/2, 0.0, 0.0, M_PI/6};
	double ax_true2[] = {0.0, 0.0, -M_PI/2, 0.0, M_PI, 5*M_PI/6};
	printf("\nHollowWrist inv pos 1:\n");
	ret = nom_inv_hw(&Smart5NJ42202_7, tcp, 0, 0, 0, q0, tol, init_tol, ax);
	assert(ret == KIN_OK);
	ret2 = nom_inv_hw(&Smart5NJ42202_7_2, tcp, 0, 0, 0, q0, tol, init_tol, ax2);
	assert(ret2 == KIN_OK);
	for (int i = 0; i < 6; i++) {
		printf("ax[%d] = %f (%f)    true = %f\n", i, ax[i], ax2[i], ax_true[i]);
		assert(fabs(ax[i] - ax_true[i])  < 1e-3);
		assert(fabs(ax2[i] - ax_true[i])  < 1e-3);
	}

	printf("\nHollowWrist inv pos 2:\n");
	ret = nom_inv_hw(&Smart5NJ42202_7, tcp2, 0, 0, 0, q0, tol, init_tol, ax);
	assert(ret == KIN_OK);
	ret2 = nom_inv_hw(&Smart5NJ42202_7_2, tcp2, 0, 0, 0, q0, tol, init_tol, ax2);
	assert(ret2 == KIN_OK);
	for (int i = 0; i < 6; i++) {
		printf("ax[%d] = %f (%f)    true = %f\n", i, ax[i], ax2[i], ax_true2[i]);
		assert(fabs(ax[i] - ax_true2[i])  < 1e-3);
		assert(fabs(ax2[i] - ax_true2[i])  < 1e-3);
	}

	ret = nom_inv_hw2(&Smart5NJ42202_7, tcp2, 0, 0, 0, q0, tol, ax);
	printf("Calling inv2\n");
	printf("ret: %d\n", ret);
	assert(ret == KIN_OK);
	ret2 = nom_inv_hw2(&Smart5NJ42202_7, tcp2, 0, 0, 0, q0, tol, ax2);
	assert(ret2 == KIN_OK);
	for (int i = 0; i < 6; i++) {
		printf("ax[%d] = %f (%f)    true = %f\n", i, ax[i], ax2[i], ax_true2[i]);
		assert(fabs(ax[i] - ax_true2[i])  < 1e-3);
		assert(fabs(ax2[i] - ax_true2[i])  < 1e-3);
	}
}

static void test_nom_inv_ur(void)
{
	int ret, ret2;
	double tcp[4][4] = {
		{-1.0, 0.0, 0.0, 0.0},
		{0.0, 0.0, -1.0, -0.291},
		{0.0, -1.0, 0.0, 1.485},
		{0.0, 0.0, 0.0, 1.0},
	};
	double tcp2[4][4] = {
		{0.0, 0.0, 1.0, 0.291},
		{-1.0, 0.0, 0.0, 0.0},
		{0.0, -1.0, 0.0, 1.485},
		{0.0, 0.0, 0.0, 1.0},
	};
	double tcp3[4][4] = {
		{0.0, 1.0, 0.0, -1.304},
		{0.0, 0.0, -1.0, -0.291},
		{-1.0, 0.0, 0.0, 0.181},
		{0.0, 0.0, 0.0, 1.0},
	};
	double tcp4[4][4] = {
		{0.16063913248869824, 0.9307646784565972,
		 0.32843900872287657, -0.6214509273559367},
		{-0.3815560090220246, 0.3654511735326534,
		 -0.8490350120830068, -0.3404194081969609},
		{-0.9102802211915093, 0.011070370418094983,
		 0.41384461553382923, 1.3086209776646436},
		{0.0, 0.0, 0.0, 1.0}
	};
	double ax[6];
	double ax2[6];
	double ax3[6];
	double ax4[6];

	double ax_true[] = {0.0, -M_PI/2, 0.0, -M_PI/2, 0.0, 0.0};
	double ax_true2[] = {M_PI/2, -M_PI/2, 0.0, -M_PI/2, 0.0, 0.0};
	double ax_true3[] = {0.0, 0.0, 0.0, -M_PI/2, 0.0, 0.0};
	double ax_true4_1[] = {0.09999999999999987, -0.9107223282823678,
			-0.30000000000000276, -0.8892776717176297,
			0.4999999999999971, 0.6000000000000003};
	double ax_true4_2[] = {0.1, -1.2,  0.3, -1.2,  0.5,  0.6};

	printf("\nUR10e inv pos 1:\n");
	ret = nom_inv_ur(&UR10e, tcp, 0, 0, 0, ax);
	printf("ret: %d\n", ret);
	assert(ret == KIN_OK);
	for (int i = 0; i < 6; i++) {
		printf("ax[%d] = %f    true = %f\n", i, ax[i], ax_true[i]);
		assert(fabs(ax[i] - ax_true[i])  < 1e-3);
	}

	printf("\nUR10e inv pos 2:\n");
	ret2 = nom_inv_ur(&UR10e, tcp2, 0, 0, 0, ax2);
	printf("ret: %d\n", ret2);
	assert(ret2 == KIN_OK);
	for (int i = 0; i < 6; i++) {
		printf("ax[%d] = %f    true = %f\n", i, ax2[i], ax_true2[i]);
		assert(fabs(ax2[i] - ax_true2[i])  < 1e-3);
	}

	printf("\nUR10e inv pos 3:\n");
	ret2 = nom_inv_ur(&UR10e, tcp3, 0, 0, 0, ax3);
	printf("ret: %d\n", ret2);
	assert(ret2 == KIN_OK);
	for (int i = 0; i < 6; i++) {
		printf("ax[%d] = %f    true = %f\n", i, ax3[i], ax_true3[i]);
		assert(fabs(ax3[i] - ax_true3[i])  < 1e-3);
	}

	printf("\nUR10e inv pos 4_1:\n");
	ret2 = nom_inv_ur(&UR10e, tcp4, 0, 0, 0, ax4);
	printf("ret: %d\n", ret2);
	assert(ret2 == KIN_OK);
	for (int i = 0; i < 6; i++) {
		printf("ax[%d] = %f    true = %f\n", i, ax4[i], ax_true4_1[i]);
		assert(fabs(ax4[i] - ax_true4_1[i])  < 1e-3);
	}

	printf("\nUR10e inv pos 4_2:\n");
	ret2 = nom_inv_ur(&UR10e, tcp4, 0, 1, 0, ax4);
	printf("ret: %d\n", ret2);
	assert(ret2 == KIN_OK);
	for (int i = 0; i < 6; i++) {
		printf("ax[%d] = %f    true = %f\n", i, ax4[i], ax_true4_2[i]);
		assert(fabs(ax4[i] - ax_true4_2[i])  < 1e-3);
	}
}

int main(void)
{
	test_inv_pos();
	test_nom_inv();
	test_nom_inv2();
	test_nom_inv3();
	test_nom_inv4();
	test_nom_inv5();
	test_tcp2wcp_hw();
	test_nom_inv_iteration_hw();
	test_nom_inv_hw();
	test_nom_inv_ur();
}
