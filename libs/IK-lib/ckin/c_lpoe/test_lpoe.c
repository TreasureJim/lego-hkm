/*
 * Copyright (C) 2017, 2018, 2019 Cognibotics
 *
 * Use or copying of this file or any part of it, without explicit
 * permission from Cognibotics is not allowed in any way.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lpoe.h"


static void test_twist_ops(void)
{
	double t[] = {1, 2, 3, 4, 5, 6};

	assert(twist_v(t) == t);
	assert(twist_omega(t) == t+3);
	assert(twist_v(t)[0] == 1);
	assert(twist_v(t)[1] == 2);
	assert(twist_v(t)[2] == 3);
	assert(twist_omega(t)[0] == 4);
	assert(twist_omega(t)[1] == 5);
	assert(twist_omega(t)[2] == 6);
}

static void test_twist_omega_hat(void)
{
	double s[] = {0, 0, 0, 1, 2, 3};
	double hat[3][3];
	double m_hat[3][3] = {
		{0, -3, 2},
		{3, 0, -1},
		{-2, 1, 0},
	};

	twist_omega_hat(s, hat);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			assert(fabs(hat[i][j] - m_hat[i][j])  < 1e-9);
}

static void test_twist_R_1(void)
{
	double s[] = {0, 0, 0, 1, 2, 3};
	double m[3][3];
	double m_hat[3][3] = {
		{-0.694920557641311, 0.713520990527788, 0.089292858861912},
		{-0.192006972792000, -0.303785044339470, 0.933192353823647},
		{0.692978167741770, 0.631349699383717, 0.348107477830265},
	};

	twist_R(s, 1, m);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			assert(fabs(m[i][j] - m_hat[i][j])  < 1e-9);
}

static void test_twist_R_2(void)
{
	double s[] = {0, 0, 0, 0, 0, 0};
	double m[3][3];
	double m_hat[3][3] = {
		{1, 0, 0},
		{0, 1, 0},
		{0, 0, 1},
	};
	twist_R(s, 1, m);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			assert(fabs(m[i][j] - m_hat[i][j])  < 1e-9);
}

static void test_twist_exp_hat(void)
{
	double t[] = {1, 2, 3, -1, 2, -3};
	double m_ref[4][4] = {
		{-0.694920557641312, -0.713520990527788, 0.089292858861912, 1.906872723367385},
		{0.192006972791999, -0.303785044339471, -0.933192353823647, -1.288346649199898},
		{0.692978167741770, -0.631349699383718, 0.348107477830265, 0.505477992744273},
		{0.0, 0.0, 0.0, 1.0},
	};
	double m[4][4] = {0};

	twist_exp_hat(t, 1, m);
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			assert(fabs(m[i][j] - m_ref[i][j])  < 1e-9);
}

static void test_inv_t44(void)
{
	double t44[4][4] = {
		{0.0, 0.0, -1.0, 0.0},
		{1.0, 0.0, 0.0, 0.16},
		{0.0, -1.0, 0.0, 0.52},
		{0.0, 0.0, 0.0, 1.0},
	};
	double t44inv_true[4][4] = {
		{0.0, 1.0, 0.0, -0.16},
		{0.0, 0.0, -1.0, 0.52},
		{-1.0, 0.0, 0.0, 0.0},
		{0.0, 0.0, 0.0, 1.0},
	};
	double t44inv[4][4];
	inv_t44(t44, t44inv);
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			assert(fabs(t44inv[i][j] - t44inv_true[i][j])  < 1e-12);
}

static void test_rot_z(void)
{
	double ang = 1.0;
	double t44_zrot_true[4][4] = {
		{cos(ang), -sin(ang), 0.0, 0.0},
		{sin(ang), cos(ang), 0.0, 0.0},
		{0.0, 0.0, 1.0, 0.0},
		{0.0, 0.0, 0.0, 1.0},
	};
	double t44_zrot[4][4];
	rot_z(ang, t44_zrot);
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			assert(fabs(t44_zrot[i][j] - t44_zrot_true[i][j])  < 1e-12);
}

static void test_rot_y(void)
{
	double ang = -0.5738;
	double t44_yrot_true[4][4] = {
		{cos(ang), 0.0, sin(ang), 0.0},
		{0.0, 1.0, 0.0, 0.0},
		{-sin(ang), 0.0, cos(ang), 0.0},
		{0.0, 0.0, 0.0, 1.0},
	};
	double t44_yrot[4][4];
	rot_y(ang, t44_yrot);
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			assert(fabs(t44_yrot[i][j] - t44_yrot_true[i][j])  < 1e-12);
}

static void test_rot_x(void)
{
	double ang = 1.337;
	double t44_xrot_true[4][4] = {
		{1.0, 0.0, 0.0, 0.0},
		{0.0, cos(ang), -sin(ang), 0.0},
		{0.0, sin(ang), cos(ang), 0.0},
		{0.0, 0.0, 0.0, 1.0},
	};
	double t44_xrot[4][4];
	rot_x(ang, t44_xrot);
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			assert(fabs(t44_xrot[i][j] - t44_xrot_true[i][j])  < 1e-12);
}

static void test_fwd_and_jac(void)
{
	struct model_lpoe_link er6[] = {
		{
			.s = {0, 0, 0, 0, 0, 0},
			.trans = {
				{1, 0, 0, 0},
				{0, 1, 0, 0},
				{0, 0, 1, 0.3165},
				{0, 0, 0, 1}},
		},
		{
			.s = {0, 0, 0, 0, 0, -1},
			.trans = {
				{1, 0, 0, 0.16},
				{0, 1, 0, -0.115},
				{0, 0, 1, 0.0955},
				{0, 0, 0, 1}},
		},
		{
			.s = {0, 0, 0, 0, 1, 0},
			.trans = {
				{1, 0, 0, 0},
				{0, 1, 0, -0.003},
				{0, 0, 1, 0.68},
				{0, 0, 0, 1}},
		},
		{
			.s = {0, 0, 0, 0, 1, 0},
			.trans = {
				{0,  0, 1, 0.121},
				{0,  1, 0, 0.118},
				{-1, 0, 0, 0.21},
				{0,  0, 0, 1}},
		},
		{
			.s = {0, 0, 0, 0, 0, -1},
			.trans = {
				{1, 0, 0, 0},
				{0, 1, 0, -0.059},
				{0, 0, 1, 0.619},
				{0, 0, 0, 1}},
		},
		{
			.s = {0, 0, 0, 0, 1, 0},
			.trans = {
				{1, 0, 0, 0},
				{0, 1, 0, 0.059},
				{0, 0, 1, 0},
				{0, 0, 0, 1}},
		},
		{
			.s = {0, 0, 0, 0, 0, -1},
			.trans = {
				{1, 0, 0, 0},
				{0, 1, 0, 0},
				{0, 0, 1, 0.089},
				{0, 0, 0, 1}},
		},
	};
	struct model_lpoe model;
	lpoe_init(&model);
	model.n_screw = 7;
	model.n_joints = 6;
	model.lrob = er6;
	double q[] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
	double fwd_true[4][4] = {
		{-0.63894042, -0.5507876 ,  0.53701783,  1.08775862},
		{-0.74204545,  0.62533077, -0.241516  , -0.12583937},
		{-0.20278976, -0.55280597, -0.80825854,  0.8360277 },
		{ 0.        ,  0.        ,  0.        ,  1.        },};
	double fwd[7][4][4];
	fwd_lpoe(&model, q, 7, fwd);
	for (int i=0; i<4; i++){
		for (int j=0; j<4; j++){
			assert(fabs(fwd[6][i][j] - fwd_true[i][j])  < 1e-7);
		}
	}
	double jac_true[6][6] = {
		{0.0, -4.09941716e-01, -1.07305754, -1.29573883e-01, -8.13374862e-01,  -3.03624813e-01},
		{0.0, 4.11313677e-02, 1.07664876e-01, -1.29141682, 2.70226775e-01, -1.32815198},
		{0.0, 0.16, 2.95095145e-01, -2.77555756e-17, 9.62679691e-01, 1.95133120e-01},
		{0.0, 9.98334166e-02, 9.98334166e-02, -8.73198304e-01, -9.38117247e-02, -5.37017831e-01},
		{0.0, 9.95004165e-01, 9.95004165e-01, 8.76120655e-02, 9.35098135e-01, 2.41515997e-01},
		{-1.0, 0.0, 0.0, 4.79425539e-01, -3.41746746e-01, 8.08258543e-01},};
	double jac[6][6];
	double fwd2[4][4];
	double jac_t[6][6];
	double fwd3[4][4];
	spatial_jacobian(&model, q, fwd2, jac);
	spatial_jacobian_transpose(&model, q, fwd3, jac_t);
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			assert(fabs(jac[i][j] - jac_true[i][j])  < 1e-7);
			assert(fabs(jac_t[j][i] - jac_true[i][j])  < 1e-7);
			if (i < 4 && j < 4) {
				assert(fabs(fwd2[i][j] - fwd_true[i][j])  < 1e-7);
				assert(fabs(fwd3[i][j] - fwd_true[i][j])  < 1e-7);
			}
		}
	}

	struct model_lpoe model_offs;
	lpoe_init(&model_offs);
	model_offs.n_screw = 7;
	model_offs.n_joints = 6;
	model_offs.lrob = er6;
	double offs[6] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
	model_offs.q_a_offs = offs;
	double q2[] = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};
	fwd_lpoe(&model_offs, q2, 7, fwd);
	for (int i=0; i<4; i++){
		for (int j=0; j<4; j++){
			assert(fabs(fwd[6][i][j] - fwd_true[i][j])  < 1e-7);
		}
	}
	spatial_jacobian(&model_offs, q2, fwd2, jac);
	spatial_jacobian_transpose(&model_offs, q2, fwd3, jac_t);
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			assert(fabs(jac[i][j] - jac_true[i][j])  < 1e-7);
			assert(fabs(jac_t[j][i] - jac_true[i][j])  < 1e-7);
			if (i < 4 && j < 4) {
				assert(fabs(fwd2[i][j] - fwd_true[i][j])  < 1e-7);
				assert(fabs(fwd3[i][j] - fwd_true[i][j])  < 1e-7);
			}
		}
	}
}

static void test_vee(void)
{
	double that[4][4] = {
		{0, -3, 2, 4},
		{3, 0, -1, 5},
		{-2, 1, 0, 6},
		{0, 0, 0, 0},
	};
	double tvec_true[6] = {4, 5, 6, 1, 2, 3};
	double tvec[6];
	vee(that, tvec);
	for (int i = 0; i < 6; i++)
		assert(fabs(tvec[i] - tvec_true[i])  < 1e-12);
}

static void test_jac_jdot(void)
{
	struct model_lpoe_link er6[] = {
		{
			.s = {0, 0, 0, 0, 0, 0},
			.trans = {
				{1, 0, 0, 0},
				{0, 1, 0, 0},
				{0, 0, 1, 0.3165},
				{0, 0, 0, 1}},
		},
		{
			.s = {0, 0, 0, 0, 0, -1},
			.trans = {
				{1, 0, 0, 0.16},
				{0, 1, 0, -0.115},
				{0, 0, 1, 0.0955},
				{0, 0, 0, 1}},
		},
		{
			.s = {0, 0, 0, 0, 1, 0},
			.trans = {
				{1, 0, 0, 0},
				{0, 1, 0, -0.003},
				{0, 0, 1, 0.68},
				{0, 0, 0, 1}},
		},
		{
			.s = {0, 0, 0, 0, 1, 0},
			.trans = {
				{0,  0, 1, 0.121},
				{0,  1, 0, 0.118},
				{-1, 0, 0, 0.21},
				{0,  0, 0, 1}},
		},
		{
			.s = {0, 0, 0, 0, 0, -1},
			.trans = {
				{1, 0, 0, 0},
				{0, 1, 0, -0.059},
				{0, 0, 1, 0.619},
				{0, 0, 0, 1}},
		},
		{
			.s = {0, 0, 0, 0, 1, 0},
			.trans = {
				{1, 0, 0, 0},
				{0, 1, 0, 0.059},
				{0, 0, 1, 0},
				{0, 0, 0, 1}},
		},
		{
			.s = {0, 0, 0, 0, 0, -1},
			.trans = {
				{1, 0, 0, 0},
				{0, 1, 0, 0},
				{0, 0, 1, 0.089},
				{0, 0, 0, 1}},
		},
	};
	double q[] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
	double qd[] = {0.6, 0.5, 0.4, 0.3, 0.2, 0.1};
	double pose[4][4];
	double jac[6][6];
	double jdot[6][6];
	jacobian_jdot(er6, 7, 6, q, qd, pose, (double *)jac, (double *)jdot);
	double pose_true[4][4] = {
		{-0.63894042, -0.5507876 ,  0.53701783,  1.08775862},
		{-0.74204545,  0.62533077, -0.241516  , -0.12583937},
		{-0.20278976, -0.55280597, -0.80825854,  0.8360277 },
		{ 0.        ,  0.        ,  0.        ,  1.        }};
	double jac_true[6][6] = {
		{-0.12583937362445355, 0.4219093296958845, -0.24120649280135054,
			0.004002840560472503, -0.07461211426118167, -5.118309998052524e-18},
		{-1.0877586162321484, -0.04233213424482876, 0.024201374360105138,
			-0.039899583493259144, -0.023081991964610887, 1.5851267191790982e-16},
		{-2.7755575615628914e-17, -0.9348873285856748, -0.7997921836450332,
			0.014581948502210082, -0.04267615320553164, -1.6425924939571717e-17},
		{0.0, 0.09983341664682815, 0.09983341664682815,
			-0.8731983044562821, -0.09381172468505247, -0.5370178305208011},
		{-5.551115123125783e-17, 0.9950041652780262, 0.9950041652780262,
			0.0876120655431924, 0.9350981347296616, 0.2415159973302142},
		{-1.0, 2.7755575615628914e-17, 2.7755575615628914e-17,
			0.4794255386042031, -0.3417467464903276, 0.8082585432498223}};
	for (int i=0; i<4; i++){
		for (int j=0; j<4; j++){
			assert(fabs(pose[i][j] - pose_true[i][j])  < 1e-7);
		}
	}
	for (int i=0; i<6; i++){
		for (int j=0; j<6; j++){
			assert(fabs(jac[i][j] - jac_true[i][j])  < 1e-7);
		}
	}
}

static void test_jac_jdot_frida(void)
{
	struct model_lpoe_link frida[] = {
		{
			.s = {0, 0, 0, 0, 0, 0},
			.trans = {
				{1, 0, 0, 0},
				{0, 1, 0, 0},
				{0, 0, 1, 0},
				{0, 0, 0, 1}},
		},
		{
			.s = {0, 0, 0, 0, 0, 1},
			.trans = {
				{1, 0, 0, -0.03},
				{0, 1, 0, 0},
				{0, 0, 1, 0.11},
				{0, 0, 0, 1}},
		},
		{
			.s = {0, 0, 0, 0, 1, 0},
			.trans = {
				{1, 0, 0, 0.03},
				{0, 1, 0, 0},
				{0, 0, 1, 0},
				{0, 0, 0, 1}},
		},
		{
			.s = {0, 0, 0, 0, 0, 1},
			.trans = {
				{1, 0, 0, 0.0405},
				{0, 1, 0, 0},
				{0, 0, 1, 0.2465},
				{0, 0, 0, 1}},
		},
		{
			.s = {0, 0, 0, 0, 1, 0},
			.trans = {
				{ 0, 0, 1, 0},
				{ 0, 1, 0, 0},
				{-1, 0, 0, 0.0405},
				{ 0, 0, 0, 1}},
		},
		{
			.s = {0, 0, 0, 0, 0, 1},
			.trans = {
				{1, 0, 0, 0.0135},
				{0, 1, 0, 0},
				{0, 0, 1, 0.265},
				{0, 0, 0, 1}},
		},
		{
			.s = {0, 0, 0, 0, 1, 0},
			.trans = {
				{1, 0, 0, -0.027},
				{0, 1, 0, 0},
				{0, 0, 1, 0},
				{0, 0, 0, 1}},
		},
		{
			.s = {0, 0, 0, 0, 0, 1},
			.trans = {
				{1, 0, 0, 0},
				{0, 1, 0, 0},
				{0, 0, 1, 0.032},
				{0, 0, 0, 1}},
		},
	};
	double q[] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
	double qd[] = {0.7, -0.6, 0.5, -0.4, 0.3, -0.2, 0.1};
	double pose[4][4];
	double jac[6][7];
	double jdot[6][7];
	jacobian_jdot(frida, 8, 7, q, qd, pose, (double *)jac, (double *)jdot);
	double pose_true[4][4] = {
		{-0.789255725399881, 0.559019327819127, 0.254111768810812, 0.331411094284484},
		{0.607708329099950, 0.651681204832158, 0.453874645701931, 0.137184725457812},
		{0.088124835693777, 0.512649001162743, -0.854063823692812, 0.194369240091434},
		{0.000000000000000, 0.000000000000000, 0.000000000000000, 1.000000000000000}};
	double jac_true[6][7] = {
		{-0.137184725457812, 0.083947745312319, -0.132717097915755, -0.139170739892609,
			-0.000751031665529, -0.023980141084683, -0.000000000000000},
		{0.331411094284484, 0.008422869498224, 0.307532079508777, -0.040580928179303,
			0.008262550416687, 0.013693860524082, 0.000000000000000},
		{0.000000000000000, -0.373451039086621, 0.020545085155443, -0.272156767896936,
			0.004167506056653, -0.031471114017551, 0.000000000000000},
		{0.000000000000000, -0.099833416646828, 0.197676811654084, -0.383557042381481,
			0.753922124568523, -0.080890932632731, 0.254111768810812},
		{0.000000000000000, 0.995004165278026, 0.019833838076210, 0.921649085609072,
			0.349203318942545, 0.889929732403870, 0.453874645701931},
		{1.000000000000000, 0.000000000000000, 0.980066577841242, 0.058710801693827,
			-0.556469650677910, 0.448867161197367, -0.854063823692812}};
	double jdot_true[6][7] = {
		{-0.396672448346503, 0.348998057042646, -0.382216891489232, 0.238348527923708,
			0.266521381064006, 0.018858759974192, 0.026715036660381},
		{-0.152517489291415, 0.035016605611833, -0.218853461659875, 0.107648015748958,
			-0.219228151494729, 0.007349196947761, -0.016904598481490},
		{0.000000000000000, 0.112154371314611, 0.081521115399008, -0.132742845122481,
			0.223518331315263, -0.011172049603981, -0.001035008628609},
		{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
		{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
		{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

	for (int i=0; i<4; i++){
		for (int j=0; j<4; j++){
			assert(fabs(pose[i][j] - pose_true[i][j])  < 1e-7);
		}
	}
	for (int i=0; i<6; i++){
		for (int j=0; j<7; j++){
			assert(fabs(jac[i][j] - jac_true[i][j])  < 1e-7);
		}
	}
	for (int i=0; i<6; i++){
		for (int j=0; j<7; j++){
			assert(fabs(jdot[i][j] - jdot_true[i][j])  < 1e-7);
		}
	}
}

static void test_fwd_par_bar(void)
{
	struct model_lpoe_link irb2400[] = {
		{
			.s = {0, 0, 0, 0, 0, 0},
			.trans = {
				{1, 0, 0, 0},
				{0, 1, 0, 0},
				{0, 0, 1, 0.18},
				{0, 0, 0, 1}},
		},
		{
			.s = {0, 0, 0, 0, 0, 1},
			.trans = {
				{1, 0, 0, 0.1},
				{0, 1, 0, 0},
				{0, 0, 1, 0.435},
				{0, 0, 0, 1}},
		},
		{
			.s = {0, 0, 0, 0, 1, 0},
			.trans = {
				{1, 0, 0, 0},
				{0, 1, 0, 0},
				{0, 0, 1, 0.705},
				{0, 0, 0, 1}},
		},
		{
			.s = {0, 0, 0, 0, 1, 0},
			.trans = {
				{0,  0, 1, 0.258},
				{0,  1, 0, 0},
				{-1, 0, 0, 0.135},
				{0,  0, 0, 1}},
		},
		{
			.s = {0, 0, 0, 0, 0, 1},
			.trans = {
				{1, 0, 0, 0},
				{0, 1, 0, 0},
				{0, 0, 1, 0.497},
				{0, 0, 0, 1}},
		},
		{
			.s = {0, 0, 0, 0, 1, 0},
			.trans = {
				{1, 0, 0, 0},
				{0, 1, 0, 0},
				{0, 0, 1, 0},
				{0, 0, 0, 1}},
		},
		{
			.s = {0, 0, 0, 0, 0, 1},
			.trans = {
				{1, 0, 0, 0},
				{0, 1, 0, 0},
				{0, 0, 1, 0.085},
				{0, 0, 0, 1}},
		},
	};
	struct model_lpoe model;
	lpoe_init(&model);
	model.n_screw = 7;
	model.n_joints = 6;
	model.lrob = irb2400;
	model.has_pbar = 1;
	model.par_fact = 1.0;
	double q[] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
	double fwd_true[4][4] = {
		{-0.58771357,  0.42940282,  0.68571567,  1.05451998},
		{ 0.74718528,  0.61315167,  0.25643555,  0.12175385},
		{-0.31033356,  0.6630673 , -0.68120102,  1.15389752},
		{ 0.        ,  0.        ,  0.        ,  1.        },};
	double fwd[7][4][4];
	fwd_lpoe(&model, q, 7, fwd);
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			assert(fabs(fwd[6][i][j] - fwd_true[i][j])  < 1e-7);
		}
	}
	double jac_true[6][6] = {
		{0.0, 0.68749508, -1.29942264, -0.14511405, -1.08730108, -0.37883919},
		{0.0, 0.06897959, -0.13037714,  1.44630014, -0.34329436,  1.5095857},
		{0.0, -1.40061878e-01,  2.40061878e-01,  3.27708185e-18,  9.22199569e-01, 1.86927885e-01},
		{0.0, 0.0, -0.09983342,  0.95056379,  0.0225534 ,  0.68571567},
		{0.0, 0.0, 0.99500417, 0.09537451, 0.92794845, 0.25643555},
		{1.0, 0.0, 0.0, -0.29552021,  0.37202555, -0.68120102},};
	double jac[6][6];
	double fwd2[4][4];
	spatial_jacobian(&model, q, fwd2, jac);
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			assert(fabs(jac[i][j] - jac_true[i][j])  < 1e-7);
			if (i < 4 && j < 4)
				assert(fabs(fwd2[i][j] - fwd_true[i][j])  < 1e-7);
		}
	}

	/* Test also with coupling defined as a coupling matrix */
	double coupling_mat[] = {
		1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
		0.0, -1.0, 1.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
	struct model_lpoe model2;
	lpoe_init(&model2);
	model2.n_screw = 7;
	model2.n_joints = 6;
	model2.lrob = irb2400;
	model2.coupling_matrix = coupling_mat;
	fwd_lpoe(&model2, q, 7, fwd);
	for (int i=0; i<4; i++){
		for (int j=0; j<4; j++){
			assert(fabs(fwd[6][i][j] - fwd_true[i][j])  < 1e-7);
		}
	}
	spatial_jacobian(&model2, q, fwd2, jac);
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			assert(fabs(jac[i][j] - jac_true[i][j])  < 1e-7);
			if (i < 4 && j < 4)
				assert(fabs(fwd2[i][j] - fwd_true[i][j])  < 1e-7);
		}
	}
}

static void test_transform_jacobian(void)
{
	double jac[2][6] = {
		{1.0, 2.0, 3.0, 4.0, 5.0, 6.0},
		{1.0, -1.0, 1.0, -1.0, 1.0, -1.0}};
	double fk[4][4] = {
		{0.0, 0.0, 1.0, 1.0},
		{0.0, 1.0, 0.0, 0.5},
		{-1.0, 0.0, 0.0, 1.4},
		{0.0, 0.0, 0.0, 1.0}};
	double tb[4][4] = {
		{0.0, 0.0, 1.0, 0.2},
		{-1.0, 0.0, 0.0, 0.23},
		{0.0, -1.0, 0.0, -0.31},
		{0.0, 0.0, 0.0, 1.0}};
	double ta[4][4] = {
		{0.0, 1.0, 0.0, 0.11},
		{-1.0, 0.0, 0.0, 0.21},
		{0.0, 0.0, 1.0, 0.33},
		{0.0, 0.0, 0.0, 1.0}};
	double fk_tot[4][4] = {
		{0.0, -1.0, 0.0, 1.49},
		{0.0, 0.0, -1.0, -1.1},
		{1.0, 0.0, 0.0, -1.02},
		{0.0, 0.0, 0.0, 1.0}};
	double jac_tot[2][6] = {
		{2.19, 0.81, -4.42, 6.0, -4.0, -5.0},
		{0.46, -1.1, 1.44, -1.0, 1.0, -1.0}};
	double t[4][4];
	double j_t[2][6];

	printf("\nTest transform_jacobian:\n");
	transform_jacobian(jac, 2, fk, tb, ta, t, j_t);

	printf("jac:\n");
	for (int i1 = 0; i1 < 2; i1++) {
		for (int i2 = 0; i2 < 6; i2++) {
			printf("%f ", j_t[i1][i2]);
			assert(fabs(j_t[i1][i2] - jac_tot[i1][i2]) < 1e-10);
		}
		printf("\n");
	}

	printf("fk:\n");
	for (int i1 = 0; i1 < 4; i1++) {
		for (int i2 = 0; i2 < 4; i2++) {
			printf("%f ", t[i1][i2]);
			assert(fabs(t[i1][i2] - fk_tot[i1][i2]) < 1e-10);
		}
		printf("\n");
	}
}

static void test_spatial_to_normal_jacobian(void)
{
	double jac[6][6] = {
		{0.0, 0.0, 0.0, 0.0, 0.0, -1.0},
		{-0.40994171609454666, 0.041131367658493206,
			0.16000000000000003, 0.09983341664682815,
			0.9950041652780258, 0.0},
		{-1.0730575385917815, 0.10766487626342709,
			0.29509514494064165, 0.09983341664682815,
			0.9950041652780258, 0.0},
		{-0.1295738827417061, -1.2914168158276715,
			-2.713317343982112e-17, -0.8731983044562818,
			0.08761206554319242, 0.47942553860420295},
		{-0.8133748624790246, 0.270226775489256,
			0.9626796911962711, -0.09381172468505243,
			0.9350981347296613, -0.3417467464903276},
		{-0.3036248130564498, -1.328151977353377,
			0.1951331196359403, -0.5370178305208008,
			0.24151599733021403, 0.808258543249822}};
	double fk[4][4] = {
		{-0.6389404236444644, -0.5507876040142562,
			0.5370178305208008, 1.0877586162321478},
		{-0.742045449856935, 0.625330771176512,
			-0.24151599733021403, -0.1258393736244534},
		{-0.2027897565944876, -0.5528059712810851,
			-0.808258543249822, 0.8360277020126783},
		{0.0, 0.0, 0.0, 1.0}};
	double jac_normal[6][6];
	double jac_normal_true[6][6] = {
		{-0.1258393736244534, -1.0877586162321478,
			0.0, 0.0, 0.0, -1.0},
		{0.4219093296958844, -0.0423321342448288,
			-0.9348873285856745, 0.09983341664682815,
			0.9950041652780258, 0.0},
		{-0.24120649280135048, 0.02420137436010509,
			-0.7997921836450329, 0.09983341664682815,
			0.9950041652780258, 0.0},
		{0.004002840560472335, -0.039899583493259304,
			0.014581948502210009, -0.8731983044562818,
			0.08761206554319242, 0.47942553860420295},
		{-0.07461211426118171, -0.023081991964610946,
			-0.04267615320553106, -0.09381172468505243,
			0.9350981347296613, -0.3417467464903276},
		{-1.1031056355814238e-17, -1.127392097465046e-17,
			5.606564051740301e-17, -0.5370178305208008,
			0.24151599733021403, 0.808258543249822}};

	printf("\nTest of spatial_to_normal_jacobian:\n");
	spatial_to_normal_jacobian(jac, 6, fk, jac_normal);
	printf("Normal jacobian:\n");
	for (int i1 = 0; i1 < 6; i1++) {
		for (int i2 = 0; i2 < 6; i2++) {
			printf("%f ", jac_normal[i1][i2]);
			assert(fabs(jac_normal[i1][i2] - jac_normal_true[i1][i2]) < 1e-10);
		}
		printf("\n");
	}
}

int main(void)
{
	test_twist_ops();
	test_twist_omega_hat();
	test_twist_R_1();
	test_twist_R_2();
	test_twist_exp_hat();
	test_inv_t44();
	test_rot_z();
	test_rot_y();
	test_rot_x();
	test_fwd_and_jac();
	test_vee();
	test_jac_jdot();
	test_jac_jdot_frida();
	test_fwd_par_bar();
	test_transform_jacobian();
	test_spatial_to_normal_jacobian();
}
