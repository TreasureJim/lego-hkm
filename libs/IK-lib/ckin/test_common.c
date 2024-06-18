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

#include "common.h"


static void test_tf2euler_zyz_intrinsic(void)
{
	double t44_1[4][4] = {
		{-1.0, 0.0, 0.0, 0.0},
		{0.0, 0.0, 1.0, 0.0},
		{0.0, 1.0, 0.0, 0.0},
		{0.0, 0.0, 0.0, 1.0},
	};
	double t44_2[4][4] = {
		{0.0, 1.0, 0.0, 0.0},
		{1.0, 0.0, 0.0, 0.0},
		{0.0, 0.0, -1.0, 0.0},
		{0.0, 0.0, 0.0, 1.0},
	};
	double euler_angles_true1[3] = {M_PI/2, M_PI/2, M_PI/2};
	double euler_angles_true2[3] = {0, M_PI, M_PI/2};
	double euler_angles[3];
	tf2euler_zyz_intrinsic(t44_1, euler_angles);
	for (int i = 0; i < 3; i++)
		assert(fabs(euler_angles[i] - euler_angles_true1[i])  < 1e-12);
	tf2euler_zyz_intrinsic(t44_2, euler_angles);
	for (int i = 0; i < 3; i++)
		assert(fabs(euler_angles[i] - euler_angles_true2[i])  < 1e-12);
}

static void test_tf2euler_zyx_intrinsic(void)
{
	double t44_1[4][4] = {
		{0.0, 0.0, 1.0, 0.0},
		{0.0, 1.0, 0.0, 0.0},
		{-1.0, 0.0, 0.0, 0.0},
		{0.0, 0.0, 0.0, 1.0}};
	double t44_2[4][4] = {
		{-1.0, 0.0, 0.0, 0.0},
		{0.0, 0.0, -1.0, 0.0},
		{0.0, -1.0, 0.0, 0.0},
		{0.0, 0.0, 0.0, 1.0}};
	double euler_angles_true1[3] = {0, M_PI/2, 0};
	double euler_angles_true2[3] = {M_PI, 0, -M_PI/2};
	double euler_angles[3];
	tf2euler_zyx_intrinsic(t44_1, euler_angles);
	for (int i = 0; i < 3; i++) {
		assert(fabs(euler_angles[i] - euler_angles_true1[i])  < 1e-12);
	}
	tf2euler_zyx_intrinsic(t44_2, euler_angles);
	for (int i = 0; i < 3; i++) {
		assert(fabs(euler_angles[i] - euler_angles_true2[i])  < 1e-12);
	}
}

static void test_euler_zyz_intrinsic2tf(void)
{
	double euler_angles1[3] = {M_PI/2, M_PI/2, M_PI/2};
	double euler_angles2[3] = {0, M_PI, M_PI/2};
	double t44_1_true[4][4] = {
		{-1.0, 0.0, 0.0, 0.0},
		{0.0, 0.0, 1.0, 0.0},
		{0.0, 1.0, 0.0, 0.0},
		{0.0, 0.0, 0.0, 1.0}};
	double t44_2_true[4][4] = {
		{0.0, 1.0, 0.0, 0.0},
		{1.0, 0.0, 0.0, 0.0},
		{0.0, 0.0, -1.0, 0.0},
		{0.0, 0.0, 0.0, 1.0}};
	double t44[4][4];
	euler_zyz_intrinsic2tf(euler_angles1, t44);
	for (int i1 = 0; i1 < 4; i1++) {
		for (int i2 = 0; i2 < 4; i2++) {
			assert(fabs(t44[i1][i2] - t44_1_true[i1][i2])  < 1e-12);
		}
	}
	euler_zyz_intrinsic2tf(euler_angles2, t44);
	for (int i1 = 0; i1 < 4; i1++) {
		for (int i2 = 0; i2 < 4; i2++) {
			assert(fabs(t44[i1][i2] - t44_2_true[i1][i2])  < 1e-12);
		}
	}
}

static void test_euler_zyx_intrinsic2tf(void)
{
	double euler_angles1[3] = {M_PI/2, M_PI/2, M_PI/2};
	double euler_angles2[3] = {0, M_PI, M_PI/2};
	double t44_1_true[4][4] = {
		{0.0, 0.0, 1.0, 0.0},
		{0.0, 1.0, 0.0, 0.0},
		{-1.0, 0.0, 0.0, 0.0},
		{0.0, 0.0, 0.0, 1.0}};
	double t44_2_true[4][4] = {
		{-1.0, 0.0, 0.0, 0.0},
		{0.0, 0.0, -1.0, 0.0},
		{0.0, -1.0, 0.0, 0.0},
		{0.0, 0.0, 0.0, 1.0}};
	double t44[4][4];
	euler_zyx_intrinsic2tf(euler_angles1, t44);
	for (int i1 = 0; i1 < 4; i1++) {
		for (int i2 = 0; i2 < 4; i2++) {
			assert(fabs(t44[i1][i2] - t44_1_true[i1][i2])  < 1e-12);
		}
	}
	euler_zyx_intrinsic2tf(euler_angles2, t44);
	for (int i1 = 0; i1 < 4; i1++) {
		for (int i2 = 0; i2 < 4; i2++) {
			assert(fabs(t44[i1][i2] - t44_2_true[i1][i2])  < 1e-12);
		}
	}
}

static void test_tf2quat(void)
{
	double t44_1[4][4] = {
		{0.0, 0.0, 1.0, 0.0},
		{0.0, 1.0, 0.0, 0.0},
		{-1.0, 0.0, 0.0, 0.0},
		{0.0, 0.0, 0.0, 1.0}};
	double t44_2[4][4] = {
		{-1.0, 0.0, 0.0, 0.0},
		{0.0, 0.0, -1.0, 0.0},
		{0.0, -1.0, 0.0, 0.0},
		{0.0, 0.0, 0.0, 1.0}};
	double t44_3[4][4] = {
		{-0.0023357426377654316, -0.999997004497775, -0.0007316432240309645, 0.0},
		{-0.2608210910685256, 0.0013155331984200285, -0.9653862583578802, 0.0},
		{0.9653843290421458, -0.0020640658615946853, -0.26082338252534987, 0.0},
		{0.0, 0.0, 0.0, 1.0}};
	double q1_true[4] = {1.0/sqrt(2), 0.0, 1.0/sqrt(2), 0.0};
	double q2_true[4] = {0.0, 0.0, -1.0/sqrt(2), 1.0/sqrt(2)};
	double q3_true[4] = {-0.4295801461995493, -0.5606184323336962, 0.5622443104117496, -0.43017346120896277};
	double q[4];
	tf2quat(t44_1, q);
	for (int i1 = 0; i1 < 4; i1++) {
		assert(fabs(q[i1] - q1_true[i1])  < 1e-12);
	}
	tf2quat(t44_2, q);
	for (int i1 = 0; i1 < 4; i1++) {
		assert(fabs(q[i1] - q2_true[i1])  < 1e-12);
	}
	tf2quat(t44_3, q);
	for (int i1 = 0; i1 < 4; i1++) {
		assert(fabs(q[i1] - q3_true[i1])  < 1e-12);
	}
}

static void test_quat2tf(void)
{
	double q1[4] = {1.0/sqrt(2), 0.0, 1.0/sqrt(2), 0.0};
	double q2[4] = {0.0, 0.0, -1.0/sqrt(2), 1.0/sqrt(2)};
	double t44_1_true[4][4] = {
		{0.0, 0.0, 1.0, 0.0},
		{0.0, 1.0, 0.0, 0.0},
		{-1.0, 0.0, 0.0, 0.0},
		{0.0, 0.0, 0.0, 1.0}};
	double t44_2_true[4][4] = {
		{-1.0, 0.0, 0.0, 0.0},
		{0.0, 0.0, -1.0, 0.0},
		{0.0, -1.0, 0.0, 0.0},
		{0.0, 0.0, 0.0, 1.0}};
	double t44[4][4];

	quat2tf(q1, t44);
	for (int i1 = 0; i1 < 4; i1++) {
		for (int i2 = 0; i2 < 4; i2++) {
			assert(fabs(t44[i1][i2] - t44_1_true[i1][i2])  < 1e-12);
		}
	}

	quat2tf(q2, t44);
	for (int i1 = 0; i1 < 4; i1++) {
		for (int i2 = 0; i2 < 4; i2++) {
			assert(fabs(t44[i1][i2] - t44_2_true[i1][i2])  < 1e-12);
		}
	}
}

static void test_quat_mul(void)
{
	double q1[4] = {1.0/sqrt(2), 0.0, 1.0/sqrt(2), 0.0};
	double q2[4] = {0.0, 0.0, -1.0/sqrt(2), 1.0/sqrt(2)};
	double q12_true[4] = {0.5, 0.5, -0.5, 0.5};
	double q3[4] = {1.0/sqrt(3), 0.0, 1.0/sqrt(3), -1.0/sqrt(3)};
	double q4[4] = {1.0/sqrt(30), 2.0/sqrt(30), 3.0/sqrt(30), 4.0/sqrt(30)};
	double q34_true[4] = {0.21081851067789195, 0.9486832980505139,
			0.21081851067789192, 0.10540925533894585};
	double qmul[4];

	quat_mul(q1, q2, qmul);
	for (int i1 = 0; i1 < 4; i1++) {
		assert(fabs(qmul[i1] - q12_true[i1])  < 1e-12);
	}

	quat_mul(q3, q4, qmul);
	for (int i1 = 0; i1 < 4; i1++) {
		assert(fabs(qmul[i1] - q34_true[i1])  < 1e-12);
	}
}

static void test_quat_inv(void)
{
	double q1[4] = {1.0/sqrt(2), 0.0, 1.0/sqrt(2), 0.0};
	double q2[4] = {0.0, 0.0, -1.0/sqrt(2), 1.0/sqrt(2)};
	double q3[4] = {1.0/sqrt(3), 0.0, 1.0/sqrt(3), -1.0/sqrt(3)};
	double q4[4] = {1.0/sqrt(30), 2.0/sqrt(30), 3.0/sqrt(30), 4.0/sqrt(30)};
	double q0[4] = {1.0, 0.0, 0.0, 0.0};
	double q_inv[4];
	double q_tmp[4];

	quat_inv(q1, q_inv);
	quat_mul(q1, q_inv, q_tmp);
	for (int i1 = 0; i1 < 4; i1++) {
		assert(fabs(q_tmp[i1] - q0[i1])  < 1e-12);
	}
	quat_mul(q_inv, q1, q_tmp);
	for (int i1 = 0; i1 < 4; i1++) {
		assert(fabs(q_tmp[i1] - q0[i1])  < 1e-12);
	}

	quat_inv(q2, q_inv);
	quat_mul(q2, q_inv, q_tmp);
	for (int i1 = 0; i1 < 4; i1++) {
		assert(fabs(q_tmp[i1] - q0[i1])  < 1e-12);
	}
	quat_mul(q_inv, q2, q_tmp);
	for (int i1 = 0; i1 < 4; i1++) {
		assert(fabs(q_tmp[i1] - q0[i1])  < 1e-12);
	}

	quat_inv(q3, q_inv);
	quat_mul(q3, q_inv, q_tmp);
	for (int i1 = 0; i1 < 4; i1++) {
		assert(fabs(q_tmp[i1] - q0[i1])  < 1e-12);
	}
	quat_mul(q_inv, q3, q_tmp);
	for (int i1 = 0; i1 < 4; i1++) {
		assert(fabs(q_tmp[i1] - q0[i1])  < 1e-12);
	}

	quat_inv(q4, q_inv);
	quat_mul(q4, q_inv, q_tmp);
	for (int i1 = 0; i1 < 4; i1++) {
		assert(fabs(q_tmp[i1] - q0[i1])  < 1e-12);
	}
	quat_mul(q_inv, q4, q_tmp);
	for (int i1 = 0; i1 < 4; i1++) {
		assert(fabs(q_tmp[i1] - q0[i1])  < 1e-12);
	}
}

static void test_inv_j2j3(void)
{
	int res;
	double wcp_arm[4][4] = {
		{1.0, 0.0, 0.0, 0.86},
		{0.0, 1.0, 0.0, 0.0},
		{0.0, 0.0, 1.0, 0.93},
		{0.0, 0.0, 0.0, 1.0},
	};
	double th23[2];

	double th23_true[] = {-1.5707963267948966, 0.0};
	double l2 = sqrt(pow(0.15, 2) + pow(0.86, 2));
	double th3_0 = atan2(-0.86, 0.15);
	double l1 = 0.78;
	printf("\nelbow up\n");
	res = inv_pos_j2j3(wcp_arm, l1, l2, 0, th23);
	assert(res == KIN_OK);
	th23[1] += th3_0;
	for (int i = 0; i < 2; i++) {
		printf("th23[%d] = %f\n", i, th23[i]);
		assert(fabs(th23[i] - th23_true[i])  < 1e-12);
	}

	double th23_true_ed[] = {-0.078172457272897122, -2.7962296703537821};
	printf("\nelbow down\n");
	res = inv_pos_j2j3(wcp_arm, l1, l2, 1, th23);
	assert(res == KIN_OK);
	th23[1] += th3_0;
	for (int i = 0; i < 2; i++) {
		printf("th23[%d] = %f\n", i, th23[i]);
		assert(fabs(th23[i] - th23_true_ed[i])  < 1e-12);
	}
}

static void test_log_trans(void)
{
	double t44[4][4] = {
		{0.591379827438346, -0.663135699679011, -0.458825613398184, -0.145701535260748},
		{0.458825613398184, 0.744612392148966, -0.484800414550126, 3.342973153728757},
		{0.663135699679011, 0.076180241988472, 0.744612392148966, 5.915823921359132},
		{0.0, 0.0, 0.0, 1.0},
	};
	double log_t44_true[4][4] = {
		{0.0, -2.0/3.0, -2.0/3.0, 3.0},
		{2.0/3.0, 0.0, -1.0/3.0, 4.0},
		{2.0/3.0, 1.0/3.0, 0.0, 5.0},
		{0.0, 0.0, 0.0, 0.0},
	};
	double log_t44[4][4];
	log_trans(t44, log_t44);

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			assert(fabs(log_t44[i][j] - log_t44_true[i][j])  < 1e-14);
		}
	}
}

static void test_vee_log_diff(void)
{
	double t44_1[4][4] = {
		{-0.074824496337654, -0.090575176194319, 0.993074736465078, 1.0},
		{0.997136412721925, 0.004156217450735, 0.075509603896154, 2.0},
		{-0.010966730226838, 0.995880948363744, 0.090004819394638, 3.0},
		{0.0, 0.0, 0.0, 1.0},
	};
	double t44_2[4][4] = {
		{-0.084970300554725, -0.079947785906341, 0.993170881345357, 1.01},
		{0.996382985374960, -0.005820212595259, 0.084776598071799, 2.01},
		{-0.000997235639662, 0.996782060760535, 0.080153158808990, 3.01},
		{0.0, 0.0, 0.0, 1.0},
	};
	double res_true[6] = {-0.0017470204457595, 0.0273226739711143, 0.0023706530727427,
			0.0091473547734040, 0.0106578078062520, 0.0101145703639450};
	double res[6];
	vee_log_diff(t44_1, t44_2, res);

	for (int i = 0; i < 6; i++) {
		assert(fabs(res[i] - res_true[i])  < 1e-14);
	}
}

static void test_pos_diff(void)
{
	double t44_1[4][4] = {
		{-0.074824496337654, -0.090575176194319, 0.993074736465078, 1.0},
		{0.997136412721925, 0.004156217450735, 0.075509603896154, 2.0},
		{-0.010966730226838, 0.995880948363744, 0.090004819394638, 3.0},
		{0.0, 0.0, 0.0, 1.0},
	};
	double t44_2[4][4] = {
		{-0.084970300554725, -0.079947785906341, 0.993170881345357, 1.01},
		{0.996382985374960, -0.005820212595259, 0.084776598071799, 2.01},
		{-0.000997235639662, 0.996782060760535, 0.080153158808990, 3.01},
		{0.0, 0.0, 0.0, 1.0},
	};
	double res_true = sqrt(3*pow(0.01, 2));
	double res = pos_diff(t44_1, t44_2);
	assert(fabs(res - res_true)  < 1e-14);
}

static void test_rot_diff(void)
{
	double t44_1[4][4] = {
		{-0.074824496337654, -0.090575176194319, 0.993074736465078, 1.0},
		{0.997136412721925, 0.004156217450735, 0.075509603896154, 2.0},
		{-0.010966730226838, 0.995880948363744, 0.090004819394638, 3.0},
		{0.0, 0.0, 0.0, 1.0},
	};
	double t44_2[4][4] = {
		{-0.084970300554725, -0.079947785906341, 0.993170881345357, 1.01},
		{0.996382985374960, -0.005820212595259, 0.084776598071799, 2.01},
		{-0.000997235639662, 0.996782060760535, 0.080153158808990, 3.01},
		{0.0, 0.0, 0.0, 1.0},
	};
	double res_true = 0.0173080183797;
	double res = rot_diff(t44_1, t44_2);
	assert(fabs(res - res_true)  < 1e-12);
}

static void test_logR(void)
{
	double t44_1[4][4] = {
		{0.9998220336356429, 0.01609695767499943, 0.009838140591428221, 0},
		{-0.01589699546785658, 0.9996720619802858, -0.02007620559714214, 0},
		{-0.01015808012285678, 0.01991623583142786, 0.9997500472410715, 0},
		{0, 0, 0, 1},
	};
	double res1[3][3];
	double true_res1[3] = {0.015998992114291774, 0.009999370071432355,
		               -0.01999874014286471};
	double t44_2[4][4] = {
		{-1, 0, 0, 0},
		{0, 0, 1, 0},
		{0, 1, 0, 0},
		{0, 0, 0, 1},
	};
	double res2[3][3];
	double true_res2[3] = {-M_PI/sqrt(2), M_PI/sqrt(2), 0};
	double t44_3[4][4] = {
		{-1, 0, 0, 0},
		{0, -1, 0, 0},
		{0, 0, 1, 0},
		{0, 0, 0, 1},
	};
	double res3[3][3];
	double true_res3[3] = {-M_PI, 0, 0};

	logR(t44_1, res1);
	assert(fabs(res1[0][1] - true_res1[0]) < 1e-12);
	assert(fabs(res1[1][0] + true_res1[0]) < 1e-12);
	assert(fabs(res1[0][2] - true_res1[1]) < 1e-12);
	assert(fabs(res1[2][0] + true_res1[1]) < 1e-12);
	assert(fabs(res1[1][2] - true_res1[2]) < 1e-12);
	assert(fabs(res1[2][1] + true_res1[2]) < 1e-12);
	assert(fabs(res1[0][0]) < 1e-12);
	assert(fabs(res1[1][1]) < 1e-12);
	assert(fabs(res1[2][2]) < 1e-12);

	logR(t44_2, res2);
	assert(fabs(res2[0][1] - true_res2[0]) < 1e-12);
	assert(fabs(res2[1][0] + true_res2[0]) < 1e-12);
	assert(fabs(res2[0][2] - true_res2[1]) < 1e-12);
	assert(fabs(res2[2][0] + true_res2[1]) < 1e-12);
	assert(fabs(res2[1][2] - true_res2[2]) < 1e-12);
	assert(fabs(res2[2][1] + true_res2[2]) < 1e-12);
	assert(fabs(res2[0][0]) < 1e-12);
	assert(fabs(res2[1][1]) < 1e-12);
	assert(fabs(res2[2][2]) < 1e-12);

	logR(t44_3, res3);
	assert(fabs(res3[0][1] - true_res3[0]) < 1e-12);
	assert(fabs(res3[1][0] + true_res3[0]) < 1e-12);
	assert(fabs(res3[0][2] - true_res3[1]) < 1e-12);
	assert(fabs(res3[2][0] + true_res3[1]) < 1e-12);
	assert(fabs(res3[1][2] - true_res3[2]) < 1e-12);
	assert(fabs(res3[2][1] + true_res3[2]) < 1e-12);
	assert(fabs(res3[0][0]) < 1e-12);
	assert(fabs(res3[1][1]) < 1e-12);
	assert(fabs(res3[2][2]) < 1e-12);
}

static void test_tr2diff(void)
{
	double t44_11[4][4] = {
		{1, 0, 0, -0.01},
		{0, 1, 0, -42.0},
		{0, 0, 1, 0.0003},
		{0, 0, 0, 1},
	};
	double t44_12[4][4] = {
		{0.9998220336356429, 0.01609695767499943, 0.009838140591428221, 0},
		{-0.01589699546785658, 0.9996720619802858, -0.02007620559714214, 0},
		{-0.01015808012285678, 0.01991623583142786, 0.9997500472410715, 0},
		{0, 0, 0, 1},
	};
	double res1[6];
	double true_res1[6] = {0.01, 42, -0.0003, 0.01999874014286471,
		               0.009999370071432355, -0.015998992114291774};
	double t44_21[4][4] = {
		{1, 0, 0, 0},
		{0, 1, 0, 0.004},
		{0, 0, 1, 0},
		{0, 0, 0, 1},
	};
	double t44_22[4][4] = {
		{-1, 0, 0, 0},
		{0, 0, 1, 0.003},
		{0, 1, 0, 0.0021},
		{0, 0, 0, 1},
	};
	double res2[6];
	double true_res2[6] = {0, -0.001, 0.0021, 0, M_PI/sqrt(2), M_PI/sqrt(2)};
	double t44_31[4][4] = {
		{0.12265444289931626, 0.9347972744070965, 0.33333158175924754, 0.9351208617290369},
		{-0.18733618935127194, 0.35163715681715507, -0.9172003391325644, -1.076895360241332},
		{-0.974608146793573, 0.050053628306159004, 0.21825121877548828, 0.3364637879245913},
		{0, 0, 0, 1},
	};
	double t44_32[4][4] = {
		{0.12234464643009106, 0.9349813738185523, 0.3329288484079867, 0.9354732566051458},
		{-0.1862671127934852, 0.35111905473316096, -0.9176164624149189, -1.076544122072603},
		{-0.9748519632136335, 0.05025176629396869, 0.21711381762306442, 0.3365485173813669},
		{0, 0, 0, 1},
	};
	double res3[6];
	double true_res3[6] = {0.0003523949, 0.0003512382, 0.0000847295, 0.0011586283, 0.0002235331, -0.0004920301};

	tr2diff(t44_11, t44_12, res1);
	for (int i = 0; i < 6; i++) {
		assert(fabs(res1[i] - true_res1[i]) < 1e-12);
	}
	tr2diff(t44_21, t44_22, res2);
	for (int i = 0; i < 6; i++) {
		assert(fabs(res2[i] - true_res2[i]) < 1e-12);
	}

	tr2diff(t44_31, t44_32, res3);
	printf("\ntr2diff res:\n");
	for (int i = 0; i < 6; i++) {
		assert(fabs(res3[i] - true_res3[i]) < 1e-10);
	}
}

static void test_normdiff(void)
{
	double t44_11[4][4] = {
		{1, 0, 0, -0.01},
		{0, 1, 0, -42.0},
		{0, 0, 1, 0.0003},
		{0, 0, 0, 1},
	};
	double t44_12[4][4] = {
		{0.9998220336356429, 0.01609695767499943, 0.009838140591428221, 0},
		{-0.01589699546785658, 0.9996720619802858, -0.02007620559714214, 0},
		{-0.01015808012285678, 0.01991623583142786, 0.9997500472410715, 0},
		{0, 0, 0, 1},
	};
	double res1[2];
	double true_res1[2] = {42.000001191547604, 0.027493722152528852};

	normdiff(t44_11, t44_12, res1);
	assert(fabs(res1[0] - true_res1[0]) < 1e-12);
	assert(fabs(res1[1] - true_res1[1]) < 1e-12);
}

int main(void)
{
	test_tf2euler_zyz_intrinsic();
	test_tf2euler_zyx_intrinsic();
	test_euler_zyz_intrinsic2tf();
	test_euler_zyx_intrinsic2tf();
	test_tf2quat();
	test_quat2tf();
	test_quat_mul();
	test_quat_inv();
	test_inv_j2j3();
	test_log_trans();
	test_vee_log_diff();
	test_pos_diff();
	test_rot_diff();
	test_logR();
	test_tr2diff();
	test_normdiff();
}
