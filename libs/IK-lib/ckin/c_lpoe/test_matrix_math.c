/*
 * Copyright (C) 2017, 2018, 2019, 2020 Cognibotics
 *
 * Use or copying of this file or any part of it, without explicit
 * permission from Cognibotics is not allowed in any way.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "matrix_math.h"

#define assert_atol(a, b, atol) assert(fabs(a - b) < atol)
#define assert_m66_atol(a, b, atol) do {                            \
	for (int i = 0; i < 6; i++)                                 \
		for (int j = 0; j < 6; j++)                         \
			assert_atol(a[i][j], b[i][j], atol);        \
} while (0)
#define assert_m6_atol(a, b, atol) do {                             \
	for (int i = 0; i < 6; i++)                                 \
		assert_atol(a[i], b[i], atol);                      \
} while (0)

static void test_xprod(void)
{
	double a[] = {1, 2, 3};
	double b[] = {4, 5, 6};
	double out[3];

	xprod(a, b, out);
	assert(out[0] == -3);
	assert(out[1] == 6);
	assert(out[2] == -3);
}

static void test_eye(void)
{
	double m[4][4] = {0};

	eye(4, (double *) m);
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			assert(m[i][j] == ((i == j) ? 1 : 0));
}

static void test_mul66(void)
{
	/* matrices and references generated with numpy */
	double a[6][6] = {
		{0.2048271 , 0.37585118, 0.88557014, 0.13387239, 0.52130793, 0.91915053},
		{0.66730111, 0.86061015, 0.85534723, 0.73500038, 0.5485134 , 0.15241321},
		{0.91722664, 0.3806087 , 0.29756559, 0.57996824, 0.24785794, 0.52289667},
		{0.28937962, 0.13536166, 0.40871196, 0.22383887, 0.3217727 , 0.59346799},
		{0.42894688, 0.99992013, 0.15335439, 0.48253759, 0.67997502, 0.41899001},
		{0.17340557, 0.63035471, 0.03501964, 0.61830651, 0.94934014, 0.86959124}};
	double b[6][6] = {
		{0.81044217, 0.38479952, 0.67769239, 0.15614976, 0.03989231, 0.33390433},
		{0.96416923, 0.7107253 , 0.13427852, 0.94077355, 0.47599077, 0.54098635},
		{0.06808038, 0.87886661, 0.65309638, 0.62354303, 0.33127651, 0.20285762},
		{0.39607022, 0.91665316, 0.46732748, 0.06418136, 0.9249554 , 0.03654914},
		{0.65548525, 0.70455955, 0.68503566, 0.77245474, 0.05935267, 0.56745363},
		{0.6759943 , 0.36694531, 0.29495077, 0.15758561, 0.94586745, 0.43472061}};
	double c[6] = {0.47757185, 0.00272743, 0.81756913, 0.82902496, 0.98201578, 0.34215497};
	double e[6][6] = {0};

	double res[6][6] = {0};
	double res6[6] = {0};

	const double ab[6][6] = {
		{1.60474767, 1.95152735, 1.45842208, 1.49388943, 1.50460288, 1.15165234},
		{2.18249981, 2.73629876, 1.89060202, 1.9420772 , 1.57618011, 1.26628348},
		{1.87623926, 1.78311193, 1.46209949, 0.99791944, 1.36207973, 0.96169268},
		{1.09361641, 1.21642232, 0.9812905 , 0.78382386, 0.99885269, 0.70152922},
		{2.24223652, 2.08565406, 1.34000867, 1.72554807, 1.42686024, 1.30091368},
		{2.20569794, 2.10024472, 1.42080002, 1.55197564, 1.76933163, 1.34535248}};

	const double abc[6] = {5.07412481, 6.1865908 , 4.59020161, 3.1986013 , 5.44890302, 5.70516691};
	const double ab_trans[6][6] = {
		{1.25938067, 1.45485723, 1.36525938, 1.36387521, 1.66164449, 1.45133661},
		{1.63917666, 2.40491305, 2.03134797, 2.01300069, 2.31649818, 1.72007753},
		{1.36652211, 2.14130342, 1.14110667, 1.13682501, 1.83266292, 1.40061911},
		{0.80954371, 1.11489773, 0.77215263, 0.76337909, 1.09380573, 0.96346008},
		{1.07870901, 2.14912643, 1.61929251, 1.83336996, 1.74157905, 1.60346445},
		{0.83160798, 2.12390429, 1.46511245, 1.61242692, 1.60918931, 1.73227133}
	};
	const double a_trans_b[6][6] = {
		{1.38484104, 1.99031474, 1.30767643, 1.60893426, 1.08679691, 0.94482687},
		{2.29545327, 2.15067771, 1.55300988, 1.98606981, 1.33150622, 1.51466989},
		{1.8487345 , 1.70574904, 1.21572254, 1.27872417, 0.96130758, 0.93597366},
		{1.67956891, 1.85565587, 1.18572572, 1.55854915, 1.3678406 , 1.11076613},
		{2.18313064, 1.93066822, 1.48500407, 1.44748317, 1.59992667, 1.33139764},
		{2.02500497, 2.07986917, 1.80572266, 1.11173548, 1.67875535, 1.13291275}
	};

	eye(6, (double *)e);
	mul66(a, e, res);
	assert_m66_atol(res, a, 1e-16);

	mul66(e, a, res);
	assert_m66_atol(res, a, 1e-16);

	mul66_transpose(a, e, res);
	assert_m66_atol(res, a, 1e-16);

	mul66_transpose2(e, a, res);
	assert_m66_atol(res, a, 1e-16);

	mul66(a, b, res);
	assert_m66_atol(res, ab, 1e-7); /* high tol due to few printed decimals  */

	mul66_6(res, c, res6);
	assert_m6_atol(res6, abc, 1e-7);

	mul66_transpose(a, b, res);
	assert_m66_atol(res, ab_trans, 1e-7);

	mul66_transpose2(a, b, res);
	assert_m66_atol(res, a_trans_b, 1e-7);

}

static void test_mul33_3(void)
{
	double a[3][3] = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
	double b[3] = {1, 5, 9};
	double out[3];
	double out1[3] = {38, 83, 128};
	double out2[3] = {84, 99, 114};

	mul33_3(a, b, out);
	for (int i = 0; i < 3; i++)
		assert(fabs(out[i] - out1[i]) < 1e-14);

	mul33transposed_3(a, b, out);
	for (int i = 0; i < 3; i++)
		assert(fabs(out[i] - out2[i]) < 1e-14);
}

static void test_inner_prod66(void)
{
	double a[6] = {1, 2, 3, 4, 5, 6};
	double b[6] = {10, -11, 12, -13, 14, -15};
	double c;
	double c_true = -48;

	c = inner_prod66(a, b);
	assert(fabs(c-c_true) < 1e-14);
}

static void test_mul_t44_rot_3(void)
{
	double t44[4][4] = {
		{0.7140753634021542, -0.43216494552774976, 0.550753879005022, 0.1},
		{0.6196565105099437, 0.7562609655231478, -0.20998847827591904, 0.2},
		{-0.32576400102638925, 0.49122582574921003, 0.8078211458932512, 0.3},
		{0.0, 0.0, 0.0, 1.0}};
	double v[3] = {1.23, 3.45, -4.78};
	double res[3];
	double res_true[3] = {-3.2452599067300927, 4.375022765140984, -2.567345699797425};
	double res_true2[3] = {4.5732795831500965, -0.2705219990254961, -3.9084180562454844};

	mul_t44_rot_3(t44, v, res);
	for (int i = 0; i < 3; i++)
		assert(fabs(res[i] - res_true[i]) < 1e-14);

	mul_t44_rot_transposed_3(t44, v, res);
	for (int i = 0; i < 3; i++)
		assert(fabs(res[i] - res_true2[i]) < 1e-14);
}

static void test_mul_t44_rot_33(void)
{
	double t44[4][4] = {
		{0.7140753634021542, -0.43216494552774976, 0.550753879005022, 0.1},
		{0.6196565105099437, 0.7562609655231478, -0.20998847827591904, 0.2},
		{-0.32576400102638925, 0.49122582574921003, 0.8078211458932512, 0.3},
		{0.0, 0.0, 0.0, 1.0}};
	double a[3][3] = {
		{1.23, 3.45, -4.78},
		{0.43, -0.21, 3.42},
		{-4.3, -4.0, 123.456}};
	double res[3][3];
	double res_true[3][3] = {
		{-1.6757599093138773, 0.35129912627817106, 63.1025865356768},
		{1.9903201796886363, 2.8189540716031214, -26.29988319218023},
		{-3.663093543531278, -4.458327810521382, 102.96751163636566}
	};
	double res_true2[3][3] = {
		{-3.2452599067300927, 4.375022765140984, -2.567345699797425},
		{2.2813853110209292, -0.6105230989442284, 2.5195123751062374},
		{66.65200660592573, -31.613904431317213, 99.16624928881384}
	};
	double res_true3[3][3] = {
		{2.545550200917399, 3.6364881406359006, -41.5115754818322},
		{-2.318641718545782, -3.614687167827438, 65.29693648540628},
		{-2.886498701823448, -1.2870861205677353, 96.37960325004957}
	};

	mul_t44_rot_33(t44, a, res);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			assert(fabs(res[i][j] - res_true[i][j]) < 1e-12);

	mul33_t44_rot_transpose(a, t44, res);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			assert(fabs(res[i][j] - res_true2[i][j]) < 1e-12);

	mul_t44_rot_transpose_33(t44, a, res);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			assert(fabs(res[i][j] - res_true3[i][j]) < 1e-12);
}


int main(void)
{
	test_xprod();
	test_eye();
	test_mul66();
	test_mul33_3();
	test_inner_prod66();
	test_mul_t44_rot_3();
	test_mul_t44_rot_33();
}
