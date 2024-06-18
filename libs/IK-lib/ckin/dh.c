/*
 * Copyright (C) 2018, 2019 Cognibotics
 *
 * Use or copying of this file or any part of it, without explicit
 * permission from Cognibotics is not allowed in any way.
 */
#include <assert.h>
#include <math.h>
#include <string.h>

#include "lpoe.h"
#include "matrix_math.h"

#include "dh.h"
#include "common.h"

/*!
 * Compute a DH t44.
 *
 * @param[in] dhrob Definition of robot structure (in m)
 * @param[in] n Which dhmatrix (between 0 and ndof-1).
 * @param[in] theta Joint angle.
 * @param[out] out Dhmatrix.
 */
void dhmatrix(const struct model_dh dhrob, int n, double theta, double out[4][4])
{
	double cth;
	double sth;
	double ca;
	double sa;

	assert(n >= 0);
	assert(n < 6);
	memset(out, 0, 16*sizeof(double));
	cth = cos(theta);
	sth = sin(theta);
	ca = cos(dhrob.alpha[n]);
	sa = sin(dhrob.alpha[n]);
	out[0][0] = cth;
	out[1][0] = sth;
	out[0][1] = -sth*ca;
	out[1][1] = cth*ca;
	out[2][1] = sa;
	out[0][2] = sth*sa;
	out[1][2] = -cth*sa;
	out[2][2] = ca;
	out[0][3] = dhrob.a[n]*cth;
	out[1][3] = dhrob.a[n]*sth;
	out[2][3] = dhrob.d[n];
	out[3][3] = 1;
}

/*!
 * Compute a t44 x rotation followed by a y-translation,
 * i.e., t44_xrot(rx)*t44_ytrans(y).
 *
 * @param[in] rx rotation angle
 * @param[in] y translation
 * @param[out] out t44.
 */
void xrot_ytr(double rx, double y, double out[4][4])
{
	double cx;
	double sx;

	memset(out, 0, 16*sizeof(double));
	cx = cos(rx);
	sx = sin(rx);
	out[0][0] = 1;
	out[1][1] = cx;
	out[2][1] = sx;
	out[1][2] = -sx;
	out[2][2] = cx;
	out[1][3] = y*cx;
	out[2][3] = y*sx;
	out[3][3] = 1;
}

/*!
 * nominal inverse kinematics for the wcp
 *
 * ref: cb.kin.model.SixAxis._inv_pos()
 *
 * The DH-model are assumed to have alpha = [-pi/2, 0, pi/2, -pi/2, pi/2, 0]
 *
 * @param[in] dhrob Definition of robot structure (in m)
 * @param[in] wcp T44 representing the wrist center point.
 * @param[in] overhead_pos Should be 1 for overhead position (backward bending), 0 otherwise.
 * @param[in] elbow_down Should be 1 for the elbow down solution, 0 otherwise.
 * @param[out] th123 Values for joints 1, 2, and 3 + a fourth value specifying success 0 or failure -1.
 */
void inv_pos_dh(const struct model_dh dhrob, const double wcp[4][4],
		int overhead_pos, int elbow_down, double th123[4])
{
	int unreach = 0;
	double v = 0.0;

	/* Handle ER300 like offset, if any. */
	if (fabs(dhrob.d[2]) > 0.0) {
		double d_0;
		double d_offs;
		double v_offs;
		double tmp;

		d_0 = sqrt(pow(wcp[0][3], 2) + pow(wcp[1][3], 2));
		d_offs = sqrt(pow(dhrob.a[0], 2) + pow(dhrob.d[2], 2));
		v_offs = atan2(dhrob.d[2], dhrob.a[0]);
		/* sin(pi-v_offs) = sin(v_offs) */
		tmp = (d_offs/d_0)*sin(v_offs);
		if (fabs(tmp) > 1.0) {
			unreach = 1;
			tmp = 1.0;
		}
		v = asin(tmp);
	}

	if (!unreach) {
		double tf_a2[4][4];
		double wcp_arm[4][4];
		double tmp1[4][4];
		double tmp2[4][4];
		double th23[2];
		double l2;
		double th3_0;
		double l1;
		int res;

		if (overhead_pos == 0) {
			th123[0] = atan2(wcp[1][3], wcp[0][3]) - v;
		} else {
			th123[0] = atan2(-wcp[1][3], -wcp[0][3]) + v;
		}

		/* Calculate wcp in the j2j3 plane. */
		dhmatrix(dhrob, 0, th123[0], tmp1);
		xrot_ytr(-dhrob.alpha[0], dhrob.d[2], tmp2);
		mul44(tmp1, tmp2, tf_a2);
		inv_t44(tf_a2, tmp1);
		mul44(tmp1, wcp, wcp_arm);

		l2 = sqrt(pow(dhrob.a[2], 2) + pow(dhrob.d[3], 2));
		th3_0 = atan2(dhrob.d[3], dhrob.a[2]);
		l1 = dhrob.a[1];
		res = inv_pos_j2j3(wcp_arm, l1, l2, elbow_down, th23);
		unreach = (res == KIN_UNREACHABLE);

		if (!unreach) {
			th123[1] = th23[0];
			th123[2] = th23[1] + th3_0;
			th123[3] = 0;
		}
	}
	if (unreach) {
		th123[0] = 0;
		th123[1] = 0;
		th123[2] = 0;
		th123[3] = -1;
	}
}

/*!
 * nominal inverse kinematics for a DH model (on closed form)
 *
 * ref: cb.kin.model.SixAxis.inv()
 *
 * The DH-model are assumed to have alpha = [-pi/2, 0, pi/2, -pi/2, pi/2, 0]
 *
 * @param[in] dhrob Definition of robot structure (in m)
 * @param[in] tcp T44 representing the flange of the robot.
 * @param[in] overhead_pos Should be 1 for overhead position (backward bending), 0 otherwise.
 * @param[in] elbow_down Should be 1 for the elbow down solution, 0 otherwise.
 * @param[in] neg_a5 Should be 1 for a solution where joint 5 has a strictly negative angle
 * @param[out] ax Calculated joint values + a seventh value specifying success 0 or failure -1.
 */
void nom_inv_dh(const struct model_dh dhrob, const double tcp[4][4],
		int overhead_pos, int elbow_down, int neg_a5, double ax[7])
{
	double th[6] = {0};
	 /* First calculate wcp from the tcp. */
	double tf_flange_inv[4][4] = {
		{-1.0, 0.0, 0.0, 0.0},
		{0.0, 1.0, 0.0, 0.0},
		{0.0, 0.0, -1.0, 0.0},
		{0.0, 0.0, 0.0, 1.0}};
	double a6_b[4][4];
	double dh6[4][4];
	double dh6_inv[4][4];
	double wcp[4][4];
	double th123[4];

	mul44(tcp, tf_flange_inv, a6_b);
	dhmatrix(dhrob, 5, 0, dh6);
	inv_t44(dh6, dh6_inv);
	mul44(a6_b, dh6_inv, wcp);

	 /* Compute position inverse. */
	inv_pos_dh(dhrob, wcp, overhead_pos, elbow_down, th123);

	if (th123[3] == 0) {
		int i;
		double dh1[4][4];
		double dh2[4][4];
		double dh3[4][4];
		double tmp[4][4];
		double pre_a4[4][4];
		double ori[4][4];
		double th456[3];
		double sol_a5_1;

		th[0] = th123[0];
		th[1] = th123[1];
		th[2] = th123[2];

		/* Compute orientation in WCP given arm angles. */
		dhmatrix(dhrob, 0, th123[0], dh1);
		dhmatrix(dhrob, 1, th123[1], dh2);
		dhmatrix(dhrob, 2, th123[2], dh3);
		mul44(dh1, dh2, tmp);
		mul44(tmp, dh3, pre_a4);
		inv_t44(pre_a4, tmp);
		mul44(tmp, wcp, ori);

		/* Compute wrist angles. */
		tf2euler_zyz_intrinsic(ori, th456);
		sol_a5_1 = (th456[1] - dhrob.rot_offs[4])*dhrob.rot_dir[4];
		if ((neg_a5 == 1 && (sol_a5_1 < 0))
				|| (neg_a5 == 0 && (sol_a5_1 >= 0))) {
			th[3] = th456[0];
			th[4] = th456[1];
			th[5] = th456[2];
		} else {
			th[3] = th456[0] + M_PI;
			th[4] = -th456[1];
			th[5] = th456[2] + M_PI;
		}
		/* Transform theta2axes (does not handle parallel bars). */
		for (i = 0; i < 6; i++)
			ax[i] = (th[i] - dhrob.rot_offs[i])*dhrob.rot_dir[i];

		/* Make sure -pi <= ax[:] <= pi. */
		for (i = 0; i < 6; i++) {
			if (ax[i] > M_PI)
				ax[i] -= 2*M_PI;
			else if (ax[i] < -M_PI)
				ax[i] += 2*M_PI;
		}
		ax[6] = 0;
	} else {
		int i;

		for (i = 0; i < 6; i++)
			ax[i] = 0;
		ax[6] = -1;
	}
}
