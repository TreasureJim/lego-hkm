/*
 * Copyright (C) 2018, 2019 Cognibotics
 *
 * Use or copying of this file or any part of it, without explicit
 * permission from Cognibotics is not allowed in any way.
 */
#ifdef CB_SYSTEM_HEADER
#include CB_SYSTEM_HEADER
#else
#include <math.h>
#include <string.h>
#include <stdio.h>
#endif

#include "lpoe.h"
#include "matrix_math.h"

#include "lpoe_invkin.h"
#include "common.h"
#include "kin_skel.h"

/*!
 * Get only translation part from a t44
 *
 * @param[in] t44 Transformatin matrix to copy translation part from
 * @param[out] out t44.
 */
void copy_translation_part(const double t44[4][4], double out[4][4])
{
	eye(4, (double *) out);
	out[0][3] = t44[0][3];
	out[1][3] = t44[1][3];
	out[2][3] = t44[2][3];
}

/*!
 * nominal inverse kinematics for the wcp
 *
 * ref: cb.kin.model_lpoe.SixAxis._inv_pos()
 *
 * @param[in] rob Definition of nominal robot structure
 * @param[in] wcp T44 representing the wrist center point.
 * @param[in] overhead_pos Should be 1 for overhead position (backward bending), 0 otherwise.
 * @param[in] elbow_down Should be 1 for the elbow down solution, 0 otherwise.
 * @param[out] th123 Values for joints 1, 2, and 3.
 */
int inv_pos(const struct model_lpoe_nom *rob, const double wcp[4][4],
		int overhead_pos, int elbow_down, double th123[3])
{
	int unreach = 0;
	double v = 0.0;
	double y_offset = rob->wcp0[1][3];
	/* Handle ER300 like offset, if any. */
	if (fabs(y_offset) > 0.0) {
		double d_0 = sqrt(pow(wcp[0][3], 2) + pow(wcp[1][3], 2));
		if (d_0 < fabs(y_offset)) {
			unreach = 1;
		} else {
			v = asin(y_offset/d_0);
		}
	}

	if (!unreach) {
		double tf_a2_p1[4][4];
		double tf_a2_p2[4][4];
		double tf_a2_p3[4][4];
		double temp[4][4];
		double tf_a2[4][4];
		double wcp_arm[4][4];
		double th23[2];
		int res;

		if (overhead_pos == 0) {
			th123[0] = atan2(wcp[1][3], wcp[0][3]) - v;
		} else {
			th123[0] = atan2(-wcp[1][3], -wcp[0][3]) + v;
		}

		 /* Calculate wcp in the j2j3 plane. */
		copy_translation_part(rob->model.lrob[0].trans, tf_a2_p1);
		tf_a2_p1[1][3] = 0;
		rot_z(th123[0], tf_a2_p2);
		copy_translation_part(rob->model.lrob[1].trans, tf_a2_p3);
		tf_a2_p3[1][3] = y_offset;
		mul44(tf_a2_p1, tf_a2_p2, temp);
		mul44(temp, tf_a2_p3, tf_a2);

		inv_t44(tf_a2, temp);
		mul44(temp, wcp, wcp_arm);

		res = inv_pos_j2j3(wcp_arm, rob->l1, rob->l2, elbow_down, th23);
		unreach = (res == KIN_UNREACHABLE);

		if (!unreach) {
			th123[1] = th23[0] + rob->th1_0;
			th123[2] = th23[1] + rob->th2_0;
		}
	}
	if (unreach) {
		th123[0] = 0;
		th123[1] = 0;
		th123[2] = 0;
	} else {
		th123[0] *= rob->rot_dirs[0];
		th123[1] *= rob->rot_dirs[1];
		th123[2] *= rob->rot_dirs[2];
	}

	return (unreach) ? KIN_UNREACHABLE : KIN_OK;
}

/*!
 * nominal inverse kinematics for a LPOE model (on closed form)
 *
 * ref: cb.kin.model_lpoe.SixAxis.nom_inv()
 *
 * @param[in] rob Definition of nominal robot structure
 * @param[in] tcp T44 representing the flange of the robot.
 * @param[in] overhead_pos Should be 1 for overhead position (backward bending), 0 otherwise.
 * @param[in] elbow_down Should be 1 for the elbow down solution, 0 otherwise.
 * @param[in] neg_a5 Should be 1 for a solution where joint 5 has a strictly negative angle
 * @param[out] q_inv_out Calculated joint values.
 */
int nom_inv(const struct model_lpoe_nom *rob, const double tcp[4][4], int overhead_pos,
		int elbow_down, int neg_a5, double q_inv_out[6])
{
	int ret;
	double q_inv[6] = {0};
	double tmp[4][4];
	double wcp[4][4];
	double th123[3];

	/* First calculate wcp from the tcp. */
	eye(4, (double *) tmp);
	tmp[2][3] = rob->model.lrob[5].trans[2][3];
	mul44(tmp, rob->model.lrob[6].trans, wcp);
	inv_t44(wcp, tmp);
	mul44(tcp, tmp, wcp);

	/* Compute position inverse. */
	ret = inv_pos(rob, wcp, overhead_pos, elbow_down, th123);

	if (ret == KIN_OK) {
		double pre_a4[4][4];
		double temp[4][4][4];
		double ori[4][4];
		int i;
		double th456[3];

		q_inv[0] = th123[0];
		q_inv[1] = th123[1];
		q_inv[2] = th123[2];

		/* Compute orientation in WCP given arm angles.
		   Need to run fwd_lpoe without taking couplings into account,
		   therefore the call to fwd_lpoe_std */
		fwd_lpoe_std(rob->model.lrob, q_inv, 4, temp);
		memcpy(pre_a4, temp[3], 4*4*sizeof(double));
		inv_t44(pre_a4, tmp);
		mul44(tmp, wcp, ori);

		/* Compute wrist angles. */
		tf2euler_zyz_intrinsic(ori, th456);
		if ((neg_a5 && (th456[1]*rob->rot_dirs[4]) < 0)
				|| (neg_a5 == 0 && (th456[1]*rob->rot_dirs[4]) >= 0)){
			q_inv[3] = th456[0] * rob->rot_dirs[3];
			q_inv[4] = th456[1] * rob->rot_dirs[4];
			q_inv[5] = th456[2] * rob->rot_dirs[5];
		} else {
			q_inv[3] = (th456[0] + M_PI) * rob->rot_dirs[3];
			q_inv[4] = -th456[1] * rob->rot_dirs[4];
			q_inv[5] = (th456[2] + M_PI) * rob->rot_dirs[5];
		}

		/* Handle any couplings between the joints. */
		if (rob->model.coupling_matrix) {
			int j;
			for (i = 0; i < 6; i++) {
				q_inv_out[i] = 0;
				for (j = 0; j < 6; j++)
					q_inv_out[i] += rob->model.inverse_coupling_matrix[i*6+j]*q_inv[j];
			}
		} else {
			memcpy(q_inv_out, q_inv, 6 * sizeof(double));
			if (rob->model.has_pbar) {
				q_inv_out[2] += rob->model.par_fact*q_inv_out[1];
			}
		}

		/* Take offsets into account. */
		if (rob->model.q_a_offs) {
			for (i = 0; i < 6; i++)
				q_inv_out[i] -= rob->model.q_a_offs[i];
		}

		/* Make sure -pi <= q_inv[:] <= pi. */
		for (i = 0; i < 6; i++) {
			if (q_inv_out[i] > M_PI)
				q_inv_out[i] -= 2*M_PI;
			else if (q_inv_out[i] < -M_PI)
				q_inv_out[i] += 2*M_PI;
		}
	} else {
		int i;

		for (i = 0; i < 6; i++)
			q_inv_out[i] = 0;
	}

	return ret;
}

/*!
 * nominal inverse kinematics for a LPOE model with Comau Hollow Wrist
  (iterative)
 *
 * ref: cb.kin.model_lpoe.ComauHollowWrist._nom_inv()
 *
 * @param[in] rob_hw Definition of nominal robot structure (Hollow Wrist)
 * @param[in] tcp T44 representing the flange of the robot.
 * @param[in] overhead_pos Should be 1 for overhead position (backward bending), 0 otherwise.
 * @param[in] elbow_down Should be 1 for the elbow down solution, 0 otherwise.
 * @param[in] neg_a5 Should be 1 for a solution where joint 5 has a negative angle
 * @param[in] q0 Initial guess of inverse kinematics solution
 * @param[in] tol Tolerance
 * @param[in] initial_tol Tolerance for initial search for joint 6 value
 * @param[out] q_inv_out Calculated joint values.
 */
int nom_inv_hw(const struct model_lpoe_nom *rob_hw, const double tcp[4][4], int overhead_pos,
		int elbow_down, int neg_a5, double q0[6], double tol, double initial_tol,
		double q_inv_out[6])
{
	int ret;
	int retval;
	double q_inv[6] = {0};
	double q_inv_tmp[6];
	double errs_tmp[2];
	double best_norm = -1.0;
	double tmp_norm, q6;
	int start_gss = 0;
	int count = 0;
	int tv_count = 0;
	int best_ind = -1;
	int i;

	/* Max number of evaluations of nom invkin */
	int max_evals = 25;
	double tried_vals[25][3];

	/* Fixed point iteration factor */
	double fp_decrease = 0.5;
	double q6s[14] = {q0[5], -M_PI/3, M_PI/3, -2*M_PI/3, 2*M_PI/3, -0.9*M_PI, 0.9*M_PI, 0.0, -M_PI/6, M_PI/6, -M_PI/2, M_PI/2, -0.78*M_PI, 0.78*M_PI};

	if (fabs(q0[5]) < 0.01) {
		q6s[7]= M_PI/12;
	}

	for (i = 0; i < 14; i++) {
		ret = nom_inv_iteration_hw(rob_hw, tcp, q6s[i], overhead_pos, elbow_down, neg_a5, q_inv_tmp, errs_tmp);
		count++;
		if (ret == KIN_OK) {
			tmp_norm = sqrt(errs_tmp[0]*errs_tmp[0] + errs_tmp[1]*errs_tmp[1]);
			tried_vals[tv_count][0] = q6s[i];
			tried_vals[tv_count][1] = tmp_norm;
			tried_vals[tv_count][2] = q_inv_tmp[5];
			if (best_norm < 0 || tmp_norm < best_norm) {
				memcpy(q_inv, q_inv_tmp, 6*sizeof(double));
				best_norm = tmp_norm;
				best_ind = tv_count;
				if (tmp_norm < initial_tol) {
					break;
				}
			}
			tv_count += 1;
		}
	}

	if (best_norm < 0) {
		/* Could not find a valid start guess */
		memset(q_inv_out, 0, 6 * sizeof(double));
		return KIN_UNREACHABLE;
	}

	/* Try fixed point iteration. If this fails to lower error, start golden section search (gss) */
	while (best_norm > tol && count < max_evals) {
		q6 = q_inv[5];
		ret = nom_inv_iteration_hw(rob_hw, tcp, q6, overhead_pos, elbow_down, neg_a5, q_inv_tmp, errs_tmp);
		count++;
		if (ret == KIN_OK) {
			tmp_norm = sqrt(errs_tmp[0]*errs_tmp[0] + errs_tmp[1]*errs_tmp[1]);
			if (tmp_norm < best_norm) {
				if (tmp_norm > fp_decrease*best_norm) {
					start_gss = 1;
				} else {
					tv_count += 1;
					tried_vals[tv_count-1][0] = q6;
					tried_vals[tv_count-1][1] = tmp_norm;
					tried_vals[tv_count-1][2] = q_inv_tmp[5];
					best_ind = tv_count - 1;
					best_norm = tmp_norm;
					memcpy(q_inv, q_inv_tmp, 6*sizeof(double));
				}
				if (start_gss == 1) {
					break;
				}
			} else {
				start_gss = 1;
				break;
			}
		}
	}

	if (start_gss == 1) {
		double a = 100, b = 100, fa, fb, c, d, fc, fd;
		double h;

		for (i = 0; i < tv_count; i++) {
			if (i != best_ind) {
				if (tried_vals[i][0] < tried_vals[best_ind][0]) {
					if (a > 10 || tried_vals[i][0] > a) {
						a = tried_vals[i][0];
					}
				} else {
					if (b > 10 || tried_vals[i][0] < b) {
						b = tried_vals[i][0];
					}
				}
			}
		}

		/* If search for initial values is needed */
		if (a > 10) {
			int count2 = 0;
			h = 0.3;
			a = tried_vals[best_ind][0] - h;
			ret = nom_inv_iteration_hw(rob_hw, tcp, a, overhead_pos, elbow_down, neg_a5, q_inv_tmp, errs_tmp);
			count++;
			if (ret == KIN_OK) {
				tmp_norm = sqrt(errs_tmp[0]*errs_tmp[0] + errs_tmp[1]*errs_tmp[1]);
			} else {
				tmp_norm = 999999;
			}
			fa = tmp_norm;
			while (count2 < 10 && count < max_evals) {
				if (fa > best_norm) {
					break;
				} else {
					tried_vals[count-1][0] = a;
					tried_vals[count-1][1] = fa;
					fb = best_norm;
					b = tried_vals[best_ind][0];
					best_ind = count-1;
					best_norm = fa;
					memcpy(q_inv, q_inv_tmp, 6*sizeof(double));
				}
				a = fmax(-M_PI, a-h);
				ret = nom_inv_iteration_hw(rob_hw, tcp, a, overhead_pos, elbow_down, neg_a5, q_inv_tmp, errs_tmp);
				count++;
				if (ret == KIN_OK) {
					tmp_norm = sqrt(errs_tmp[0]*errs_tmp[0] + errs_tmp[1]*errs_tmp[1]);
				} else {
					tmp_norm = 999999;
				}
				fa = tmp_norm;
				count2++;
			}
		}
		if (b > 10) {
			int count3 = 0;
			h = 0.3;
			b = tried_vals[best_ind][0] + h;
			ret = nom_inv_iteration_hw(rob_hw, tcp, b, overhead_pos, elbow_down, neg_a5, q_inv_tmp, errs_tmp);
			count++;
			if (ret == KIN_OK) {
				tmp_norm = sqrt(errs_tmp[0]*errs_tmp[0] + errs_tmp[1]*errs_tmp[1]);
			} else {
				tmp_norm = 999999;
			}
			fb = tmp_norm;
			while (count3 < 10 && count < max_evals) {
				if (fb > best_norm) {
					break;
				} else {
					tried_vals[count-1][0] = b;
					tried_vals[count-1][1] = fb;
					fa = best_norm;
					a = tried_vals[best_ind][0];
					best_ind = count-1;
					best_norm = fb;
					memcpy(q_inv, q_inv_tmp, 6*sizeof(double));
				}
				b = fmin(M_PI, b+h);
				ret = nom_inv_iteration_hw(rob_hw, tcp, b, overhead_pos, elbow_down, neg_a5, q_inv_tmp, errs_tmp);
				count++;
				if (ret == KIN_OK) {
					tmp_norm = sqrt(errs_tmp[0]*errs_tmp[0] + errs_tmp[1]*errs_tmp[1]);
				} else {
					tmp_norm = 999999;
				}
				fb = tmp_norm;
				count3++;
			}
		}

		h = b - a;
		c = a + invphi2*h;
		d = a + invphi*h;

		ret = nom_inv_iteration_hw(rob_hw, tcp, c, overhead_pos, elbow_down, neg_a5, q_inv_tmp, errs_tmp);
		count++;
		if (ret == KIN_OK) {
			fc = sqrt(errs_tmp[0]*errs_tmp[0] + errs_tmp[1]*errs_tmp[1]);
			if (fc < best_norm) {
				best_norm = fc;
				memcpy(q_inv, q_inv_tmp, 6*sizeof(double));
			}
		} else {
			fc = 999999;
		}
		ret = nom_inv_iteration_hw(rob_hw, tcp, d, overhead_pos, elbow_down, neg_a5, q_inv_tmp, errs_tmp);
		count++;
		if (ret == KIN_OK) {
			fd = sqrt(errs_tmp[0]*errs_tmp[0] + errs_tmp[1]*errs_tmp[1]);
			if (fd < best_norm) {
				best_norm = fd;
				memcpy(q_inv, q_inv_tmp, 6*sizeof(double));
			}
		} else {
			fd = 999999;
		}

		while (fmin(fc, fd) > tol && count < max_evals) {
			if (fc < fd) {
				b = d;
				d = c;
				fd = fc;
				h = invphi*h;
				c = a + invphi2*h;
				ret = nom_inv_iteration_hw(rob_hw, tcp, c, overhead_pos, elbow_down, neg_a5, q_inv_tmp, errs_tmp);
				count++;
				if (ret == KIN_OK) {
					fc = sqrt(errs_tmp[0]*errs_tmp[0] + errs_tmp[1]*errs_tmp[1]);
					if (fc < best_norm) {
						best_norm = fc;
						memcpy(q_inv, q_inv_tmp, 6*sizeof(double));
					}
				} else {
					fc = 999999;
				}
			} else {
				a = c;
				c = d;
				fc = fd;
				h = invphi*h;
				d = a + invphi*h;
				ret = nom_inv_iteration_hw(rob_hw, tcp, d, overhead_pos, elbow_down, neg_a5, q_inv_tmp, errs_tmp);
				count++;
				if (ret == KIN_OK) {
					fd = sqrt(errs_tmp[0]*errs_tmp[0] + errs_tmp[1]*errs_tmp[1]);
					if (fd < best_norm) {
						best_norm = fd;
						memcpy(q_inv, q_inv_tmp, 6*sizeof(double));
					}
				} else {
					fd = 999999;
				}
			}
		}
	}

	if (best_norm < tol) {
		retval = KIN_OK;
	} else {
		retval = KIN_BAD_SOL;
	}

	/* Make sure -pi <= q_inv[:] <= pi. */
	for (i = 0; i < 6; i++) {
		if (q_inv[i] > M_PI)
			q_inv[i] -= 2*M_PI;
		else if (q_inv[i] < -M_PI)
			q_inv[i] += 2*M_PI;
		q_inv_out[i] = q_inv[i];
	}

	return retval;
}

/*!
 * nominal inverse kinematics for a LPOE model with Comau Hollow Wrist ver 2
  (iterative)
 *
 * ref: cb.kin.model_lpoe.ComauHollowWrist.inv2()
 *
 * @param[in] rob_hw Definition of nominal robot structure (Hollow Wrist)
 * @param[in] tcp T44 representing the flange of the robot.
 * @param[in] overhead_pos Should be 1 for overhead position (backward bending), 0 otherwise.
 * @param[in] elbow_down Should be 1 for the elbow down solution, 0 otherwise.
 * @param[in] neg_a5 Should be 1 for a solution where joint 5 has a negative angle
 * @param[in] q0 Initial guess of inverse kinematics solution
 * @param[in] tol Tolerance
 * @param[out] q_inv_out Calculated joint values.
 */
int nom_inv_hw2(const struct model_lpoe_nom *rob_hw, const double tcp[4][4],
		int overhead_pos, int elbow_down, int neg_a5, double q0[6],
		double tol, double q_inv_out[6])
{
	double init_tol = 0.15;
	double tol1 = 1e-3;
	double q0_1[6];
	int ret1;
	int ret;

	/* Call nom_inv_hw to get a rough initial guess */
	ret1 = nom_inv_hw(rob_hw, tcp, overhead_pos, elbow_down, neg_a5, q0, tol1, init_tol, q0_1);

	if (ret1 == KIN_UNREACHABLE) {
		return ret1;
	}

	ret = kin_skel_inv(6, &rob_hw->model, tcp, q0_1, 0, tol, 0, q_inv_out, NULL);
	return ret;
}

/*!
 * Calculation of wcp for a robot with Comau Hollow Wrist
 *
 * ref: cb.kin.model_lpoe.ComauHollowWrist.tcp2wcp()
 *
 * @param[in] rob_hw Definition of nominal robot structure (Hollow Wrist)
 * @param[in] q6 Value of joint 6
 * @param[out] wcp_out Calculated wcp
 */
void tcp2wcp_hw(const struct model_lpoe_nom *rob_hw, const double tcp[4][4],
		double q6, double wcp_out[4][4])
{
	double t1[4][4];
	double t2[4][4];
	double t3[4][4];
	double tmp1[4][4];
	double tmp2[4][4];
	double rot_angle;

	inv_t44(rob_hw->model.lrob[6].trans, t1);
	inv_t44(rob_hw->model.lrob[5].trans, t3);
	rot_angle = rob_hw->rot_dirs[5]*q6*-1;
	rot_z(rot_angle, t2);
	mul44(tcp, t1, tmp1);
	mul44(t2, t3, tmp2);
	mul44(tmp1, tmp2, wcp_out);
}

/*!
 * Single nominal inverse kinematics iteration for a LPOE model with Comau Hollow Wrist
 *
 * ref: cb.kin.model_lpoe.ComauHollowWrist._nom_inv_iteration()
 *
 * @param[in] rob_hw Definition of nominal robot structure (Hollow Wrist)
 * @param[in] tcp T44 representing the flange of the robot.
 * @param[in] q6 Value of joint 6.
 * @param[in] overhead_pos Should be 1 for overhead position (backward bending), 0 otherwise.
 * @param[in] elbow_down Should be 1 for the elbow down solution, 0 otherwise.
 * @param[in] neg_a5 Should be 1 for a solution where joint 5 is negative
 * @param[out] q_inv_out Calculated joint values.
 * @param[out] errs errors.
 */
int nom_inv_iteration_hw(const struct model_lpoe_nom *rob_hw, const double tcp[4][4],
	double q6, int overhead_pos, int elbow_down, int neg_a5, double q_inv_out[6], double errs[2])
{
	double tmp[4][4];
	double wcp[4][4];
	int ret;
	double th123[3];

	memset(q_inv_out, 0, 6 * sizeof(double));
	q_inv_out[5] = q6;

	tcp2wcp_hw(rob_hw, tcp, q6, wcp);

	 /* Compute position inverse. */
	ret = inv_pos(rob_hw, wcp, overhead_pos, elbow_down, th123);

	if (ret == KIN_OK) {
		double pre_a4[4][4];
		double temp[4][4][4];
		double wristtarget[4][4];
		double l1 = rob_hw->model.lrob[5].trans[0][3];
		double l2 = rob_hw->model.lrob[5].trans[2][3];
		double cosval = rob_hw->model.lrob[4].trans[0][0];
		double sinval = rob_hw->model.lrob[4].trans[0][2];
		double r22, c5, th5, s5, bpos, j6_dir;
		double tf_j4[4][4];
		double tf_j5[4][4];
		double a6_t44[4][4];
		double fk_prel[4][4];

		q_inv_out[0] = th123[0];
		q_inv_out[1] = th123[1];
		q_inv_out[2] = th123[2];

		/* Compute orientation in WCP given arm angles. */

		fwd_lpoe_std(rob_hw->model.lrob, q_inv_out, 4, temp);
		memcpy(pre_a4, temp[3], 4*4*sizeof(double));

		inv_t44(pre_a4, tmp);
		mul44(tmp, tcp, wristtarget);

		r22 = wristtarget[2][2];
		c5 = (r22 - cosval*cosval)/(sinval*sinval);
		c5 = fmin(1.0, fmax(-1.0, c5));
		th5 = acos(c5);

		if (neg_a5 == 0) {
			q_inv_out[4] = fabs(th5);
		} else {
			q_inv_out[4] = -fabs(th5);
		}

		s5 = sin(q_inv_out[4]);
		bpos = atan2(l1*c5*cosval + l2*sinval, l1*s5);
		q_inv_out[3] = atan2(wristtarget[0][3], wristtarget[1][3]) - bpos;

		rot_z(q_inv_out[3]*rob_hw->rot_dirs[3], tf_j4);
		rot_z(q_inv_out[4]*rob_hw->rot_dirs[4], tf_j5);

		/* Reuse of pre_a4, called pre_a6 in python code */
		mul44(pre_a4, tf_j4, tmp);
		memcpy(pre_a4, tmp, 4*4*sizeof(double));
		mul44(pre_a4, rob_hw->model.lrob[4].trans, tmp);
		memcpy(pre_a4, tmp, 4*4*sizeof(double));
		mul44(pre_a4, tf_j5, tmp);
		memcpy(pre_a4, tmp, 4*4*sizeof(double));
		mul44(pre_a4, rob_hw->model.lrob[5].trans, tmp);
		memcpy(pre_a4, tmp, 4*4*sizeof(double));

		/* Reuse of variables, too ugly? */
		inv_t44(pre_a4, tmp);
		inv_t44(rob_hw->model.lrob[6].trans, tf_j4);
		mul44(tcp, tf_j4, tf_j5);
		mul44(tmp, tf_j5, a6_t44);

		j6_dir = rob_hw->rot_dirs[5];
		q_inv_out[5] = atan2(a6_t44[1][0], a6_t44[0][0])*j6_dir;

		/* Reuse of variables, too ugly? */
		rot_z(q_inv_out[5]*j6_dir, tf_j4);
		mul44(tf_j4, rob_hw->model.lrob[6].trans, tmp);
		mul44(pre_a4, tmp, fk_prel);

		normdiff(fk_prel, tcp, errs);

		/* Handle any couplings between the joints. */
		if (rob_hw->model.coupling_matrix) {
			int i, j;
			double q_tmp[6];
			for (i = 0; i < 6; i++) {
				q_tmp[i] = 0;
				for (j = 0; j < 6; j++)
					q_tmp[i] += rob_hw->model.inverse_coupling_matrix[i*6+j]*q_inv_out[j];
			}
			memcpy(q_inv_out, q_tmp, 6 * sizeof(double));
		} else if (rob_hw->model.has_pbar) {
			q_inv_out[2] += rob_hw->model.par_fact*q_inv_out[1];
		}

		/* Take offsets into account. */
		if (rob_hw->model.q_a_offs) {
			int i;
			for (i = 0; i < 6; i++)
				q_inv_out[i] -= rob_hw->model.q_a_offs[i];
		}

	}

	return ret;
}

/*!
 * Help function for nom_inv_ur to create a special t44
 *
 * @param[in] ang Angle
 * @param[in] ytr Translation along y-axis
 * @param[in] t1 Integer that may be 0 or 1
 * @param[out] out T44
 */
void _t_func(double ang, double ytr, int t1, double out[4][4]){
	double sa, ca, fac;
	memset(out, 0, 4 * 4 * sizeof(double));
	if (t1 > 0) {
		fac = -1.0;
	} else {
		fac = 1.0;
	}
	sa = sin(ang);
	ca = cos(ang);
	out[0][0] = ca;
	out[0][1] = -sa;
	out[1][2] = fac;
	out[1][3] = fac*ytr;
	out[2][0] = -fac*sa;
	out[2][1] = -fac*ca;
	out[3][3] = 1.0;
}

/*!
 * nominal inverse kinematics for a UR robot LPOE model (on closed form)
 *
 * ref: cb.kin.model_lpoe.UR.nom_inv()
 *
 * @param[in] rob Definition of nominal robot structure
 * @param[in] tcp T44 representing the flange of the robot.
 * @param[in] overhead_pos Should be 1 for overhead position (backward bending), 0 otherwise.
 * @param[in] elbow_down Should be 1 for the elbow down solution, 0 otherwise.
 * @param[in] neg_a5 Should be 1 for a solution where joint 5 has a strictly negative angle
 * @param[out] q_inv_out Calculated joint values.
 */
int nom_inv_ur(const struct model_lpoe_nom *rob, const double tcp[4][4], int overhead_pos,
		int elbow_down, int neg_a5, double q_inv_out[6])
{
	int i;
	double a2, a3, d1, d4, d5, d6, alpha1, alpha2, tmp1, tmp2;
	double p_0_5[3];
	double th1s[2];
	double t_0_1[4][4];
	double t_4_5[4][4];
	double t_5_6[4][4];
	double t_4_6[4][4];

	/* Pick out DH parameters (stored in wcp0) */
	a2 = rob->wcp0[0][2];
	a3 = rob->wcp0[0][3];
	d1 = rob->wcp0[2][0];
	d4 = rob->wcp0[2][3];
	d5 = rob->wcp0[3][0];
	d6 = rob->wcp0[3][1];

	for (i = 0; i < 3; i++) {
		p_0_5[i] = tcp[i][3] - d6*tcp[i][2];
	}

	alpha1 = atan2(p_0_5[1], p_0_5[0]);
	tmp2 = sqrt(pow(p_0_5[0], 2) + pow(p_0_5[1], 2));
	if (tmp2 < fabs(d4)) {
		if (tmp2 < 1e-8) {
			return KIN_UNREACHABLE;
		}
		tmp1 = d4/tmp2;
		if (fabs(tmp1 - 1) < 1e-10) {
			if (tmp1 > 0) {
				alpha2 = 0.0;
			} else {
				alpha2 = M_PI;
			}
		} else {
			return KIN_UNREACHABLE;
		}
	} else if (tmp2 < 1e-8) {
		alpha2 = 0.0;
	} else {
		alpha2 = acos(d4/tmp2);
	}

	th1s[0] = alpha1 + alpha2 + M_PI/2;
	th1s[1] = alpha1 - alpha2 + M_PI/2;

	tmp1 = -cos(th1s[0])*p_0_5[0] - sin(th1s[0])*p_0_5[1];
	tmp2 = -cos(th1s[1])*p_0_5[0] - sin(th1s[1])*p_0_5[1];

	if ((overhead_pos > 0 && tmp1 < tmp2) || (overhead_pos == 0 && tmp1 >= tmp2)) {
		q_inv_out[0] = th1s[0];
	} else {
		q_inv_out[0] = th1s[1];
	}

	alpha1 = sin(q_inv_out[0]);
	alpha2 = cos(q_inv_out[0]);

	tmp1 = tcp[0][3]*alpha1 - tcp[1][3]*alpha2 - d4;
	if (fabs(tmp1) > fabs(d6)) {
		if ((fabs(tmp1)-fabs(d6)) > 1e-10) {
			return KIN_UNREACHABLE;
		}
		if ((tmp1 > 0 && d6 > 0) || (tmp1 < 0 && d6 < 0)) {
			q_inv_out[4] = 0.0;
		} else {
			q_inv_out[4] = M_PI;
		}
	} else if (fabs(d6) < 1e-8) {
		return KIN_UNREACHABLE;
	} else {
		q_inv_out[4] = acos(tmp1/d6);
	}

	tmp1 = -tcp[0][1]*alpha1 + tcp[1][1]*alpha2;
	tmp2 = tcp[0][0]*alpha1 - tcp[1][0]*alpha2;
	alpha1 = sin(q_inv_out[4]);

	if (fabs(alpha1) < 1e-8 || ( fabs(tmp1) < 1e-8 && fabs(tmp2) < 1e-8 )) {
		q_inv_out[5] = 0.0;
	} else {
		q_inv_out[5] = atan2(tmp1/alpha1, tmp2/alpha1);
	}

	if ((neg_a5 > 0 && q_inv_out[4] >= 0) || (neg_a5 == 0 && q_inv_out[4] < 0)) {
		q_inv_out[4] *= -1;
		q_inv_out[5] += M_PI;
	}

	rot_z(q_inv_out[0], t_0_1);
	t_0_1[2][3] = d1;

	_t_func(q_inv_out[4], d5, 1, t_4_5);
	_t_func(q_inv_out[5], d6, 0, t_5_6);
	mul44(t_4_5, t_5_6, t_4_6);

	/* Now use t_4_5, t_5_6, and t_4_6 as temp variables */
	inv_t44(t_0_1, t_4_5);
	inv_t44(t_4_6, t_5_6);
	mul44(tcp, t_5_6, t_4_6);
	mul44(t_4_5, t_4_6, t_5_6); /* t_5_6 now is t_1_4 */

	tmp1 = pow(t_5_6[0][3], 2) + pow(t_5_6[2][3], 2);
	if (fabs(a2*a3) < 1e-10) {
		return KIN_UNREACHABLE;
	}
	tmp2 = (tmp1 - pow(a2, 2) - pow(a3, 2))/(a2*a3*2);
	if (fabs(tmp2) < 1) {
		q_inv_out[2] = acos(tmp2);
	} else {
		if ((fabs(tmp2) - 1) < 1e-10) {
			if (tmp2 > 0) {
				q_inv_out[2] = 0.0;
			} else {
				q_inv_out[2] = M_PI;
			}
		} else {
			return KIN_UNREACHABLE;
		}
	}

	if (elbow_down == 0) {
		q_inv_out[2] *= -1;
	}

	alpha1 = sqrt(tmp1);
	if (alpha1 < 1e-8) {
		return KIN_UNREACHABLE;
	}
	tmp2 = -a3*sin(q_inv_out[2])/alpha1;
	tmp1 = atan2(-t_5_6[2][3], -t_5_6[0][3]);
	if (fabs(tmp2) < 1) {
		q_inv_out[1] = tmp1 - asin(tmp2);
	} else {
		if ((fabs(tmp2) - 1) > 1e-10) {
			return KIN_UNREACHABLE;
		}
		if (tmp2 > 0) {
			q_inv_out[1] = tmp1 - M_PI/2;
		} else {
			q_inv_out[1] = tmp1 + M_PI/2;
		}
	}

	_t_func(q_inv_out[1], 0.0, 1, t_0_1);  /* t_1_2 */
	rot_z(q_inv_out[2], t_4_5);  /* t_2_3 */
	t_4_5[0][3] = a2;

	mul44(t_0_1, t_4_5, t_4_6);  /* t_4_6 = t_1_3 */

	inv_t44(t_4_6, t_0_1);

	mul44(t_0_1, t_5_6, t_4_5);  /* t_4_5 = t_3_4 */

	q_inv_out[3] = atan2(t_4_5[1][0], t_4_5[0][0]);


	/* Make sure -pi <= q_inv[:] <= pi. */
	for (i = 0; i < 6; i++) {
		if (q_inv_out[i] > M_PI)
			q_inv_out[i] -= 2*M_PI;
		else if (q_inv_out[i] < -M_PI)
			q_inv_out[i] += 2*M_PI;
	}

	return KIN_OK;
}

