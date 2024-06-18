#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(EXTRA_CALC) && !defined(CB_SYSTEM_HEADER)
	#include <stdint.h>
#endif

#ifdef CB_SYSTEM_HEADER
    #include CB_SYSTEM_HEADER
#else
    #include <math.h>
#endif

#ifndef _POSIX_C_SOURCE
    #define M_PI        3.14159265358979323846    /* pi */
    #define M_PI_2        1.57079632679489661923    /* pi/2 */
    #define M_PI_4        0.78539816339744830962    /* pi/4 */
#endif

#include "c_lpoe/lpoe.h"
#include "c_lpoe/matrix_math.h"
#include "ckin/lin_alg.h"
#include "ckin/common.h"
#include "kinematics.h"
#include "calibrated_kinematics.h"
#include "myalloc.h"

/**
 * Nominal angle values for inverse kinematics
 *
 * @param pos Position array of the TCP
 * @param orient z-orientation of the TCP
 * @param th_out Joint angles
 * @return Negative on error
 */
int nominal_angles(const struct agile_pkm_model *rob, double pos[3], double orient, double th_out[13])
{
	double q_[2][4];
	double q[4];
	double q23_[2];
	double q23;
    double tmp44[4][4];
	int ret;
	ret = inv_nom(rob, pos, orient, q23_, q_);
	q23 = q23_[0];
	memcpy(q, q_[0], 4*sizeof(double));
    q[1] = q[1] + M_PI_2;
	// Serial forward kinematics
    double tf1[4][4];
    rot_z(q[0], tf1);

    double tf2_0[4][4];
    eye(4, (double*)tf2_0);
    tf2_0[0][3] = rob->ax2x;
    tf2_0[1][3] = rob->ax2y;
    tf2_0[2][3] = rob->ax2z;

    double tf2_q[4][4];
    rot_z(q[1] + rob->q2_offs, tf2_q);

    double tf2_1[4][4];
    eye(4, (double*)tf2_1);
    tf2_1[0][3] = rob->L3b;

    double tf2_tmp[4][4];
    mul44(tf2_q, tf2_1, tf2_tmp);

    double tf2[4][4];
    mul44(tf2_0, tf2_tmp, tf2);

    double tf3_0[4][4];
    eye(4, (double*)tf3_0);
    tf3_0[0][3] = rob->L1;
    tf3_0[1][3] = rob->ax3y;
    tf3_0[2][3] = 0.0;

    double tf3_q[4][4];
    rot_x(q[2], tf3_q);

    double tf3[4][4];
    mul44(tf3_0, tf3_q, tf3);

    double tf3_31[4][4];
    eye(4, (double*)tf3_31);
    tf3_31[1][3] = rob->L3;

    double tf3_32[4][4];
    eye(4, (double*)tf3_32);
    tf3_32[1][3] = rob->L2 - rob->d2;

    double tf_L2[4][4];
    eye(4, (double*)tf_L2);
    tf_L2[1][3] = (rob->L2 - rob->L3 - rob->d2);

	double tf3_2[4][4];
	rot_z(q23, tf3_2);

	double tf3_[4][4];
	mul44(tf3, tf3_2, tf3_);

	double tf4_[4][4];
	mul44(tf3_, tf3_31, tf4_);

	//# Add L2 distance
	double out[4][4];
	mul44(tf4_, tf_L2, out);

	double qx = asin(-out[2][1]);
	double qy = atan2(out[2][0], out[2][2]);

	double tf2_in_4_[4][4];
	trans_inv(tf4_, tmp44);
	mul44(tmp44, tf2, tf2_in_4_);

	double v3[2] = {tf2_in_4_[0][3], tf2_in_4_[2][3]};
	double tmp1_2 = sqrt(v3[0]*v3[0] + v3[1]*v3[1]);
	double tmp2_2 = tf2_in_4_[1][3];
	double beta1 = -(acos(-1) + atan2(v3[1], v3[0]));
	double beta2 = atan2(tmp1_2, tmp2_2);

	q[1] -= M_PI_2;
	if (beta1 < -M_PI) {
		beta1 += 2.0*M_PI;
	}

	double th_tmp[15] = {q[0], q[1], q[2], q[3], q23, q[2], q23, beta1, beta2, qy, -qx, qy, -qx, 0.0, 0.0};
	memcpy(th_out, th_tmp, 15*sizeof(double));
	return ret;
}

/**
 * Iterative inverse kinematics. If rob_lpoe, dpar or tool is NULL, ax3 is considered stiff.
 *
 * @param pos Position array of the TCP
 * @param orient z-orientation of the TCP
 * @param q_out Motor angles
 * @param tol MSE tolerance for the numerical iterations
 * @param max_iter max number of iterations
 * @return Negative on error
 */
 int inv_calib_full(const struct model_hkm* rob, const struct model_lpoe_agile_pkm* rob_lpoe,
 	const struct dyn_params_link dpar[], const struct dyn_params_link* tool,
 	double pos[3], double orient, double q_out[4], double tol, int max_iter)
 {
 	int ret, i, j, k, iter;
 	int converged = 0;
 	double th[15];

 	struct model_lpoe_link frame_tmp[4];
 	for (k = 0; k < 4; k++) {
 		double tmp_s[6] = {0.0};
 		if (k > 0) {
 			tmp_s[6-k] = 1.0;
 		}
 		double tmp_t[4][4];
 		eye(4, (double*) tmp_t);
 		if (k == 0) {
 			for (i = 0; i < 3; i++) {
 				tmp_t[i][3] = pos[i];
 			}
 		}
 		struct model_lpoe_link single_tmp = {
 			.s = {tmp_s[0], tmp_s[1], tmp_s[2], tmp_s[3], tmp_s[4], tmp_s[5]},
 			.trans = {{tmp_t[0][0], tmp_t[0][1], tmp_t[0][2], tmp_t[0][3]},
 						{tmp_t[1][0], tmp_t[1][1], tmp_t[1][2], tmp_t[1][3]},
 						{tmp_t[2][0], tmp_t[2][1], tmp_t[2][2], tmp_t[2][3]},
 						{tmp_t[3][0], tmp_t[3][1], tmp_t[3][2], tmp_t[3][3]}},
 		};
 		frame_tmp[k] = single_tmp;
 	}
 	struct model_lpoe frame_tcp = {
 		.lrob = frame_tmp,
 		.n_screw = 4,
 	};

 	if (tol <= 0.0) {
 		tol = DEFAULT_TOL;
	}
	if (max_iter <= 0) {
 		max_iter = DEFAULT_MAX_ITER;
	}

 	ret = nominal_angles(rob->nominal_model, pos, orient, th);
 	if (ret != 0) {
 		return ret;
 	}

 	for (iter = 0; iter < max_iter; iter++) {
 		double q3 = th[2];
 		if (rob_lpoe != NULL && dpar != NULL && tool != NULL) {
 			double q[4] = {th[0], th[1], th[2], th[3]};
 			double dq3 = 0;
 			ret = _find_dq3(rob->nominal_model, rob_lpoe, dpar, tool, q, rob->ax3_stiffness, &dq3);
			/* fwd_nom may fail here if an inv_full() iteration brings us outside of
			possible motor values. In that case, simply ignore ax3 elasticity for that
			iteration. This is represented by error code 2.*/
			if (ret != 0 && ret != 2) {
				return ret;
			}
			q3 = th[2] - dq3;
 		}
#ifdef DYNMEM
		double* th_tmp = (double*)myalloc(6 * sizeof(double));
		double (*jac_t_tcp_spat)[6] = (double(*)[6])mymatrixalloc(6, 6);

		double (*tf4_0)[4] = (double(*)[4])mymatrixalloc(4, 4);
		double (*tf4_6)[4] = (double(*)[4])mymatrixalloc(4, 4);
		double (*tf9_7)[4] = (double(*)[4])mymatrixalloc(4, 4);
		double (*tf9_8)[4] = (double(*)[4])mymatrixalloc(4, 4);
		double (*tf_tcp)[4] = (double(*)[4])mymatrixalloc(4, 4);

		double (*jac_t_4_0)[6] = (double(*)[6])mymatrixalloc(2, 6);
		double (*jac_t_4_6)[6] = (double(*)[6])mymatrixalloc(5, 6);
		double (*jac_t_9_7)[6] = (double(*)[6])mymatrixalloc(6, 6);
		double (*jac_t_9_8)[6] = (double(*)[6])mymatrixalloc(6, 6);
		double (*jac_t_tcp)[6] = (double(*)[6])mymatrixalloc(6, 6);
#else
		double th_tmp[6] = { th[0], th[1] };
		double jac_t_tcp_spat[6][6] = { 0 };

 		double tf4_0[4][4] = {0};
 		double tf4_6[4][4] = {0};
 		double tf9_7[4][4] = {0};
 		double tf9_8[4][4] = {0};
 		double tf_tcp[4][4] = {0};

 		double jac_t_4_0[2][6] = {0};
 		double jac_t_4_6[5][6] = {0};
 		double jac_t_9_7[6][6] = {0};
 		double jac_t_9_8[6][6] = {0};
 		double jac_t_tcp[6][6] = {0};
#endif
		th_tmp[0] = th[0];
		th_tmp[1] = th[1];
 		spatial_jacobian_transpose(&rob->frame4_0, th_tmp, tf4_0, jac_t_tcp_spat);
 		spatial_to_normal_jacobian(jac_t_tcp_spat, 2, tf4_0, jac_t_4_0);
 		th_tmp[1] = th[5];
 		th_tmp[2] = th[6];
 		th_tmp[3] = th[7];
 		th_tmp[4] = th[8];
 		spatial_jacobian_transpose(&rob->frame4_6, th_tmp, tf4_6, jac_t_tcp_spat);
 		spatial_to_normal_jacobian(jac_t_tcp_spat, 5, tf4_6, jac_t_4_6);
 		th_tmp[3] = th[11];
 		th_tmp[4] = th[12];
 		th_tmp[5] = th[3];
 		spatial_jacobian_transpose(&rob->frame9_8, th_tmp, tf9_8, jac_t_tcp_spat);
 		spatial_to_normal_jacobian(jac_t_tcp_spat, 6, tf9_8, jac_t_9_8);
 		th_tmp[1] = q3;
 		th_tmp[2] = th[4];
 		th_tmp[3] = th[9];
 		th_tmp[4] = th[10];
 		spatial_jacobian_transpose(&rob->frame9_7, th_tmp, tf9_7, jac_t_tcp_spat);
 		spatial_to_normal_jacobian(jac_t_tcp_spat, 6, tf9_7, jac_t_9_7);
 		th_tmp[0] = orient;
 		th_tmp[1] = th[14];
 		th_tmp[2] = th[13];
 		spatial_jacobian_transpose(&frame_tcp, th_tmp, tf_tcp, jac_t_tcp_spat);
 		spatial_to_normal_jacobian(jac_t_tcp_spat, 3, tf_tcp, jac_t_tcp);
#ifdef DYNMEM
 		myfree(th_tmp);
 		myfree(jac_t_tcp_spat);
 		double* diff7_8 = (double*)myalloc(6 * sizeof(double));
 		double* diff7_t = (double*)myalloc(6 * sizeof(double));
#else
 		double diff7_t[6];
 		double diff7_8[6];
#endif
 		tr2diff(tf9_8, tf9_7, diff7_8);
 		tr2diff(tf_tcp, tf9_7, diff7_t);
#ifdef DYNMEM
 		myfree(tf9_7);
 		myfree(tf9_8);
 		myfree(tf_tcp);
 		double* f_ = (double*)myalloc(15 * sizeof(double));
#else
		double f_[15];
#endif
 		double cost = 0.0;
 		for (i = 0; i < 3; i++) {
 			f_[i] = tf4_6[i][3] - tf4_0[i][3];
 			cost += pow(f_[i], 2);
 		}
 		for (i = 0; i < 6; i++) {
 			f_[3+i] = diff7_8[i];
 			cost += pow(f_[3+i], 2);
 			f_[9+i] = diff7_t[i];
 			cost += pow(f_[9+i], 2);
 		}
#ifdef DYNMEM
 		myfree(tf4_0);
 		myfree(tf4_6);
 		myfree(diff7_8);
 		myfree(diff7_t);
#endif
 		cost = sqrt(cost);
 		if (cost < tol) {
 			converged = 1;
#ifdef DYNMEM
			myfree(f_);
			myfree(jac_t_4_0);
			myfree(jac_t_4_6);
			myfree(jac_t_9_7);
			myfree(jac_t_9_8);
			myfree(jac_t_tcp);
#endif
 			break;
 		}

 		/*Insert into total Jacobian*/
#ifdef DYNMEM
 		double (*jac_total)[15] = (double(*)[15])mymatrixalloc(15, 15);
		for (i = 0; i < 15; i++) {
			for (j = 0; j < 15; j++) {
				jac_total[i][j] = 0;
			}
		}
#else
 		double jac_total[15][15] = {0};
#endif
 		for (i = 0; i < 3; i++) {
 			for (j = 5; j < 9; j++) {
 				jac_total[i][j] = jac_t_4_6[j-4][i];
 			}
 			jac_total[i][0] = jac_t_4_6[0][i];
 			jac_total[i][0] -= jac_t_4_0[0][i];
 			jac_total[i][1] -= jac_t_4_0[1][i];
 		}
 		for (i = 3; i < 9; i++) {
 			jac_total[i][0] = jac_t_9_7[0][i-3];
 			jac_total[i][2] = jac_t_9_7[1][i-3];
 			jac_total[i][4] = jac_t_9_7[2][i-3];
 			jac_total[i][9] = jac_t_9_7[3][i-3];
 			jac_total[i][10] = jac_t_9_7[4][i-3];
 			jac_total[i][3] = jac_t_9_7[5][i-3];
 			jac_total[i][0] -= jac_t_9_8[0][i-3];
 			jac_total[i][5] -= jac_t_9_8[1][i-3];
 			jac_total[i][6] -= jac_t_9_8[2][i-3];
 			jac_total[i][11] -= jac_t_9_8[3][i-3];
 			jac_total[i][12] -= jac_t_9_8[4][i-3];
 			jac_total[i][3] -= jac_t_9_8[5][i-3];
 		}
 		for (i = 9; i < 15; i++) {
 			jac_total[i][0] = jac_t_9_7[0][i-9];
 			jac_total[i][2] = jac_t_9_7[1][i-9];
 			jac_total[i][4] = jac_t_9_7[2][i-9];
 			jac_total[i][9] = jac_t_9_7[3][i-9];
 			jac_total[i][10] = jac_t_9_7[4][i-9];
 			jac_total[i][3] = jac_t_9_7[5][i-9];
 		}
 		for (i = 12; i < 15; i++) {
 			jac_total[i][13] = -jac_t_tcp[2][i-9];
 			jac_total[i][14] = -jac_t_tcp[1][i-9];
 		}
#ifdef DYNMEM
 		myfree(jac_t_4_0);
 		myfree(jac_t_4_6);
 		myfree(jac_t_9_7);
 		myfree(jac_t_9_8);
 		myfree(jac_t_tcp);
 		double* sol = (double*)myalloc(15 * sizeof(double));
#else
 		double sol[15];
#endif
 		ret = solve_15x15_15_damped_ls(jac_total, f_, 0.0, sol);
 		if (ret != 0) {
 			/* Solving system failed -> add non-zero lambda*/
 			double lambda;
 			for (lambda = 0.00001; lambda <1; lambda*=10) {
 				ret = solve_15x15_15_damped_ls(jac_total, f_, lambda, sol);
 				if (ret == 0) {
 					break;
 				}
 			}
 		}
 		if (ret != 0) {
 			return ret;
 		}
 		for (i = 0; i < 15; i++) {
 			th[i] -= sol[i];
 		}
#ifdef DYNMEM
 		myfree(jac_total);
 		myfree(f_);
 		myfree(sol);
#endif
 	}
 	for (i = 0; i < 4; i++) {
 		q_out[i] = th[i];
 	}
	if (rob_lpoe != NULL && dpar != NULL && tool != NULL) {
		q_out[2] -= rob->ax3_offs;
	}

 	if (converged == 1) {
 		return 0;
 	}
 	return -1;
 }

int inv_calib(const struct model_hkm *rob, double pos[3], double orient,
	double q_out[4], double tol, int max_iter)
{
	return inv_calib_full(rob, NULL, NULL, NULL, pos, orient, q_out, tol, max_iter);
}

/**
 * Iterative forward kinematics. If rob_lpoe, dpar or tool is NULL, ax3 is considered stiff.
 *
 * @param q Motor angles
 * @param out TCP T44
 * @param orient z-orientation of the TCP
 * @param tol MSE tolerance for the numerical iterations
 * @param max_iter Max number of iterations
 * @return Negative on error
 */
int fwd_calib_full(const struct model_hkm* rob, const struct model_lpoe_agile_pkm* rob_lpoe,
	const struct dyn_params_link dpar[], const struct dyn_params_link* tool,
	const double q_in[4], double out[4][4], double* orient, double elbow_out[4][4], double tol, int max_iter)
{
	int ret, i, j, iter;
	int converged = 0;
	double f_[9];
	double sol[9];
	double tf4_0[4][4] = {0};
	double tf4_6[4][4] = {0};
	double tf9_7[4][4] = {0};
	double tf9_8[4][4] = {0};
	double jac_t_tmp[6][6] = {0};
	double jac_t_4_6[5][6] = {0};
	double jac_t_9_7[6][6] = {0};
	double jac_t_9_8[6][6] = {0};
	double th[9];
	double th_tmp[6] = {q_in[0], q_in[1]};

	if (tol <= 0.0)
		tol = DEFAULT_TOL;
   	if (max_iter <= 0)
		max_iter = DEFAULT_MAX_ITER;

	spatial_jacobian_transpose(&rob->frame4_0, th_tmp, tf4_0, jac_t_tmp);

	double temp_out_tcp[4][4];
	double orient_angle;
	res_q_extra_t res_q_extra;

	double q[4] = {q_in[0], q_in[1], q_in[2], q_in[3]};

#ifdef EXTRA_CALC
	ax4_fwd_ret_t temp_ax4_extra;
    double tmp44[4][4];
	ret = fwd_nom(rob->nominal_model, q, temp_out_tcp, &orient_angle, tmp44, &res_q_extra, &temp_ax4_extra);
#else
	ret = fwd_nom(rob->nominal_model, q, temp_out_tcp, &orient_angle, &res_q_extra);
#endif

	if (rob_lpoe != NULL && dpar != NULL && tool != NULL) {
		q[2] += rob->ax3_offs;
		double dq3 = 0;
		ret = _find_dq3(rob->nominal_model, rob_lpoe, dpar, tool, q, rob->ax3_stiffness, &dq3);
		if (ret != 0) {
			return ret;
		}
		q[2] -= dq3;
	}
	th[0] = res_q_extra.q23;
	th[1] = q[2];
	th[2] = th[0];
	th[3] = res_q_extra.beta1;
	th[4] = res_q_extra.beta2;
	th[5] = res_q_extra.qy;
	th[6] = -res_q_extra.qx;
	th[7] = th[5];
	th[8] = th[6];

	/* Numerical iterations */
	for (iter = 0; iter < max_iter; iter++) {
		th_tmp[0] = q[0];
		th_tmp[1] = th[1];
		th_tmp[2] = th[2];
		th_tmp[3] = th[3];
		th_tmp[4] = th[4];
		spatial_jacobian_transpose(&rob->frame4_6, th_tmp, tf4_6, jac_t_tmp);
		spatial_to_normal_jacobian(jac_t_tmp, 5, tf4_6, jac_t_4_6);
		th_tmp[3] = th[7];
		th_tmp[4] = th[8];
		th_tmp[5] = q[3];
		spatial_jacobian_transpose(&rob->frame9_8, th_tmp, tf9_8, jac_t_tmp);
		spatial_to_normal_jacobian(jac_t_tmp, 6, tf9_8, jac_t_9_8);
		th_tmp[1] = q[2];
		th_tmp[2] = th[0];
		th_tmp[3] = th[5];
		th_tmp[4] = th[6];
		spatial_jacobian_transpose(&rob->frame9_7, th_tmp, tf9_7, jac_t_tmp);
		spatial_to_normal_jacobian(jac_t_tmp, 6, tf9_7, jac_t_9_7);

		double diff7_8[6];
		tr2diff(tf9_8, tf9_7, diff7_8);

		double cost = 0.0;
		for (i = 0; i < 3; i++) {
			f_[i] = tf4_6[i][3] - tf4_0[i][3];
			cost += pow(f_[i], 2);
		}
		for (i = 0; i < 6; i++) {
			f_[3+i] = diff7_8[i];
			cost += pow(f_[3+i], 2);
		}

		cost = sqrt(cost);
		if (cost < tol) {
			converged = 1;
			break;
		}

		/*Insert into total Jacobian*/
		double jac_total[9][9] = {0};
		for (i = 0; i < 3; i++) {
			for (j = 1; j < 5; j++) {
				jac_total[i][j] = jac_t_4_6[j][i];
			}
		}
		for (i = 3; i < 9; i++) {
			jac_total[i][0] = jac_t_9_7[2][i-3];
			jac_total[i][5] = jac_t_9_7[3][i-3];
			jac_total[i][6] = jac_t_9_7[4][i-3];
			jac_total[i][1] = -jac_t_9_8[1][i-3];
			jac_total[i][2] = -jac_t_9_8[2][i-3];
			jac_total[i][7] = -jac_t_9_8[3][i-3];
			jac_total[i][8] = -jac_t_9_8[4][i-3];
		}

		ret = solve_9x9_9_damped_ls(jac_total, f_, 0.0, sol);
		if (ret != 0) {
			/* Solving system failed -> add non-zero lambda*/
			double lambda;
			for (lambda = 0.00001; lambda <1; lambda*=10) {
				ret = solve_9x9_9_damped_ls(jac_total, f_, lambda, sol);
				if (ret == 0) {
					break;
				}
			}
		}
		if (ret != 0) {
			return ret;
		}
		for (i = 0; i < 9; i++) {
			th[i] -= sol[i];
		}
	}
	memcpy(out, tf9_7, 4*4*sizeof(double));
	double tmp_zyx[3] = {0};
	tf2euler_zyx_intrinsic(out, tmp_zyx);
	*orient = tmp_zyx[0];

    double rotz[4][4];
    rot_z(q_in[0], rotz);
    double tf3_0[4][4];
    eye(4, (double *)tf3_0);
    tf3_0[0][3] = rob->nominal_model->L1;
    tf3_0[1][3] = rob->nominal_model->ax3y;
    tf3_0[2][3] = 0.0;
    double tf3_0_0[4][4];
    mul44(rotz, tf3_0, tf3_0_0);

#ifndef EXTRA_CALC
    double tmp44[4][4];
#endif
    inv_t44(tf3_0_0, tmp44);
    double tf_tcp_in_3_0[4][4];
    mul44(tmp44, out, tf_tcp_in_3_0);
    double ori_nom = q_in[0] + q_in[3] - atan2(tf_tcp_in_3_0[0][3], tf_tcp_in_3_0[1][3]);

    mul44(rob->frame4_6.lrob[0].trans, rotz, tmp44);
    mul44(tmp44, rob->frame4_6.lrob[1].trans, elbow_out);

    while (ori_nom > M_PI) {
        *orient += 2 * M_PI;
        ori_nom -= 2 * M_PI;
    }
    while (ori_nom < -M_PI) {
        *orient -= 2 * M_PI;
        ori_nom += 2 * M_PI;
    }

	if (converged == 1) {
		return 0;
	}
	return -1;
}

int fwd_calib(const struct model_hkm *rob, const double q[4], double out[4][4],
	double* orient, double elbow_out[4][4], double tol, int max_iter)
{
	return fwd_calib_full(rob, NULL, NULL, NULL, q, out, orient, elbow_out, tol, max_iter);
}