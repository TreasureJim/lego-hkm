/*
 * Copyright (C) 2020 Cognibotics
 *
 * Use or copying of this file or any part of it, without explicit
 * permission from Cognibotics is not allowed in any way.
 */
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

#include <string.h>
#include <stdio.h>

#include "dyn.h"
#include "c_lpoe/lpoe.h"
#include "c_lpoe/matrix_math.h"
#include "kinematics.h"


 /*!
  * Fwd kin for joint
  *
  * @param[in] s Screw for joint
  * @param[in] q Joint angle
  * @param[out] fk Forward kin t44 for joint
  */
void joint_calc(const double s[6], double q, double fk[4][4])
{
        double sum_v = fabs(s[0]) + fabs(s[1]) + fabs(s[2]);
        double sum_w = fabs(s[3]) + fabs(s[4]) + fabs(s[5]);
        if (sum_v == 0 && sum_w == 1.0 && fabs(s[3]) == 1.0) {
                rot_x(q, fk);
        } else if (sum_v == 0 && sum_w == 1.0 && fabs(s[4]) == 1.0) {
                rot_y(q, fk);
        } else if (sum_v == 0 && sum_w == 1.0 && fabs(s[5]) == 1.0) {
                rot_z(q, fk);
        } else {
                double twist[6];
                if (sum_w == 0) {
                        memcpy(twist, s, 6 * sizeof(double));
                } else {
                        /* Compute -omega(s[k]) x v(s[k]) = v(s[k]) x omega(s[k]) */
                        xprod(twist_v(s), twist_omega(s), twist_v_w(twist));
                        memcpy(twist_omega_w(twist), twist_omega(s), 3 * sizeof(double));
                }
                twist_exp_hat(twist, q, fk);
        }
}


/*!
 * Fwd kin for for model
 *
 * @param[in] rob LPOE robot definition
 * @param[in] q Joint angles
 * @param[out] tfs Fwd kin t44s for each link in the robot
 */
void kin(const struct model_lpoe_agile_pkm* rob, const double q[9], double tfs[9][4][4])
{
        int i;
        double joint_fwd[4][4];

        double q_tmp[9];
        memcpy(q_tmp, q, 9 * sizeof(double));
        q_tmp[6] = q_tmp[6] + M_PI_2;

        for (i = 0; i < rob->n_screws; i++) {
                joint_calc(rob->lrob[i].s, q_tmp[i], joint_fwd);
                mul44(rob->lrob[i].trans, joint_fwd, tfs[i]);
        }
}


/*!
 * Transform coordinate frame
 *
 * Transform a quantity from frame a to frame b when the transformation from
 * a to b is given by tf
 *
 * c_b = tf[:3, :3]^T * c_a
 *
 * @param[in] tf Transformation from frame a to frame b
 * @param[in] c_a Quantity expressed in frame a
 * @param[out] c_b Quantity expressed in frame b
 */
void transform_a2b(const double tf[4][4], const double c_a[3], double c_b[3])
{
        int i, j;

        for (i = 0; i < 3; i++) {
                c_b[i] = 0;
                for (j = 0; j < 3; j++) {
                        c_b[i] += tf[j][i] * c_a[j];
                }
        }
}


/*!
 * Transform coordinate frame
 *
 * Transform a quantity from frame b to frame a when the transformation from
 * a to b is given by tf
 *
 * c_a = tf[:3, :3] * c_b
 *
 * @param[in] tf Transformation from frame a to frame b
 * @param[in] c_b Quantity expressed in frame b
 * @param[out] c_a Quantity expressed in frame a
 */
void transform_b2a(const double tf[4][4], const double c_b[3], double c_a[3])
{
        int i, j;

        for (i = 0; i < 3; i++) {
                c_a[i] = 0;
                for (j = 0; j < 3; j++) {
                        c_a[i] += tf[i][j] * c_b[j];
                }
        }
}


/*!
 * Inverse dynamics for the extended agile robot
 *
 * Torque is calculated for all joints, also those that are unactuated.
 *
 * @param[in] rob LPOE robot definition
 * @param[in] dpar Dynamic parameters
 * @param[in] tool Dynamic parameters for tool/load
 * @param[in] q Joint angles
 * @param[in] qd Joint velocities
 * @param[in] qdd Joint accelerations
 * @param[in] g Acceleration of gravity
 * @param[out] trq Fwd kin t44s for each link in the robot
 */
int dyn_inv_agile_extended(const struct model_lpoe_agile_pkm* rob,
        const struct dyn_params_link dpar[], const struct dyn_params_link* tool,
        const double q_in[9], const double qd[9], const double qdd[9],
        const double g[3], double trq[9])
{
        int i, j, k, ind1, ind2;
        double tfs[9][4][4];
        double ws[11][3];
        double wds[11][3];
        double accs[11][3];
        double acc_cogs[11][3];
        double rs[11][3];
        double fs[11][3];
        double ts[11][3];

        double w_joint[3];
        double wd_joint[3];
        double prev_w[3];
        double prev_wd[3];
        double tmp[3];
        double fnew[3];
        double tnew[3];
        int prev;

        double q[9] = {q_in[0], q_in[1], q_in[2], q_in[3], q_in[4], q_in[5],
                q_in[6], q_in[7], q_in[8]};

        kin(rob, q, tfs);

        /* Initial values for outward recursion */
        memset(ws[0], 0, 3 * sizeof(double));
        memset(wds[0], 0, 3 * sizeof(double));
        accs[0][0] = -g[0];
        accs[0][1] = -g[1];
        accs[0][2] = -g[2];
        acc_cogs[0][0] = -g[0];
        acc_cogs[0][1] = -g[1];
        acc_cogs[0][2] = -g[2];
        memset(rs[0], 0, 3 * sizeof(double));


        for (k = 1; k < 10; k++) {
                for (j = 0; j < 3; j++) {
                        w_joint[j] = qd[k - 1] * rob->lrob[k - 1].s[3 + j];
                        wd_joint[j] = qdd[k - 1] * rob->lrob[k - 1].s[3 + j];
                }
                prev = rob->lam[k - 1];
                transform_a2b(tfs[k - 1], ws[prev], prev_w);
                transform_a2b(tfs[k - 1], wds[prev], prev_wd);
                xprod(prev_w, w_joint, tmp);
                for (j = 0; j < 3; j++) {
                        ws[k][j] = prev_w[j] + w_joint[j];
                        wds[k][j] = prev_wd[j] + wd_joint[j] + tmp[j];
                }

                if (k == rob->chain_ends[0]) {
                        for (j = 0; j < 3; j++) {
                                rs[k][j] = rob->last_trans[0][j][3];
                        }
                } else if (k == rob->chain_ends[1]) {
                        for (j = 0; j < 3; j++) {
                                rs[k][j] = rob->last_trans[1][j][3];
                        }
                } else {
                        for (j = 0; j < 3; j++) {
                                rs[k][j] = tfs[k][j][3];
                        }
                }

                memcpy(tmp, accs[prev], 3 * sizeof(double));
                i = rob->mu_inds[prev][1] - rob->mu_inds[prev][0];
                if (i > 1) {
                        ind2 = -1;
                        for (j = 0; j < i; j++) {
                                ind1 = rob->mu_inds[prev][0] + j;
                                if (k == rob->mu[ind1]) {
                                        ind2 = rob->mu[ind1] - 1;
                                        break;
                                }
                        }
                        if (ind2 < 0) {
                                return 1;
                        }
                        if (ind2 != prev) {
                                /* Use w_joint, wd_joint, prev_w, prev_wd as temp variables */
                                for (j = 0; j < 3; j++) {
                                        /* w_joint = r_star_prev */
                                        w_joint[j] = tfs[ind2][j][3] - rs[prev][j];
                                }
                                xprod(wds[prev], w_joint, wd_joint);  /* wd_joint = cross(wds[prev], r_star_prev) */
                                xprod(ws[prev], w_joint, prev_w);  /* prev_w = cross(ws[prev], r_star_prev) */
                                xprod(ws[prev], prev_w, prev_wd);  /* prev_wd = cross(ws[prev], cross(ws[prev],  r_star_prev)) */
                                for (j = 0; j < 3; j++) {
                                        tmp[j] = accs[prev][j] + wd_joint[j] + prev_wd[j];
                                }
                        }
                }
                transform_a2b(tfs[k - 1], tmp, prev_wd);
                xprod(wds[k], rs[k], wd_joint);  /* wd_joint = cross(wds[k], r_star) */
                xprod(ws[k], rs[k], prev_w);  /* prev_w = cross(ws[k], r_star) */
                xprod(ws[k], prev_w, tmp);  /* tmp = cross(ws[k], cross(ws[k], r_star)) */
                for (j = 0; j < 3; j++) {
                        accs[k][j] = prev_wd[j] + wd_joint[j] + tmp[j];
                }

                xprod(wds[k], dpar[k].r, wd_joint);  /* wd_joint = cross(wds[k], rcg) */
                xprod(ws[k], dpar[k].r, prev_w);  /* prev_w = cross(ws[k], rcg) */
                xprod(ws[k], prev_w, tmp);  /* tmp = cross(ws[k], cross(ws[k], rcg)) */
                for (j = 0; j < 3; j++) {
                        acc_cogs[k][j] = prev_wd[j] + wd_joint[j] + tmp[j];
                }
        }

        /* tool/load handling */
        memcpy(ws[10], ws[rob->tool_body], 3 * sizeof(double));
        memcpy(wds[10], wds[rob->tool_body], 3 * sizeof(double));
        memcpy(accs[10], accs[rob->tool_body], 3 * sizeof(double));
        xprod(wds[10], tool->r, wd_joint);  /* wd_joint = cross(wds[10], rcg_load) */
        xprod(ws[10], tool->r, prev_w);  /* prev_w = cross(ws[10], rcg_load) */
        xprod(ws[10], prev_w, tmp);  /* tmp = cross(ws[10], cross(ws[10], rcg_load)) */
        for (j = 0; j < 3; j++) {
                acc_cogs[k][j] = accs[10][j] + wd_joint[j] + tmp[j];
                fs[10][j] = tool->m * acc_cogs[k][j];
        }

        xprod(fs[10], tool->r, wd_joint);  /* wd_joint = cross(fnew, rcg_load) */
        mul33_3(tool->ival, wds[10], prev_wd);  /* prev_wd = i_load*wds[10] */
        mul33_3(tool->ival, ws[10], prev_w);  /* prev_w = i_load*ws[10] */
        xprod(ws[10], prev_w, tmp);  /* tmp = cross(ws[10], i_load*ws[10]) */
        for (j = 0; j < 3; j++) {
                ts[10][j] = -wd_joint[j] + prev_wd[j] + tmp[j];
        }

        for (k = 9; k >= 0; k--) {
                mul33_3(dpar[k].ival, wds[k], tmp);  /* tmp = imat*wds[k] */
                mul33_3(dpar[k].ival, ws[k], w_joint);  /* w_joint = imat*ws[k] */
                xprod(ws[k], w_joint, wd_joint);  /* wd_joint = cross(ws[k], imat*ws[k]) */
                for (j = 0; j < 3; j++) {
                        fnew[j] = dpar[k].m * acc_cogs[k][j];
                        tnew[j] = tmp[j] + wd_joint[j];
                }
                for (i = rob->mu_inds[k][0]; i < rob->mu_inds[k][1]; i++) {
                        ind1 = rob->mu[i];
                        transform_b2a(tfs[ind1 - 1], fs[ind1], tmp);  /* tmp = prev_force */
                        transform_b2a(tfs[ind1 - 1], ts[ind1], w_joint);  /* w_joint = prev_torque */
                        for (j = 0; j < 3; j++) {
                                fnew[j] += tmp[j];
                                prev_w[j] = dpar[k].r[j] - tfs[ind1 - 1][j][3];  /* prev_w = rcg - r_star */
                        }
                        xprod(tmp, prev_w, prev_wd);  /* prev_wd = cross(prev_force, rcg-r_star) */
                        for (j = 0; j < 3; j++) {
                                tnew[j] += w_joint[j] + prev_wd[j];
                        }
                }
                if (k == rob->tool_body) {
                        ind1 = -1;
                        for (i = 0; i < 2; i++) {
                                if (rob->chain_ends[i] == k) {
                                        ind1 = i;
                                        break;
                                }
                        }
                        if (ind1 >= 0) {
                                transform_b2a(rob->last_trans[ind1], fs[10], prev_w);  /* prev_w = prev_force */
                                transform_b2a(rob->last_trans[ind1], ts[10], prev_wd);  /* prev_wd = prev_torque */
                                for (j = 0; j < 3; j++) {
                                        tmp[j] = rob->last_trans[ind1][j][3];  /* tmp = r_star */
                                }
                        } else {
                                memcpy(prev_w, fs[10], 3 * sizeof(double));
                                memcpy(prev_wd, ts[10], 3 * sizeof(double));
                                memset(tmp, 0, 3 * sizeof(double));
                        }
                        for (j = 0; j < 3; j++) {
                                w_joint[j] = dpar[k].r[j] - tmp[j];  /* w_joint = rcg - r_star */
                        }
                        xprod(prev_w, w_joint, wd_joint);  /* wd_joint = cross(prev_force, rcg-r_star) */
                        for (j = 0; j < 3; j++) {
                                fnew[j] += prev_w[j];
                                tnew[j] += prev_wd[j] + wd_joint[j];
                        }
                }
                memcpy(fs[k], fnew, 3 * sizeof(double));
                xprod(fnew, dpar[k].r, tmp);  /* tmp = cross(fnew, rcg) */
                for (j = 0; j < 3; j++) {
                        ts[k][j] = tnew[j] - tmp[j];
                }
                if (k > 0) {
                        for (j = 0; j < 3; j++) {
                                tmp[j] = rob->lrob[k - 1].s[j + 3];  /* tmp = rot_axis */
                        }
                        trq[k - 1] = mul3_3(tmp, ts[k]);
                }
        }
        return 0;
}


/*!
 * Calculate the matrix G (and possibly the vector g)
 *
 * G = df/dq where f = f(q) is a vector of all relevant angles for the lpoe model
 *
 * ydot = G*qdot
 * ydotdot = G*qdotdot + g  ( g = Gdot*qdot )
 *
 * The calculation of G (and g) is via numerical differentiation. The input
 * extra_accurate controls if a one-sided difference quotient (extra_accurate=0)
 * or a two-sided (extra_accurate=1) difference quotient is used. The accuracy is
 * improved with a two-sided difference quotient, but the computational burden
 * is increased.
 *
 * The vector g is only calculated if the input calc_gvec is 1.
 *
 * @param[in] rob Robot kinematics definition
 * @param[in] q Joint angles
 * @param[in] delta Step size used for numerical differentiation
 * @param[in] extra_accurate Set to 1 to use two-sided difference quotient instead of one-sided
 * @param[in] calc_gvec Set to 1 to calculate the vector g
 * @param[in] qd Joint velocities
 * @param[out] qe_nom Extended joint vector
 * @param[out] gmat The matrix G
 * @param[out] gvec The vector g (note that this will only be filled with values if calc_gvec is 1)
 */
int calc_gmat(const struct agile_pkm_model* rob, const double q[4],
        double delta, int extra_accurate, int calc_gvec,
        const double qd[4], double qe_nom[9], double gmat[9][4], double gvec[9])
{
        int ret, i, j, k, stop_ind;
        double th[4];
        double q_[4];
        double qe_plus[3][9];
        double qe_minus[3][9];
        res_q_extra_t res_q_extra;

#ifdef EXTRA_CALC
        ax4_fwd_ret_t ax4_fwd_ret;
        ret = drive_to_joint_full(rob, q, th, &res_q_extra, &ax4_fwd_ret);
#else
        ret = drive_to_joint_full(rob, q, th, &res_q_extra);
#endif
        if (ret != 0) {
                return 1;
        }
        qe_nom[0] = q[0];
        qe_nom[1] = q[2];
        qe_nom[2] = res_q_extra.q23;
        qe_nom[3] = res_q_extra.qy;
        qe_nom[4] = res_q_extra.qx;
        qe_nom[5] = th[3];
        qe_nom[6] = q[1] + rob->q2_offs;
        qe_nom[7] = res_q_extra.alpha1;
        qe_nom[8] = res_q_extra.alpha2;

        memcpy(q_, q, 4 * sizeof(double));
        memset(gmat, 0, 9 * 4 * sizeof(double));
        gmat[0][0] = 1.0;
        if (rob->bh) {
                stop_ind = 4;
        } else {
                stop_ind = 3;
                gmat[5][3] = 1.0;
        }

        for (k = 1; k < stop_ind; k++) {
                q_[k] = q[k] + delta;
#ifdef EXTRA_CALC
                ret = drive_to_joint_full(rob, q_, th, &res_q_extra, &ax4_fwd_ret);
#else
                ret = drive_to_joint_full(rob, q_, th, &res_q_extra);
#endif
                if (ret != 0) {
                        return 1;
                }

                qe_plus[k - 1][0] = q_[0];
                qe_plus[k - 1][1] = q_[2];
                qe_plus[k - 1][2] = res_q_extra.q23;
                qe_plus[k - 1][3] = res_q_extra.qy;
                qe_plus[k - 1][4] = res_q_extra.qx;
                qe_plus[k - 1][5] = th[3];
                qe_plus[k - 1][6] = q_[1] + rob->q2_offs;
                qe_plus[k - 1][7] = res_q_extra.alpha1;
                qe_plus[k - 1][8] = res_q_extra.alpha2;
                if (extra_accurate > 0 || calc_gvec > 0) {
                        q_[k] = q[k] - delta;
#ifdef EXTRA_CALC
                        ret = drive_to_joint_full(rob, q_, th, &res_q_extra, &ax4_fwd_ret);
#else
                        ret = drive_to_joint_full(rob, q_, th, &res_q_extra);
#endif
                        if (ret != 0) {
                                return 1;
                        }
                        qe_minus[k - 1][0] = q_[0];
                        qe_minus[k - 1][1] = q_[2];
                        qe_minus[k - 1][2] = res_q_extra.q23;
                        qe_minus[k - 1][3] = res_q_extra.qy;
                        qe_minus[k - 1][4] = res_q_extra.qx;
                        qe_minus[k - 1][5] = th[3];
                        qe_minus[k - 1][6] = q_[1] + rob->q2_offs;
                        qe_minus[k - 1][7] = res_q_extra.alpha1;
                        qe_minus[k - 1][8] = res_q_extra.alpha2;
                        for (j = 0; j < 9; j++) {
                                gmat[j][k] = (qe_plus[k - 1][j]
                                        - qe_minus[k - 1][j]) / (2 * delta);
                        }
                } else {
                        for (j = 0; j < 9; j++) {
                                gmat[j][k] = (qe_plus[k - 1][j]
                                        - qe_nom[j]) / delta;
                        }
                }
                q_[k] = q[k];
        }

        if (calc_gvec > 0) {
                double delta2 = pow(delta, 2);
                double gacc[4][9][4];
                double gdot[9][4];
                double qe_[9];
                double dq, tmp_val;
                memset(gacc, 0, 4 * 9 * 4 * sizeof(double));
                memset(gdot, 0, 9 * 4 * sizeof(double));
                for (i = 1; i < stop_ind; i++) {
                        if (i > 0) {
                                for (j = 0; j < 9; j++) {
                                        gacc[i][j][i] += (qe_plus[i - 1][j] - 2 * qe_nom[j]
                                                + qe_minus[i - 1][j]) / delta2;
                                }
                        }
                        q_[i] = q[i] + delta;
                        for (k = i + 1; k < stop_ind; k++) {
                                q_[k] = q[k] + delta;
#ifdef EXTRA_CALC
                                ret = drive_to_joint_full(rob, q_, th, &res_q_extra, &ax4_fwd_ret);
#else
                                ret = drive_to_joint_full(rob, q_, th, &res_q_extra);
#endif
                                if (ret != 0) {
                                        return 1;
                                }
                                qe_[0] = q_[0];
                                qe_[1] = q_[2];
                                qe_[2] = res_q_extra.q23;
                                qe_[3] = res_q_extra.qy;
                                qe_[4] = res_q_extra.qx;
                                qe_[5] = th[3];
                                qe_[6] = q_[1] + rob->q2_offs;
                                qe_[7] = res_q_extra.alpha1;
                                qe_[8] = res_q_extra.alpha2;
                                for (j = 0; j < 9; j++) {
                                        if (i == 0) {
                                                tmp_val = qe_nom[j];
                                                if (j == 0) {
                                                        tmp_val += delta;
                                                }
                                        } else {
                                                tmp_val = qe_plus[i - 1][j];
                                        }
                                        dq = (qe_[j] - qe_plus[k - 1][j] - tmp_val
                                                + qe_nom[j]) / delta2;
                                        gacc[k][j][i] += dq;
                                        gacc[i][j][k] += dq;
                                }
                                q_[k] = q[k];
                        }
                        q_[i] = q[i];

                        /* gdot += gacc[i, :, :]*qd[i] */
                        for (j = 0; j < 9; j++) {
                                for (k = 0; k < 4; k++) {
                                        gdot[j][k] += gacc[i][j][k] * qd[i];
                                }
                        }
                }
                /* gvec = gdot*qd */
                for (j = 0; j < 9; j++) {
                        gvec[j] = 0;
                        for (k = 0; k < 4; k++) {
                                gvec[j] += gdot[j][k] * qd[k];
                        }
                }
        }
        return 0;
}


/*!
 * Inverse dynamics for the agile robot
 *
 * @param[in] rob Robot kinematics definition
 * @param[in] rob_lpoe LPOE robot definition
 * @param[in] dpar Dynamic parameters
 * @param[in] tool Dynamic parameters for tool/load
 * @param[in] q Joint angles
 * @param[in] qd Joint velocities
 * @param[in] qdd Joint accelerations
 * @param[in] g Acceleration of gravity
 * @param[in] full_model Set to 1 to use the full model including all coriolis/centrifugal forces terms
 * @param[out] trq Calculated joint torques
 */
int dyn_inv_agile(const struct agile_pkm_model* rob,
        const struct model_lpoe_agile_pkm* rob_lpoe,
        const struct dyn_params_link dpar[], const struct dyn_params_link* tool,
        const double q[4], const double qd[4], const double qdd[4],
        const double g[3], int full_model, double trq[4])
{
        double qe[9];
        double qed[9];
        double qedd[9];
        double gmat[9][4];
        double gvec[9];
        double trqe[9];
        int ret, j, k;

        double delta = 1e-6;
        /* The calculation of the g vector seems to be troublesome with a too small
           delta value. The problem is the ax4 kinematics which gives differences
           between python and C in the order of 1e-10, and as gvec is calculated
           from a approximation of a second derivative a too small delta tends to
           amplify this difference (there is a division with delta^2) */
        if (full_model > 0) {
                delta = 1e-4;
        }

        ret = calc_gmat(rob, q, delta, 0, full_model, qd, qe, gmat, gvec);
        if (ret != 0) {
                return 1;
        }

        for (k = 0; k < 9; k++) {
                qed[k] = 0;
                qedd[k] = 0;
                for (j = 0; j < 4; j++) {
                        qed[k] += gmat[k][j] * qd[j];
                        qedd[k] += gmat[k][j] * qdd[j];
                }
                if (full_model > 0) {
                        qedd[k] += gvec[k];
                }
        }

        ret = dyn_inv_agile_extended(rob_lpoe, dpar, tool, qe, qed, qedd, g, trqe);
        if (ret != 0) {
                return 1;
        }

        /* trq = Gu^(-T)*G^T*trqe */
        /* Gu = eye(4) */
        for (k = 0; k < 4; k++) {
                trq[k] = 0;
                for (j = 0; j < 9; j++) {
                        trq[k] += gmat[j][k] * trqe[j];
                }
        }
        return 0;
}


/*!
 * Friction model for the Agile PKM robot
 *
 * @param[in] fricpars Friction parameters
 * @param[in] qd Joint velocities
 * @param[out] trq Calculated joint torques
 */
void fric_trq_agile(const struct fricpars_joint fricpars[], const double qd[4], double trq[4])
{
        int k;
        double sgn_qd;

        for (k = 0; k < 4; k++) {
                if (k == 2) {
                        sgn_qd = tanh(qd[k] / 0.015);
                } else if (k == 0 || k == 1) {
                        sgn_qd = tanh(qd[k] / 0.0001);
                } else {
                        sgn_qd = 0.0;
                        if (qd[k] > 0) {
                                sgn_qd = 1.0;
                        } else if (qd[k] < 0) {
                                sgn_qd = -1.0;
                        }
                }
                trq[k] = sgn_qd * fricpars[k].coul + qd[k] * fricpars[k].visc;
        }
}


/*!
 * Calculate inertia matrix
 *
 * @param[in] rob Robot kinematics definition
 * @param[in] rob_lpoe LPOE robot definition
 * @param[in] dpar Dynamic parameters
 * @param[in] tool Dynamic parameters for tool/load
 * @param[in] q Joint angles
 * @param[out] imat Calculated inertia matrix
 */
int inertia_matrix(const struct agile_pkm_model* rob,
        const struct model_lpoe_agile_pkm* rob_lpoe,
        const struct dyn_params_link dpar[], const struct dyn_params_link* tool,
        const double q[4], double imat[4][4])
{
        int i, k;
        double qdd[4] = {0.0, 0.0, 0.0, 0.0};
        double qd[4] = {0.0, 0.0, 0.0, 0.0};
        double g[3] = {0.0, 0.0, 0.0};
        double tmp[4];
        int ret;

        for (k = 0; k < 4; k++) {
                qdd[k] = 1.0;
                ret = dyn_inv_agile(rob, rob_lpoe, dpar, tool, q, qd, qdd, g, 0, tmp);
                if (ret != 0) {
                        return ret;
                }
                for (i = 0; i < 4; i++) {
                        imat[i][k] = tmp[i];
                }
                qdd[k] = 0.0;
        }
        return 0;
}

/*!
 * Calculate inertia matrix
 *
 * @param[in] rob Robot kinematics definition
 * @param[in] rob_lpoe LPOE robot definition
 * @param[in] dpar Dynamic parameters
 * @param[in] tool Dynamic parameters for tool/load
 * @param[in] q Joint angles
 * @param[out] imat Calculated inertia matrix
 */
int inertia_matrix_new_joint_space(const struct agile_pkm_model* rob,
        const struct model_lpoe_agile_pkm* rob_lpoe,
        const struct dyn_params_link dpar[], const struct dyn_params_link* tool,
        const double th[4], double imat[4][4])
{
        int i, k;
        double thdd[4] = {0.0, 0.0, 0.0, 0.0};
        double thd[4] = {0.0, 0.0, 0.0, 0.0};
        double q[4] = {0.0};
        double qd[4] = {0.0};
        double qdd[4] = {0.0};
        double g[3] = {0.0, 0.0, 0.0};
        double tmp[4];
        int ret;

        for (k = 0; k < 4; k++) {
                thdd[k] = 1.0;
                ret = joint_to_drive_with_vel_acc(rob, th, thd, thdd, q, qd, qdd);
                if (ret)
                        return ret;
                ret = dyn_inv_agile(rob, rob_lpoe, dpar, tool, q, qd, qdd, g, 0, tmp);
                if (ret != 0) {
                        return ret;
                }
                for (i = 0; i < 4; i++) {
                        imat[i][k] = tmp[i];
                }
                thdd[k] = 0.0;
        }
        return 0;
}


/*!
 * Inverse dynamics for the extended agile robot
 *
 * Torque is calculated for all joints, also those that are unactuated.
 *
 * @param[in] rob LPOE robot definition
 * @param[in] dpar Dynamic parameters
 * @param[in] tool Dynamic parameters for tool/load
 * @param[in] q Joint angles
 * @param[in] qd Joint velocities
 * @param[in] qdd Joint accelerations
 * @param[in] g Acceleration of gravity
 * @param[out] trq Fwd kin t44s for each link in the robot
 */
int dyn_inv_agile_extended_grav(const struct model_lpoe_agile_pkm* rob,
        const struct dyn_params_link dpar[], const struct dyn_params_link* tool,
        const double q_in[9], const double g[3], double trq[9])
{
        int i, j, k, ind1;
        double tfs[9][4][4];
        double accs[11][3];
        double acc_cogs[11][3];
        double rs[11][3];
        double fs[11][3];
        double ts[11][3];

        double tmp[3];
        double tmp2[3];
        double tmp3[3];
        double fnew[3];
        double tnew[3];
        int prev;


        double q[9] = {q_in[0], q_in[1], q_in[2], q_in[3], q_in[4], q_in[5],
                q_in[6], q_in[7], q_in[8]};

        kin(rob, q, tfs);

        /* Initial values for outward recursion */
        accs[0][0] = -g[0];
        accs[0][1] = -g[1];
        accs[0][2] = -g[2];
        acc_cogs[0][0] = -g[0];
        acc_cogs[0][1] = -g[1];
        acc_cogs[0][2] = -g[2];
        memset(rs[0], 0, 3 * sizeof(double));


        for (k = 1; k < 10; k++) {
                prev = rob->lam[k - 1];

                if (k == rob->chain_ends[0]) {
                        for (j = 0; j < 3; j++) {
                                rs[k][j] = rob->last_trans[0][j][3];
                        }
                } else if (k == rob->chain_ends[1]) {
                        for (j = 0; j < 3; j++) {
                                rs[k][j] = rob->last_trans[1][j][3];
                        }
                } else {
                        for (j = 0; j < 3; j++) {
                                rs[k][j] = tfs[k][j][3];
                        }
                }

                memcpy(tmp, accs[prev], 3 * sizeof(double));
                transform_a2b(tfs[k - 1], tmp, tmp2);
                for (j = 0; j < 3; j++) {
                        accs[k][j] = tmp2[j];
                }

                for (j = 0; j < 3; j++) {
                        acc_cogs[k][j] = tmp2[j];
                }
        }

        /* tool/load handling */
        memcpy(accs[10], accs[rob->tool_body], 3 * sizeof(double));
        for (j = 0; j < 3; j++) {
                acc_cogs[k][j] = accs[10][j];
                fs[10][j] = tool->m * acc_cogs[k][j];
        }

        xprod(fs[10], tool->r, tmp);  /* tmp = cross(fnew, rcg_load) */
        for (j = 0; j < 3; j++) {
                ts[10][j] = tmp[j];
        }

        for (k = 9; k >= 0; k--) {
                for (j = 0; j < 3; j++) {
                        fnew[j] = dpar[k].m * acc_cogs[k][j];
                        tnew[j] = 0;
                }
                for (i = rob->mu_inds[k][0]; i < rob->mu_inds[k][1]; i++) {
                        ind1 = rob->mu[i];
                        transform_b2a(tfs[ind1 - 1], fs[ind1], tmp);  /* tmp = prev_force */
                        for (j = 0; j < 3; j++) {
                                fnew[j] += tmp[j];
                                tmp2[j] = dpar[k].r[j] - tfs[ind1 - 1][j][3];  /* tmp2 = rcg - r_star */
                        }
                        xprod(tmp, tmp2, tmp3);  /* tmp3 = cross(prev_force, rcg-r_star) */
                        transform_b2a(tfs[ind1 - 1], ts[ind1], tmp2);  /* tmp2 = prev_torque */
                        for (j = 0; j < 3; j++) {
                                tnew[j] += tmp2[j] + tmp3[j];
                        }
                }
                if (k == rob->tool_body) {
                        ind1 = -1;
                        for (i = 0; i < 2; i++) {
                                if (rob->chain_ends[i] == k) {
                                        ind1 = i;
                                        break;
                                }
                        }
                        if (ind1 >= 0) {
                                transform_b2a(rob->last_trans[ind1], fs[10], tmp2);  /* prev_w = prev_force */
                                for (j = 0; j < 3; j++) {
                                        tmp[j] = rob->last_trans[ind1][j][3];  /* tmp = r_star */
                                }
                        } else {
                                memcpy(tmp2, fs[10], 3 * sizeof(double));
                                memset(tmp, 0, 3 * sizeof(double));
                        }
                        for (j = 0; j < 3; j++) {
                                tmp3[j] = dpar[k].r[j] - tmp[j];  /* w_joint = rcg - r_star */
                        }
                        xprod(tmp2, tmp3, tmp);  /* wd_joint = cross(prev_force, rcg-r_star) */
                        if (ind1 >= 0) {
                                transform_b2a(rob->last_trans[ind1], ts[10], tmp3);  /* prev_wd = prev_torque */
                        } else {
                                memcpy(tmp3, ts[10], 3 * sizeof(double));
                        }
                        for (j = 0; j < 3; j++) {
                                fnew[j] += tmp2[j];
                                tnew[j] += tmp3[j] + tmp[j];
                        }
                }
                memcpy(fs[k], fnew, 3 * sizeof(double));
                xprod(fnew, dpar[k].r, tmp);  /* tmp = cross(fnew, rcg) */
                for (j = 0; j < 3; j++) {
                        ts[k][j] = tnew[j] - tmp[j];
                }
                if (k > 0) {
                        for (j = 0; j < 3; j++) {
                                tmp[j] = rob->lrob[k - 1].s[j + 3];  /* tmp = rot_axis */
                        }
                        trq[k - 1] = mul3_3(tmp, ts[k]);
                }
        }
        return 0;
}


/*!
 * Calculate the matrix G (and possibly the vector g)
 *
 * G = df/dq where f = f(q) is a vector of all relevant angles for the lpoe model
 *
 * ydot = G*qdot
 * ydotdot = G*qdotdot + g  ( g = Gdot*qdot )
 *
 * The calculation of G (and g) is via numerical differentiation. The input
 * extra_accurate controls if a one-sided difference quotient (extra_accurate=0)
 * or a two-sided (extra_accurate=1) difference quotient is used. The accuracy is
 * improved with a two-sided difference quotient, but the computational burden
 * is increased.
 *
 * The vector g is only calculated if the input calc_gvec is 1.
 *
 * @param[in] rob Robot kinematics definition
 * @param[in] q Joint angles
 * @param[in] delta Step size used for numerical differentiation
 * @param[in] extra_accurate Set to 1 to use two-sided difference quotient instead of one-sided
 * @param[in] calc_gvec Set to 1 to calculate the vector g
 * @param[in] qd Joint velocities
 * @param[out] qe_nom Extended joint vector
 * @param[out] gmat The matrix G
 * @param[out] gvec The vector g (note that this will only be filled with values if calc_gvec is 1)
 */
int calc_gmat_grav(const struct agile_pkm_model* rob, const double q[4],
        double delta, double qe_nom[9], double gmat[9][4])
{
        int ret, j, k, stop_ind;
        double q_[4];
        double th[4];
        double qe_plus[3][9];
        res_q_extra_t res_q_extra;

#ifdef EXTRA_CALC
        ax4_fwd_ret_t ax4_fwd_ret;
        ret = drive_to_joint_full(rob, q, th, &res_q_extra, &ax4_fwd_ret);
#else
        ret = drive_to_joint_full(rob, q, th, &res_q_extra);
#endif
        if (ret != 0) {
                return 2;
        }
        qe_nom[0] = q[0];
        qe_nom[1] = q[2];
        qe_nom[2] = res_q_extra.q23;
        qe_nom[3] = res_q_extra.qy;
        qe_nom[4] = res_q_extra.qx;
        qe_nom[5] = th[3];
        qe_nom[6] = q[1] + rob->q2_offs;
        qe_nom[7] = res_q_extra.alpha1;
        qe_nom[8] = res_q_extra.alpha2;

        memcpy(q_, q, 4 * sizeof(double));
        memset(gmat, 0, 9 * 4 * sizeof(double));
        gmat[0][0] = 1.0;
        if (rob->bh) {
                stop_ind = 4;
        } else {
                stop_ind = 3;
                gmat[5][3] = 1.0;
        }

        for (k = 1; k < stop_ind; k++) {
                q_[k] = q[k] + delta;
#ifdef EXTRA_CALC
                ret = drive_to_joint_full(rob, q_, th, &res_q_extra, &ax4_fwd_ret);
#else
                ret = drive_to_joint_full(rob, q_, th, &res_q_extra);
#endif
                if (ret != 0) {
                        return 1;
                }

                qe_plus[k - 1][0] = q_[0];
                qe_plus[k - 1][1] = q_[2];
                qe_plus[k - 1][2] = res_q_extra.q23;
                qe_plus[k - 1][3] = res_q_extra.qy;
                qe_plus[k - 1][4] = res_q_extra.qx;
                qe_plus[k - 1][5] = th[3];
                qe_plus[k - 1][6] = q_[1] + rob->q2_offs;
                qe_plus[k - 1][7] = res_q_extra.alpha1;
                qe_plus[k - 1][8] = res_q_extra.alpha2;
                for (j = 0; j < 9; j++) {
                        gmat[j][k] = (qe_plus[k - 1][j]
                                - qe_nom[j]) / delta;
                }
                q_[k] = q[k];
        }
        return 0;
}


/*!
 * Inverse dynamics for the agile robot
 *
 * @param[in] rob Robot kinematics definition
 * @param[in] rob_lpoe LPOE robot definition
 * @param[in] dpar Dynamic parameters
 * @param[in] tool Dynamic parameters for tool/load
 * @param[in] q Joint angles
 * @param[in] qd Joint velocities
 * @param[in] qdd Joint accelerations
 * @param[in] g Acceleration of gravity
 * @param[in] full_model Set to 1 to use the full model including all coriolis/centrifugal forces terms
 * @param[out] trq Calculated joint torques
 */
int dyn_inv_agile_grav(const struct agile_pkm_model* rob,
        const struct model_lpoe_agile_pkm* rob_lpoe,
        const struct dyn_params_link dpar[], const struct dyn_params_link* tool,
        const double q[4], const double g[3], double trq[4])
{
        double qe[9];
        double gmat[9][4];
        double trqe[9];
        int ret, j, k;

        double delta = 1e-6;

        ret = calc_gmat_grav(rob, q, delta, qe, gmat);
        if (ret != 0) {
                return ret;
        }

        ret = dyn_inv_agile_extended_grav(rob_lpoe, dpar, tool, qe, g, trqe);
        if (ret != 0) {
                return ret;
        }

        /* trq = Gu^(-T)*G^T*trqe */
        /* Gu = eye(4) */
        for (k = 0; k < 4; k++) {
                trq[k] = 0;
                for (j = 0; j < 9; j++) {
                        trq[k] += gmat[j][k] * trqe[j];
                }
        }
        return 0;
}

/*!
 * Inverse dynamics for the extended agile robot
 *
 * Torque is calculated for all joints, also those that are unactuated.
 *
 * @param[in] rob LPOE robot definition
 * @param[in] dpar Dynamic parameters
 * @param[in] tool Dynamic parameters for tool/load
 * @param[in] q Joint angles
 * @param[in] qd Joint velocities
 * @param[in] qdd Joint accelerations
 * @param[in] g Acceleration of gravity
 * @param[out] trq Fwd kin t44s for each link in the robot
 */
int dyn_inv_agile_extended_granular(const struct model_lpoe_agile_pkm* rob,
        const struct dyn_params_link dpar[], const struct dyn_params_link* tool,
        const double q[9], const double qd[9], const double qdd_vel[9], const double qdd_acc[9],
        const double g[3], double trq_grav[9], double trq_vel[9], double trq_acc[9])
{
        int i, j, k, ind1, ind2;
        double tfs[9][4][4];
        double ws_vel[11][3];
        double wds_vel[11][3];
        double wds_acc[11][3];
        double accs_grav[11][3];
        double accs_vel[11][3];
        double acc_cogs_vel[11][3];
        double accs_acc[11][3];
        double acc_cogs_acc[11][3];
        double rs[11][3];
        double fs_grav[11][3];
        double ts_grav[11][3];
        double fs_vel[11][3];
        double ts_vel[11][3];
        double fs_acc[11][3];
        double ts_acc[11][3];

        double w_joint_vel[3];
        double wd_joint_vel[3];
        double wd_joint_acc[3];
        double prev_w_vel[3];
        double prev_wd_vel[3];
        double prev_wd_acc[3];
        double tmp1[3];
        double tmp2[3];
        double tmp3[3];
        double fnew_grav[3];
        double fnew_vel[3];
        double fnew_acc[3];
        double tnew_grav[3];
        double tnew_vel[3];
        double tnew_acc[3];
        int prev;

        kin(rob, q, tfs);

        /* Initial values for outward recursion */
        memset(ws_vel[0], 0, 3 * sizeof(double));
        memset(wds_vel[0], 0, 3 * sizeof(double));
        memset(wds_acc[0], 0, 3 * sizeof(double));
        accs_grav[0][0] = -g[0];
        accs_grav[0][1] = -g[1];
        accs_grav[0][2] = -g[2];
        memset(accs_vel[0], 0, 3 * sizeof(double));
        memset(acc_cogs_vel[0], 0, 3 * sizeof(double));
        memset(accs_acc[0], 0, 3 * sizeof(double));
        memset(acc_cogs_acc[0], 0, 3 * sizeof(double));
        memset(rs[0], 0, 3 * sizeof(double));


        for (k = 1; k < 10; k++) {
                for (j = 0; j < 3; j++) {
                        w_joint_vel[j] = qd[k - 1] * rob->lrob[k - 1].s[3 + j];
                        wd_joint_vel[j] = qdd_vel[k - 1] * rob->lrob[k - 1].s[3 + j];
                        wd_joint_acc[j] = qdd_acc[k - 1] * rob->lrob[k - 1].s[3 + j];
                }
                prev = rob->lam[k - 1];
                transform_a2b(tfs[k - 1], ws_vel[prev], prev_w_vel);
                transform_a2b(tfs[k - 1], wds_vel[prev], prev_wd_vel);
                transform_a2b(tfs[k - 1], wds_acc[prev], prev_wd_acc);
                xprod(prev_w_vel, w_joint_vel, tmp1);
                for (j = 0; j < 3; j++) {
                        ws_vel[k][j] = prev_w_vel[j] + w_joint_vel[j];
                        wds_vel[k][j] = prev_wd_vel[j] + wd_joint_vel[j] + tmp1[j];
                        wds_acc[k][j] = prev_wd_acc[j] + wd_joint_acc[j];
                }

                if (k == rob->chain_ends[0]) {
                        for (j = 0; j < 3; j++) {
                                rs[k][j] = rob->last_trans[0][j][3];
                        }
                } else if (k == rob->chain_ends[1]) {
                        for (j = 0; j < 3; j++) {
                                rs[k][j] = rob->last_trans[1][j][3];
                        }
                } else {
                        for (j = 0; j < 3; j++) {
                                rs[k][j] = tfs[k][j][3];
                        }
                }

                memcpy(tmp1, accs_grav[prev], 3 * sizeof(double));
                memcpy(tmp2, accs_vel[prev], 3 * sizeof(double));
                memcpy(tmp3, accs_acc[prev], 3 * sizeof(double));
                i = rob->mu_inds[prev][1] - rob->mu_inds[prev][0];
                if (i > 1) {
                        ind2 = -1;
                        for (j = 0; j < i; j++) {
                                ind1 = rob->mu_inds[prev][0] + j;
                                if (k == rob->mu[ind1]) {
                                        ind2 = rob->mu[ind1] - 1;
                                        break;
                                }
                        }
                        if (ind2 < 0) {
                                return 1;
                        }
                        if (ind2 != prev) {
                                /* Use w_joint, wd_joint, prev_w, prev_wd as temp variables */
                                for (j = 0; j < 3; j++) {
                                        /* w_joint = r_star_prev */
                                        w_joint_vel[j] = tfs[ind2][j][3] - rs[prev][j];
                                }
                                xprod(wds_vel[prev], w_joint_vel, wd_joint_vel);  /* wd_joint = cross(wds[prev], r_star_prev) */
                                xprod(wds_acc[prev], w_joint_vel, wd_joint_acc);  /* wd_joint = cross(wds[prev], r_star_prev) */
                                xprod(ws_vel[prev], w_joint_vel, prev_w_vel);  /* prev_w = cross(ws[prev], r_star_prev) */
                                xprod(ws_vel[prev], prev_w_vel, prev_wd_vel);  /* prev_wd = cross(ws[prev], cross(ws[prev],  r_star_prev)) */
                                for (j = 0; j < 3; j++) {
                                        tmp2[j] = accs_vel[prev][j] + wd_joint_vel[j] + prev_wd_vel[j];
                                        tmp3[j] = accs_acc[prev][j] + wd_joint_acc[j];
                                }
                        }
                }
                /* Some temp variable shenanigans: tmp1 = rot_matrix*tmp1 etc.*/
                transform_a2b(tfs[k - 1], tmp1, prev_wd_vel);
                transform_a2b(tfs[k - 1], tmp2, tmp1);
                transform_a2b(tfs[k - 1], tmp3, tmp2);
                memcpy(tmp3, tmp2, 3 * sizeof(double));
                memcpy(tmp2, tmp1, 3 * sizeof(double));
                memcpy(tmp1, prev_wd_vel, 3 * sizeof(double));

                xprod(wds_vel[k], rs[k], wd_joint_vel);  /* wd_joint = cross(wds[k], r_star) */
                xprod(wds_acc[k], rs[k], wd_joint_acc);  /* wd_joint = cross(wds[k], r_star) */
                xprod(ws_vel[k], rs[k], prev_w_vel);  /* prev_w = cross(ws[k], r_star) */
                xprod(ws_vel[k], prev_w_vel, prev_wd_vel);  /* prev_wd_vel = cross(ws[k], cross(ws[k], r_star)) */
                for (j = 0; j < 3; j++) {
                        accs_grav[k][j] = tmp1[j];
                        accs_vel[k][j] = tmp2[j] + wd_joint_vel[j] + prev_wd_vel[j];
                        accs_acc[k][j] = tmp3[j] + wd_joint_acc[j];
                }

                xprod(wds_vel[k], dpar[k].r, wd_joint_vel);  /* wd_joint = cross(wds[k], rcg) */
                xprod(wds_acc[k], dpar[k].r, wd_joint_acc);  /* wd_joint = cross(wds[k], rcg) */
                xprod(ws_vel[k], dpar[k].r, prev_w_vel);  /* prev_w = cross(ws[k], rcg) */
                xprod(ws_vel[k], prev_w_vel, prev_wd_vel);  /* prev_wd_vel = cross(ws[k], cross(ws[k], rcg)) */
                for (j = 0; j < 3; j++) {
                        acc_cogs_vel[k][j] = tmp2[j] + wd_joint_vel[j] + prev_wd_vel[j];
                        acc_cogs_acc[k][j] = tmp3[j] + wd_joint_acc[j];
                }
        }

        /* tool/load handling */
        memcpy(ws_vel[10], ws_vel[rob->tool_body], 3 * sizeof(double));
        memcpy(wds_vel[10], wds_vel[rob->tool_body], 3 * sizeof(double));
        memcpy(wds_acc[10], wds_acc[rob->tool_body], 3 * sizeof(double));
        memcpy(accs_grav[10], accs_grav[rob->tool_body], 3 * sizeof(double));
        memcpy(accs_vel[10], accs_vel[rob->tool_body], 3 * sizeof(double));
        memcpy(accs_acc[10], accs_acc[rob->tool_body], 3 * sizeof(double));
        xprod(wds_vel[10], tool->r, wd_joint_vel);  /* wd_joint = cross(wds[10], rcg_load) */
        xprod(wds_acc[10], tool->r, wd_joint_acc);  /* wd_joint = cross(wds[10], rcg_load) */
        xprod(ws_vel[10], tool->r, prev_w_vel);  /* prev_w = cross(ws[10], rcg_load) */
        xprod(ws_vel[10], prev_w_vel, tmp2);  /* tmp = cross(ws[10], cross(ws[10], rcg_load)) */
        for (j = 0; j < 3; j++) {
                acc_cogs_vel[k][j] = accs_vel[10][j] + wd_joint_vel[j] + tmp2[j];
                acc_cogs_acc[k][j] = accs_acc[10][j] + wd_joint_acc[j];
                fs_grav[10][j] = tool->m * accs_grav[k][j];
                fs_vel[10][j] = tool->m * acc_cogs_vel[k][j];
                fs_acc[10][j] = tool->m * acc_cogs_acc[k][j];
        }

        xprod(fs_grav[10], tool->r, tmp1);  /* tmp = cross(fnew, rcg_load) */
        xprod(fs_vel[10], tool->r, tmp2);  /* tmp = cross(fnew, rcg_load) */
        xprod(fs_acc[10], tool->r, tmp3);  /* tmp = cross(fnew, rcg_load) */
        mul33_3(tool->ival, wds_vel[10], prev_wd_vel);  /* prev_wd = i_load*wds[10] */
        mul33_3(tool->ival, wds_acc[10], prev_wd_acc);  /* prev_wd = i_load*wds[10] */
        mul33_3(tool->ival, ws_vel[10], prev_w_vel);  /* prev_w = i_load*ws[10] */
        xprod(ws_vel[10], prev_w_vel, wd_joint_vel);  /* wd_joint = cross(ws[10], i_load*ws[10]) */
        for (j = 0; j < 3; j++) {
                ts_grav[10][j] = -tmp1[j];
                ts_vel[10][j] = -tmp2[j] + prev_wd_vel[j] + wd_joint_vel[j];
                ts_acc[10][j] = -tmp3[j] + prev_wd_acc[j];
        }

        for (k = 9; k >= 0; k--) {
                mul33_3(dpar[k].ival, wds_vel[k], tmp2);  /* tmp = imat*wds[k] */
                mul33_3(dpar[k].ival, wds_acc[k], tmp3);  /* tmp = imat*wds[k] */
                mul33_3(dpar[k].ival, ws_vel[k], w_joint_vel);  /* w_joint = imat*ws[k] */
                xprod(ws_vel[k], w_joint_vel, wd_joint_vel);  /* wd_joint = cross(ws[k], imat*ws[k]) */
                for (j = 0; j < 3; j++) {
                        fnew_grav[j] = dpar[k].m * accs_grav[k][j];
                        fnew_vel[j] = dpar[k].m * acc_cogs_vel[k][j];
                        fnew_acc[j] = dpar[k].m * acc_cogs_acc[k][j];
                        tnew_grav[j] = 0.0;
                        tnew_vel[j] = tmp2[j] + wd_joint_vel[j];
                        tnew_acc[j] = tmp3[j];
                }
                for (i = rob->mu_inds[k][0]; i < rob->mu_inds[k][1]; i++) {
                        ind1 = rob->mu[i];
                        /* Split calculations up here to be able to reuse temp variables */
                        /* grav */
                        transform_b2a(tfs[ind1 - 1], fs_grav[ind1], tmp1);  /* tmp1 = prev_force */
                        for (j = 0; j < 3; j++) {
                                fnew_grav[j] += tmp1[j];
                                tmp2[j] = dpar[k].r[j] - tfs[ind1 - 1][j][3]; /* tmp2 = rcg - r_star*/
                        }
                        xprod(tmp1, tmp2, tmp3); /* tmp3 = cross(prev_force, rcg-r_star) */
                        transform_b2a(tfs[ind1 - 1], ts_grav[ind1], tmp1);  /* tmp1 = prev_torque */
                        for (j = 0; j < 3; j++) {
                                tnew_grav[j] += tmp1[j] + tmp3[j];
                        }

                        /* vel */
                        transform_b2a(tfs[ind1 - 1], fs_vel[ind1], tmp1);  /* tmp1 = prev_force */
                        for (j = 0; j < 3; j++) {
                                fnew_vel[j] += tmp1[j];
                        }
                        xprod(tmp1, tmp2, tmp3); /* tmp3 = cross(prev_force, rcg-r_star) */
                        transform_b2a(tfs[ind1 - 1], ts_vel[ind1], tmp1);  /* tmp1 = prev_torque */
                        for (j = 0; j < 3; j++) {
                                tnew_vel[j] += tmp1[j] + tmp3[j];
                        }

                        /* acc */
                        transform_b2a(tfs[ind1 - 1], fs_acc[ind1], tmp1);  /* tmp1 = prev_force */
                        for (j = 0; j < 3; j++) {
                                fnew_acc[j] += tmp1[j];
                        }
                        xprod(tmp1, tmp2, tmp3); /* tmp3 = cross(prev_force, rcg-r_star) */
                        transform_b2a(tfs[ind1 - 1], ts_acc[ind1], tmp1);  /* tmp1 = prev_torque */
                        for (j = 0; j < 3; j++) {
                                tnew_acc[j] += tmp1[j] + tmp3[j];
                        }
                }
                if (k == rob->tool_body) {
                        ind1 = -1;
                        for (i = 0; i < 2; i++) {
                                if (rob->chain_ends[i] == k) {
                                        ind1 = i;
                                        break;
                                }
                        }
                        /* grav */
                        if (ind1 >= 0) {
                                transform_b2a(rob->last_trans[ind1], fs_grav[10], prev_w_vel);  /* prev_w = prev_force */
                                transform_b2a(rob->last_trans[ind1], ts_grav[10], prev_wd_vel);  /* prev_wd = prev_torque */
                                for (j = 0; j < 3; j++) {
                                        tmp1[j] = rob->last_trans[ind1][j][3];  /* tmp = r_star */
                                }
                        } else {
                                memcpy(prev_w_vel, fs_grav[10], 3 * sizeof(double));
                                memcpy(prev_wd_vel, ts_grav[10], 3 * sizeof(double));
                                memset(tmp1, 0, 3 * sizeof(double));
                        }
                        for (j = 0; j < 3; j++) {
                                w_joint_vel[j] = dpar[k].r[j] - tmp1[j];  /* w_joint = rcg - r_star */
                        }
                        xprod(prev_w_vel, w_joint_vel, wd_joint_vel);  /* wd_joint = cross(prev_force, rcg-r_star) */
                        for (j = 0; j < 3; j++) {
                                fnew_grav[j] += prev_w_vel[j];
                                tnew_grav[j] += prev_wd_vel[j] + wd_joint_vel[j];
                        }

                        /* vel */
                        if (ind1 >= 0) {
                                transform_b2a(rob->last_trans[ind1], fs_vel[10], prev_w_vel);  /* prev_w = prev_force */
                                transform_b2a(rob->last_trans[ind1], ts_vel[10], prev_wd_vel);  /* prev_wd = prev_torque */
                                for (j = 0; j < 3; j++) {
                                        tmp1[j] = rob->last_trans[ind1][j][3];  /* tmp = r_star */
                                }
                        } else {
                                memcpy(prev_w_vel, fs_vel[10], 3 * sizeof(double));
                                memcpy(prev_wd_vel, ts_vel[10], 3 * sizeof(double));
                                memset(tmp1, 0, 3 * sizeof(double));
                        }
                        for (j = 0; j < 3; j++) {
                                w_joint_vel[j] = dpar[k].r[j] - tmp1[j];  /* w_joint = rcg - r_star */
                        }
                        xprod(prev_w_vel, w_joint_vel, wd_joint_vel);  /* wd_joint = cross(prev_force, rcg-r_star) */
                        for (j = 0; j < 3; j++) {
                                fnew_vel[j] += prev_w_vel[j];
                                tnew_vel[j] += prev_wd_vel[j] + wd_joint_vel[j];
                        }

                        /* acc */
                        if (ind1 >= 0) {
                                transform_b2a(rob->last_trans[ind1], fs_acc[10], prev_w_vel);  /* prev_w = prev_force */
                                transform_b2a(rob->last_trans[ind1], ts_acc[10], prev_wd_vel);  /* prev_wd = prev_torque */
                                for (j = 0; j < 3; j++) {
                                        tmp1[j] = rob->last_trans[ind1][j][3];  /* tmp = r_star */
                                }
                        } else {
                                memcpy(prev_w_vel, fs_acc[10], 3 * sizeof(double));
                                memcpy(prev_wd_vel, ts_acc[10], 3 * sizeof(double));
                                memset(tmp1, 0, 3 * sizeof(double));
                        }
                        for (j = 0; j < 3; j++) {
                                w_joint_vel[j] = dpar[k].r[j] - tmp1[j];  /* w_joint = rcg - r_star */
                        }
                        xprod(prev_w_vel, w_joint_vel, wd_joint_vel);  /* wd_joint = cross(prev_force, rcg-r_star) */
                        for (j = 0; j < 3; j++) {
                                fnew_acc[j] += prev_w_vel[j];
                                tnew_acc[j] += prev_wd_vel[j] + wd_joint_vel[j];
                        }
                }
                memcpy(fs_grav[k], fnew_grav, 3 * sizeof(double));
                memcpy(fs_vel[k], fnew_vel, 3 * sizeof(double));
                memcpy(fs_acc[k], fnew_acc, 3 * sizeof(double));
                xprod(fnew_grav, dpar[k].r, tmp1);  /* tmp = cross(fnew, rcg) */
                xprod(fnew_vel, dpar[k].r, tmp2);  /* tmp = cross(fnew, rcg) */
                xprod(fnew_acc, dpar[k].r, tmp3);  /* tmp = cross(fnew, rcg) */
                for (j = 0; j < 3; j++) {
                        ts_grav[k][j] = tnew_grav[j] - tmp1[j];
                        ts_vel[k][j] = tnew_vel[j] - tmp2[j];
                        ts_acc[k][j] = tnew_acc[j] - tmp3[j];
                }
                if (k > 0) {
                        for (j = 0; j < 3; j++) {
                                tmp1[j] = rob->lrob[k - 1].s[j + 3];  /* tmp = rot_axis */
                        }
                        trq_grav[k - 1] = mul3_3(tmp1, ts_grav[k]);
                        trq_vel[k - 1] = mul3_3(tmp1, ts_vel[k]);
                        trq_acc[k - 1] = mul3_3(tmp1, ts_acc[k]);
                }
        }
        return 0;
}


/*!
 * Inverse dynamics for the agile robot
 *
 * @param[in] rob Robot kinematics definition
 * @param[in] rob_lpoe LPOE robot definition
 * @param[in] dpar Dynamic parameters
 * @param[in] tool Dynamic parameters for tool/load
 * @param[in] q Joint angles
 * @param[in] qd Joint velocities
 * @param[in] qdd Joint accelerations
 * @param[in] g Acceleration of gravity
 * @param[in] full_model Set to 1 to use the full model including all coriolis/centrifugal forces terms
 * @param[out] trq Calculated joint torques
 */
int dyn_inv_agile_granular(const struct agile_pkm_model* rob,
        const struct model_lpoe_agile_pkm* rob_lpoe,
        const struct dyn_params_link dpar[], const struct dyn_params_link* tool,
        const double q[4], const double qd[4], const double qdd[4],
        const double g[3], int full_model, double trq_grav[4], double trq_vel[4], double trq_acc[4])
{
        double qe[9];
        double qed[9];
        double qedd_vel[9];
        double qedd_acc[9];
        double gmat[9][4];
        double gvec[9];
        double trqe_grav[9];
        double trqe_vel[9];
        double trqe_acc[9];
        int ret, j, k;

        double delta = 1e-6;
        /* The calculation of the g vector seems to be troublesome with a too small
           delta value. The problem is the ax4 kinematics which gives differences
           between python and C in the order of 1e-10, and as gvec is calculated
           from a approximation of a second derivative a too small delta tends to
           amplify this difference (there is a division with delta^2) */
        if (full_model > 0) {
                delta = 1e-4;
        }

        ret = calc_gmat(rob, q, delta, 0, full_model, qd, qe, gmat, gvec);
        if (ret != 0) {
                return 1;
        }

        for (k = 0; k < 9; k++) {
                qed[k] = 0;
                qedd_vel[k] = 0;
                qedd_acc[k] = 0;
                for (j = 0; j < 4; j++) {
                        qed[k] += gmat[k][j] * qd[j];
                        qedd_acc[k] += gmat[k][j] * qdd[j];
                }
                if (full_model > 0) {
                        qedd_vel[k] += gvec[k];
                }
        }

        ret = dyn_inv_agile_extended_granular(rob_lpoe, dpar, tool, qe, qed, qedd_vel, qedd_acc, g, trqe_grav, trqe_vel, trqe_acc);
        if (ret != 0) {
                return 1;
        }

        /* trq = Gu^(-T)*G^T*trqe */
        /* Gu = eye(4) */
        for (k = 0; k < 4; k++) {
                trq_grav[k] = 0;
                trq_vel[k] = 0;
                trq_acc[k] = 0;
                for (j = 0; j < 9; j++) {
                        trq_grav[k] += gmat[j][k] * trqe_grav[j];
                        trq_vel[k] += gmat[j][k] * trqe_vel[j];
                        trq_acc[k] += gmat[j][k] * trqe_acc[j];
                }
        }
        return 0;
}