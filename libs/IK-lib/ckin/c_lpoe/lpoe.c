/*
 * Copyright (C) 2017, 2018, 2019 Cognibotics
 *
 * Use or copying of this file or any part of it, without explicit
 * permission from Cognibotics is not allowed in any way.
 */
/*!
 * @file
 * @brief LPOE-utils
 */
#ifdef CB_SYSTEM_HEADER
#include CB_SYSTEM_HEADER
#else
#include <stdlib.h>
#include <string.h>
#include <math.h>
#endif

#include "lpoe.h"
#include "matrix_math.h"

/* Allow code to work without VLAs and dynamic memory allocation. */
#ifdef LPOE_MAX_N_SCREWS
#define N_SCREWS LPOE_MAX_N_SCREWS
#else
#define N_SCREWS n_screws  /*!< Parameter in relevant scopes. */
#endif

#ifdef LPOE_MAX_N_JOINTS
#define N_JOINTS LPOE_MAX_N_JOINTS
#else
#define N_JOINTS n_joints  /*!< Parameter in relevant scopes. */
#endif


static void handle_coupling_offs(const struct model_lpoe *model, const double th_in[],
		int end_joint, double th[]);

/*!
 * Function to initialize lpoe model with default parameters
 *
 * @param[in] model Pointer to a lpoe model object to initialize
 */
void lpoe_init(struct model_lpoe *model)
{
	model->n_screw = 0;
	model->lrob = NULL;
	model->q_a_offs = NULL;
	model->has_pbar = 0;
	model->par_fact = 1.0;
	model->n_joints = 0;
	model->coupling_matrix = NULL;
	model->inverse_coupling_matrix = NULL;
	model->lrob_pbar = NULL;
}


/*!
 * @param[in] twist Pointer to a twist.
 * return Pointer to the start of the omega triple.
 */
const double *twist_omega(const double twist[6])
{
	return &twist[3];
}
/*!
 * @param[in] twist Pointer to a twist.
 * return Pointer to the start of the omega triple.
 */
double *twist_omega_w(double twist[6])
{
	return &twist[3];
}

/*!
 * @param[in] twist Pointer to a twist.
 * return Pointer to the start of the v triple.
 */
const double *twist_v(const double twist[6])
{
	return &twist[0];
}
/*!
 * @param[in] twist Pointer to a twist.
 * return Pointer to the start of the v triple.
 */
double *twist_v_w(double twist[6])
{
	return &twist[0];
}

/*!
 * @param[in] t Pointer to a twist.
 * @param[out] out The corresponding rotation matrix.
 */
void twist_omega_hat(const double t[6], double out[3][3])
{
	out[0][0] = 0;
	out[0][1] = -twist_omega(t)[2];
	out[0][2] = twist_omega(t)[1];
	out[1][0] = twist_omega(t)[2];
	out[1][1] = 0;
	out[1][2] = -twist_omega(t)[0];
	out[2][0] = -twist_omega(t)[1];
	out[2][1] = twist_omega(t)[0];
	out[2][2] = 0;
}

/*!
 * Go from a twist represented as 4x4-matrix to vector representation
 *
 * Reference: cb.operator.vee
 *
 * @param[in] that Twist represented as 4x4 matrix.
 * @param[out] out The corresponding twist in vector representation.
 */
void vee(const double that[4][4], double out[6])
{
	out[0] = that[0][3];
	out[1] = that[1][3];
	out[2] = that[2][3];
	out[3] = that[2][1];
	out[4] = that[0][2];
	out[5] = that[1][0];
}

/*!
 * @param[in] t Pointer to a twist.
 * @param[in] theta An angle in radians.
 * @param[out] out33 Pointer to a 3x3 matrix.
 */
void twist_R(const double t[6], double theta, double out33[3][3])
{
	double omega_hat[3][3];
	double norm_h;
	int i, j;

	twist_omega_hat(t, omega_hat);
	norm_h = sqrt(pow(t[3], 2) + pow(t[4], 2) + pow(t[5], 2));
	if (norm_h == 0) {
		eye(3, (double *) out33);
	} else {
		double hn[3][3];
		double hn2[3][3];
		double t1[3][3] = {0};
		double t2[3][3] = {0};

		for (i = 0; i < 3; i++)
			for (j = 0; j < 3; j++)
				hn[i][j] = 1.0/norm_h * omega_hat[i][j];
		mul33(hn, hn, hn2);

		/* Compute intermediate value */
		for (i = 0; i < 3; i++)
			for (j = 0; j < 3; j++)
				hn[i][j] = 1.0/norm_h * omega_hat[i][j];

		/* Compute matrix term 2 */
		for (i = 0; i < 3; i++)
			for (j = 0; j < 3; j++)
				t1[i][j] = sin(theta*norm_h) * hn[i][j];

		/* Compute matrix term 3 */
		for (i = 0; i < 3; i++)
			for (j = 0; j < 3; j++)
				t2[i][j] = (1-cos(theta*norm_h)) * hn2[i][j];

		/* Add terms */
		eye(3, (double *) out33);
		for (i = 0; i < 3; i++)
			for (j = 0; j < 3; j++)
				out33[i][j] += t1[i][j] + t2[i][j];
	}
}

/*!
 * @param[in] t Pointer to a twist.
 * @param[in] theta An angle in radians (or distance in m if prismatic twist).
 * @param[out] out Pointer to a 3x3 matrix.
 */
void twist_exp_hat(const double t[6], double theta, double out[4][4])
{
	memset(out, 0, 4*4*sizeof(double));
	if (fabs(twist_omega(t)[0]) + fabs(twist_omega(t)[1]) + fabs(twist_omega(t)[2]) == 0) {
		eye(4, (double *) out);
		out[0][3] = theta * twist_v(t)[0];
		out[1][3] = theta * twist_v(t)[1];
		out[2][3] = theta * twist_v(t)[2];
	} else {
		double r[3][3] = {0};
		double omh[3][3];
		double omh2[3][3];
		double om;
		double b1_term1[3][3] = {0};
		double b1_term2[3][3];
		double b1_term3[3][3];
		double b1[3][3];
		double b[3];
		int i, j;

		twist_R(t, theta, r);
		twist_omega_hat(t, omh);
		om = sqrt(pow(twist_omega(t)[0], 2) +
				pow(twist_omega(t)[1], 2) +
				pow(twist_omega(t)[2], 2));

		mul33(omh, omh, omh2);

		/* Compute matrix b1_term 1 */
		for (i = 0; i < 3; i++) {
			b1_term1[i][i] = theta;
		}

		/* Compute matrix b1_term 2 */
		for (i = 0; i < 3; i++)
			for (j = 0; j < 3; j++)
				b1_term2[i][j] = ((1.0-cos(om*theta))
						/ pow(om, 2)) * omh[i][j];

		/* Compute matrix b1_term 3 */
		for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			b1_term3[i][j] = ((om*theta - sin(om*theta))
					/ pow(om, 3)) * omh2[i][j];

		/* Compute first factor of b. */
		for (i = 0; i < 3; i++)
			for (j = 0; j < 3; j++)
				b1[i][j] = b1_term1[i][j] + b1_term2[i][j]
						+ b1_term3[i][j];

		/* Compute b */
		for (i = 0; i < 3; i++) {
		b[i] = 0;
			for (j = 0; j < 3; j++)
				b[i] += b1[i][j] * twist_v(t)[j];
		}

		/* Put result together. */
		for (i = 0; i < 3; i++)
			for (j = 0; j < 3; j++)
				out[i][j] = r[i][j];

		out[0][3] = b[0];
		out[1][3] = b[1];
		out[2][3] = b[2];
		out[3][0] = 0;
		out[3][1] = 0;
		out[3][2] = 0;
		out[3][3] = 1;
	}
}

/*!
 * hat operator for vector
 *
 * Reference: cb.operator.hat
 *
 * @param[in] a First value
 * @param[in] b Second value
 * @param[in] c Third value
 * @param[out] out The "hat" matrix
 */
void hat(double a, double b, double c, double out[3][3])
{
	out[0][0] = 0;
	out[0][1] = -c;
	out[0][2] = b;
	out[1][0] = c;
	out[1][1] = 0;
	out[1][2] = -a;
	out[2][0] = -b;
	out[2][1] = a;
	out[2][2] = 0;
}

/*!
 * Compute adjoint
 *
 * Reference: cb.tfm.Ad
 *
 * @param[in] trans Transformation matrix (t44) to compute adjoint for
 * @param[out] ad Adjoint matrix
 */
void adjoint(const double trans[4][4], double ad[6][6])
{
	int i, j, k;
	double pos_hat[3][3];
	double tmp[3][3] = {0};

	memset(ad, 0, 36 * sizeof(double));

	/* compute hat(trans[:3][3]) */
	hat(trans[0][3], trans[1][3], trans[2][3], pos_hat);

	/* compute hat(trans[:3][3]) * trans[:3][:3] */
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			for (k = 0; k < 3; k++)
				tmp[i][j] += pos_hat[i][k] * trans[k][j];

	/* Setup adjo= [R hat(p)*R; 0 R] */
	for (j = 0; j < 3; j++){
		for (k = 0; k < 3; k++){
			ad[j][k] = trans[j][k];
			ad[j+3][k+3] = trans[j][k];
			ad[j][k+3] = tmp[j][k];
		}
	}
}

/*!
 * Compute a t44 inverse
 *
 * @param[in] t44 t44 to invert
 * @param[out] out inverted t44.
 */
void inv_t44(const double t44[4][4], double out[4][4])
{
	int i, j;

	memset(out, 0, 16*sizeof(double));
	for (i = 0; i < 3; i++){
		for (j = 0; j < 3; j++){
			out[i][j] = t44[j][i];
			out[i][3] += -t44[j][i]*t44[j][3];
		}
	}
	out[3][3] = 1;
}

/*!
 * Pure x-rotation t44
 *
 * @param[in] x_angle Angle to rotate
 * @param[out] out X-rotation t44.
 */
void rot_x(double x_angle, double out[4][4])
{
	double cx;
	double sx;

	memset(out, 0, 16*sizeof(double));
	cx = cos(x_angle);
	sx = sin(x_angle);
	out[1][1] = cx;
	out[1][2] = -sx;
	out[2][1] = sx;
	out[2][2] = cx;
	out[0][0] = 1;
	out[3][3] = 1;
}

/*!
 * Pure y-rotation t44
 *
 * @param[in] y_angle Angle to rotate
 * @param[out] out Y-rotation t44.
 */
void rot_y(double y_angle, double out[4][4])
{
	double cy;
	double sy;

	memset(out, 0, 16*sizeof(double));
	cy = cos(y_angle);
	sy = sin(y_angle);
	out[0][0] = cy;
	out[0][2] = sy;
	out[2][0] = -sy;
	out[2][2] = cy;
	out[1][1] = 1;
	out[3][3] = 1;
}

/*!
 * Pure z-rotation t44
 *
 * @param[in] z_angle Angle to rotate
 * @param[out] out Z-rotation t44.
 */
void rot_z(double z_angle, double out[4][4])
{
	double cz;
	double sz;

	memset(out, 0, 16*sizeof(double));
	cz = cos(z_angle);
	sz = sin(z_angle);
	out[0][0] = cz;
	out[0][1] = -sz;
	out[1][0] = sz;
	out[1][1] = cz;
	out[2][2] = 1;
	out[3][3] = 1;
}


/*!
 * Compute fwd for screw
 *
 * @param[in] screw Screw vector
 * @param[in] theta Joint angle
 * @param[out] out Homogenous transformation for the screw with given joint angle.
 * @param[out] twist The corresponding twist to the screw. Will only be set if
                     the screw is not all zeros.
 * @return 0 if the screw was all zeros, 1 otherwise.
 */
int screw_fwd(const double screw[6], double theta, double out[4][4], double twist[6])
{
	int i_screw;
	double sum_screw = 0, sum_omega = 0, sum_v = 0;

	for (i_screw = 0; i_screw < 6; i_screw++){
		sum_screw += fabs(screw[i_screw]);
		if (i_screw > 2){
			sum_omega += fabs(screw[i_screw]);
			sum_v += fabs(screw[i_screw-3]);
		}
	}

	if (sum_screw == 0) {
		eye(4, (double *) out);
		return 0;
	}

	/* Performance optimization */
	if(sum_v == 0 && fabs(screw[3]) == 1.0 && sum_omega == 1.0){
		rot_x(screw[3]*theta, out);
		memcpy(twist, screw, 6 * sizeof(double));
	} else if(sum_v == 0 && fabs(screw[4]) == 1.0 && sum_omega == 1.0){
		rot_y(screw[4]*theta, out);
		memcpy(twist, screw, 6 * sizeof(double));
	} else if(sum_v == 0 && fabs(screw[5]) == 1.0 && sum_omega == 1.0){
		rot_z(screw[5]*theta, out);
		memcpy(twist, screw, 6 * sizeof(double));
	} else {
		if (sum_omega == 0) {
			memcpy(twist, screw, 6 * sizeof(double));
		} else {
			/* Compute -omega(s[i]) x v(s[i]) = v(s[i]) x omega(s[i]) */
			xprod(twist_v(screw), twist_omega(screw), twist_v_w(twist));
			memcpy(twist_omega_w(twist), twist_omega(screw), 3 * sizeof(double));
		}
		twist_exp_hat(twist, theta, out);
	}
	return 1;
}


/*!
 * Compute transpose of spatial Jacobian and forward kinematics for each joint
 *
 * Reference: cb.num.kin.np_lpoe_spatial_jacobian
 *
 * The transpose of the Jacobian is calculated to allow for any number of joints
 *
 * @param[in] lrob Definition of the robot structure.
 * @param[in] n_screws Number of screws.
 * @param[in] q Joint angles.
 * @param[out] pose Forward kinematics t44
 * @param[out] jac_t Transpose of spatial Jacobian
 */
void spatial_jacobian_transpose_full_std(const struct model_lpoe_link lrob[],
		int n_screws, const double q[], double pose[][4][4],
		double jac_t[][6])
{
	double fk[4][4];
	int k;
	int count = 0;

	/* TODO: Add deformation? */
	eye(4, (double *) fk);

	for (k = 0; k < n_screws; k++){
		double tw_exp_hat[4][4];
		double twist[6];
		int ret = screw_fwd(lrob[k].s, q[count], tw_exp_hat, twist);

		if (ret > 0) {
			double ad[6][6];
			double tmp[4][4];
			adjoint(fk, ad);
			mul66_6(ad, twist, jac_t[count]);
			mul44(fk, tw_exp_hat, tmp);
			memcpy(fk, tmp, 4*4*sizeof(double));
			count++;
		}
		mul44(fk, lrob[k].trans, pose[k]);
		memcpy(fk, pose[k], 4*4*sizeof(double));
	}
}


void spatial_jacobian_full(const struct model_lpoe *model,
		const double q[6], double pose[][4][4], double jac[6][6])
{
	int i, j;
	double tmp;
	spatial_jacobian_transpose_full(model, q, pose, jac);
	/* Need to transpose the Jacobian */
	for (i = 0; i < 6; i++) {
		for (j = i + 1; j < 6; j++) {
			tmp = jac[i][j];
			jac[i][j] = jac[j][i];
			jac[j][i] = tmp;
		}
	}
}


void spatial_jacobian_transpose_full(const struct model_lpoe *model,
		const double q[], double pose[][4][4], double jac_t[][6])
{
	int n_joints = model->n_joints;  /* Needed for current VS workaround. */
	double th[N_JOINTS];
	handle_coupling_offs(model, q, n_joints, th);
	spatial_jacobian_transpose_full_std(model->lrob, model->n_screw,
			th, pose, jac_t);
	if (model->coupling_matrix) {
		int i, j, k, n = model->n_joints;
		double jac_t_[N_JOINTS][6];
		memset(jac_t_, 0, N_JOINTS * 6 * sizeof(double));
		for (i = 0; i < n; i++) {
			for (j = 0; j < n; j++) {
				if (fabs(model->coupling_matrix[i*n + j])){
					for (k = 0; k < n; k++) {
						jac_t_[j][k] += model->coupling_matrix[i*n + j]*jac_t[i][k];
					}
				}
			}
		}
		memcpy(jac_t, jac_t_, N_JOINTS*6*sizeof(double));
	} else if (model->has_pbar) {
		int k;
		for (k = 0; k < 6; k++)
			jac_t[1][k] -= model->par_fact*jac_t[2][k];
	}
}


/*!
 * Compute spatial Jacobian and forward kinematics
 *
 * Reference: cb.num.kin.np_lpoe_spatial_jacobian
 *
 * @param[in] model Definition of the robot model.
 * @param[in] q Joint angles.
 * @param[out] fk Forward kinematics t44
 * @param[out] jac Spatial Jacobian Matrix
 */
void spatial_jacobian(const struct model_lpoe *model,
		const double q[6], double fk[4][4], double jac[6][6])
{
	int n_screws = model->n_screw;
	double pose[N_SCREWS][4][4];

	spatial_jacobian_full(model, q, pose, jac);
	memcpy(fk, pose[model->n_screw-1], 4*4*sizeof(double));
}


/*!
 * Compute transpose of spatial Jacobian and forward kinematics
 *
 * Transpose of the Jacobian to allow for any number of joints
 *
 * Reference: cb.num.kin.np_lpoe_spatial_jacobian
 *
 * @param[in] model Definition of the robot model.
 * @param[in] q Joint angles.
 * @param[out] fk Forward kinematics t44
 * @param[out] jac_t Transpose of spatial Jacobian
 */
void spatial_jacobian_transpose(const struct model_lpoe *model,
		const double q[], double fk[4][4], double jac_t[][6])
{
	int n_screws = model->n_screw;
	double pose[N_SCREWS][4][4];

	spatial_jacobian_transpose_full(model, q, pose, jac_t);
	memcpy(fk, pose[model->n_screw-1], 4*4*sizeof(double));
}


/*!
 * @param[in] lrob Definition of the robot structure.
 * @param[in] theta Joint angles given in radians starting at the base.
 * @param[in] end_link Index of the link for which to compute the
 *         position.
 * @param[out] out Homogenous transformation from base to the
 *         chosen link.
 */
void fwd_lpoe_std(const struct model_lpoe_link lrob[], const double theta[],
		int end_link, double out[][4][4])
{
	double all[4][4];
	int i_theta = 0; /* Index for nonzero elements in model. */
	int i;

	eye(4, (double *) all);
	for (i = 0; i < end_link; i++) {
		double twist[6];
		double t[4][4];
		double twist_mul_trans[4][4];

		i_theta += screw_fwd(lrob[i].s, theta[i_theta], t, twist);
		mul44(t, lrob[i].trans, twist_mul_trans);
		mul44(all, twist_mul_trans, out[i]);
		memcpy(all, out[i], 4*4*sizeof(double));
	}
}

/* TODO: Impl. non-full fwd_lpoe() */
#ifndef _MSC_VER
/*!
 * @param[in] lrob Definition of the robot structure.
 * @param[in] theta Joint angles given in radians starting at the base.
 * @param[in] end_link Index of the link for which to compute the
 *         position.
 * @param n_sample Number of samples
 * @param[out] pose Homogenous transformations from base to the
 *         chosen link.
 */
void fwd_lpoe_list(const struct model_lpoe *model, const double *theta[],
		int end_link, unsigned int n_sample, double pose[][4][4])
{
	size_t n = end_link-1;  /* VS workaround */
	double pose_joints[n+1][4][4];
	unsigned int i;

	for (i = 0; i < n_sample; i++) {
		fwd_lpoe(model, theta[i], end_link, pose_joints);
		memcpy(pose[i], pose_joints[end_link-1], 16 * sizeof(double));
	}
}
#endif


static void handle_coupling_offs(const struct model_lpoe *model, const double th_in[],
		int end_joint, double th[])
{
	int i;
	int n_joints = model->n_joints;
	double th_offs[N_JOINTS];
	if (model->q_a_offs) {
		memcpy(th_offs, model->q_a_offs, n_joints * sizeof(double));
	} else {
		memset(th_offs, 0, n_joints * sizeof(double));
	}
	if (model->coupling_matrix) {
		int i, j;
		for (i = 0; i < end_joint; i++) {
			th[i] = 0.0;
			for (j = 0; j < model->n_joints; j++) {
				th[i] += model->coupling_matrix[i*model->n_joints + j]*(th_in[j] + th_offs[j]);
			}
		}
	} else {
		for (i = 0; i < end_joint; i++)
			th[i] = th_in[i] + th_offs[i];
		if (model->has_pbar && end_joint > 2)
			th[2] -= model->par_fact*th[1];
	}
}


void fwd_lpoe(const struct model_lpoe *model, const double theta[],
		int end_link, double out[][4][4])
{
	int n_joints = end_link;  /* Needed for current VS workaround. */
#ifdef LPOE_MAX_N_SCREWS
	double th[N_SCREWS];
#endif
	if (end_link > model->n_joints)
		n_joints = model->n_joints;
#ifndef LPOE_MAX_N_SCREWS
	double th[N_JOINTS];
#endif

	handle_coupling_offs(model, theta, n_joints, th);
	fwd_lpoe_std(model->lrob, th, end_link, out);
}

/*!
 * Compute geometric Jacobian and time derivative of Jacobian
 *
 * @param[in] lrob Definition of the robot structure.
 * @param[in] n_screws Number of screws.
 * @param[in] n_joints Number of joints.
 * @param[in] q Joint angles.
 * @param[in] qd Joint velocities.
 * @param[out] fk Forward kinematics t44
 * @param[out] jac Geometric Jacobian
 * @param[out] jdot Time derivative of Jacobian
 */
void jacobian_jdot(const struct model_lpoe_link lrob[], int n_screws,
		int n_joints, const double q[], const double qd[],
		double fk[4][4], double *jac, double *jdot)
{
	double omegas[N_SCREWS][3];
	int non_zero_omega[N_SCREWS];
	int k, i;
	int count = 0;

	memset(jac, 0, 6 * n_joints * sizeof(double));
	memset(jdot, 0, 6 * n_joints * sizeof(double));
	eye(4, (double *) fk);
	memset(omegas, 0, 3 * n_joints * sizeof(double));
	memset(non_zero_omega, 0, n_joints * sizeof(int));

	for (k = 0; k < n_screws; k++) {
		double tmp[4][4];
		double tw_exp_hat[4][4];
		double twist[6];
		int ret = screw_fwd(lrob[k].s, q[count], tw_exp_hat, twist);

		if (ret > 0) {
			double rot_mat[3][3];
			double p_prev[3];
			double r_tilde[3];
			double tmpvec[3];
			double tmpvec2[3];
			double sum_omega = 0, sum_v = 0;
			int j, m;

			for (i=0; i<3; i++) {
				memcpy(rot_mat[i], &fk[i][0], 3 * sizeof(double));
				sum_omega += fabs(lrob[k].s[i+3]);
				sum_v += fabs(lrob[k].s[i]);
			}

			if (sum_omega > 0.0 && sum_v == 0.0) {
				/* Revolute joint */
				double joint_axis[3];
				double rot_axis[3];
				memcpy(rot_axis, twist_omega(lrob[k].s), 3 * sizeof(double));
				mul33_3(rot_mat, rot_axis, joint_axis);
				non_zero_omega[count] = 1;
				memcpy(omegas[count], joint_axis, 3 * sizeof(double));
				for (i=0; i<3; i++) {
					jac[(i+3)*n_joints + count] = joint_axis[i];
				}
			}
			for (i=0; i<3; i++) {
				p_prev[i] = fk[i][3];
			}

			/* Do lpoe forward*/
			mul44(fk, tw_exp_hat, tmp);
			memcpy(fk, tmp, 4*4*sizeof(double));
			mul44(fk, lrob[k].trans, tmp);
			memcpy(fk, tmp, 4*4*sizeof(double));

			for (i=0; i<3; i++) {
				r_tilde[i] = fk[i][3] - p_prev[i];
			}

			for (i=0; i<n_joints; i++) {
				if (non_zero_omega[i] == 1) {
					xprod(omegas[i], r_tilde, tmpvec);
					for (j=0; j<3; j++) {
						jac[j*n_joints+i] += tmpvec[j];
					}
					for (j=0; j<n_joints; j++) {
						if (non_zero_omega[j] == 1) {
							xprod(omegas[j], r_tilde, tmpvec);
							xprod(omegas[i], tmpvec, tmpvec2);
							for (m=0; m<3; m++) {
								jdot[m*n_joints+i] += tmpvec2[m]*qd[j];
							}
						}
					}
				}
			}

			if (sum_omega == 0.0 && sum_v > 0.0) {
				/* Prismatic joint */
				double prism_axis[3];
				double loc_axis[3];
				memcpy(loc_axis, twist_v(lrob[k].s), 3 * sizeof(double));
				mul33_3(rot_mat, loc_axis, prism_axis);
				for (i=0; i<3; i++) {
					jac[i*n_joints+count] = prism_axis[i];
				}
				for (i=0; i<n_joints; i++) {
					xprod(omegas[i], prism_axis, tmpvec);
					for (j=0; j<3; j++) {
						jdot[j*n_joints+count] += tmpvec[j]*qd[i];
						jdot[j*n_joints+i] += tmpvec[j]*qd[count];
					}
				}
			}
			count++;
		} else {
			mul44(fk, lrob[k].trans, tmp);
			memcpy(fk, tmp, 4*4*sizeof(double));
		}
	}
}


/*!
 * Transform Jacobian wrt. transformation before and after
 *
 * @param[in] jac_t Transpose of Jacobian.
 * @param[in] n Number of rows in transposed Jacobian.
 * @param[in] t Forward kinematics for function Jacobian is given for.
 * @param[in] tb Forward kinematics before Jacobian.
 * @param[in] ta Forward kinematics after Jacobian.
 * @param[out] fk Forward kinematics (tb*t*ta)
 * @param[out] jac_out Transformed transposed Jacobian
 */
void transform_jacobian(double jac_t[][6], int n, const double t[4][4],
		const double tb[4][4], const double ta[4][4],
		double fk[4][4], double jac_out[][6])
{
	int i, j, k;
	double t1[4][4];
	double r[3];
	double tmp[3];

	/* Handle transformation before */
	for (i = 0; i < n; i++) {
		for (j = 0; j < 3; j++) {
			jac_out[i][j] = 0.0;
			jac_out[i][j+3] = 0.0;
			for (k = 0; k < 3; k++) {
				jac_out[i][j] += jac_t[i][k]*tb[j][k];
				jac_out[i][j+3] += jac_t[i][k+3]*tb[j][k];
			}
		}
	}

	/* Calculate total fwd */
	mul44(tb, t, t1);
	mul44(t1, ta, fk);

	/* Handle transformation after */
	for (i = 0; i < 3; i++)
		r[i] = fk[i][3] - t1[i][3];
	for (i = 0; i < n; i++) {
		xprod(&jac_out[i][3], r, tmp);
		for (j = 0; j < 3; j++)
			jac_out[i][j] += tmp[j];
	}
}


/*!
 * Transform from spatial Jacobian to "normal" geometric Jacobian
 * (body Jacobian specified in base frame)
 *
 * @param[in] spat_jac_t Transpose of spatial Jacobian.
 * @param[in] n Number of rows in transposed spatial Jacobian.
 * @param[in] t Forward kinematics for function Jacobian is given for.
 * @param[out] jac_out Normal transposed Jacobian
 */
void spatial_to_normal_jacobian(double spat_jac_t[][6], int n,
		const double t[4][4], double jac_out[][6])
{
	double r[3] = {t[0][3], t[1][3], t[2][3]};
	double tmp[3];
	int i, j;

	for (i = 0; i < n; i++) {
		xprod(r, &spat_jac_t[i][3], tmp);
		for (j = 0; j < 3; j++)
			jac_out[i][j] = spat_jac_t[i][j] - tmp[j];
		memcpy(&jac_out[i][3], &spat_jac_t[i][3], 3 * sizeof(double));
	}
}
