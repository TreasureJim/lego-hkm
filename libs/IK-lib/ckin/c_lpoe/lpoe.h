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
#ifndef LPOE_H
#define LPOE_H

/*!
 * Single link attached to (optional) joint.
 */
struct model_lpoe_link {
        double s[6];  /*!< Joint motion as screw. */
        double trans[4][4];  /*!< Link as homogenous transformation. */
};

/*!
 * LPOE model structure
 *
 * The fwd kinematics is calculated as
 * e^(hat(twist(s0))*q0)*tr0 * e^(hat(twist(s1))*q1)*tr1 * ... * e^(hat(twist(s_n))*q_m)*tr_n
 * where twist(.) is a transformation from screw to twist. The joint angles q
 * need only to have as many elements as there are non-zero screws, therefore
 * m <= n (where usually n_screw = n + 1). The number of joints is specified in
 * the parameter n_joints.
 *
 * Parallel bar structure may either be specified as a coupling matrix, i.e., a
 * square matrix with as many rows/columns as the number of joints, where the
 * forward kinematics is calculated as f(th), where f(.) is the fwd kin
 * described above, and th = coupling_matrix*q.
 *
 * To reduce calculations, also the inverse coupling matrix should be specified.
 *
 * An alternative way to describe a parallel bar structure is to let the
 * coupling matrix be NULL and set has_pbar to 1 and par_fact to +/-1 1.0.
 *
 * Any eventual parallel linkage geometry is described with the argument
 * lrob_par. This will only be considered for dynamics calculation and only
 * if has_pbar is non-zero.
 */
struct model_lpoe {
	int n_screw;
	struct model_lpoe_link *lrob;
	int has_pbar;
	double par_fact;
	int n_joints;
	double *coupling_matrix;
	double *inverse_coupling_matrix;
	double *q_a_offs;
	struct model_lpoe_link *lrob_pbar;
};

/*!
 * Function to initialize lpoe model with default parameters
 *
 * @param[in] model Pointer to a lpoe model object to initialize
 */
void lpoe_init(struct model_lpoe *model);

const double *twist_omega(const double twist[6]);
const double *twist_v(const double twist[6]);
double *twist_omega_w(double twist[6]);
double *twist_v_w(double twist[6]);
void twist_omega_hat(const double t[6], double out[3][3]);
void twist_R(const double t[6], double theta, double out33[3][3]);
void twist_exp_hat(const double t[6], double theta, double out[4][4]);
void hat(double a, double b, double c, double out[3][3]);
void inv_t44(const double t44[4][4], double out[4][4]);
void rot_x(double x_angle, double out[4][4]);
void rot_y(double y_angle, double out[4][4]);
void rot_z(double z_angle, double out[4][4]);
int screw_fwd(const double screw[6], double theta, double out[4][4], double twist[6]);
void vee(const double that[4][4], double out[6]);
void adjoint(const double trans[4][4], double ad[6][6]);

void fwd_lpoe(const struct model_lpoe *lrob, const double theta[],
                int end_link, double out[][4][4]);
void fwd_lpoe_list(const struct model_lpoe *model, const double *theta[],
                int end_link, unsigned int n_sample, double pose[][4][4]);

void fwd_lpoe_std(const struct model_lpoe_link lrob[], const double theta[],
                int end_link, double out[][4][4]);

void spatial_jacobian(const struct model_lpoe *model,
                const double q[6], double fk[4][4], double jac[6][6]);
void spatial_jacobian_full(const struct model_lpoe *lrob,
		const double q[6], double pose[][4][4], double jac[6][6]);

void spatial_jacobian_transpose(const struct model_lpoe *model,
                const double q[], double fk[4][4], double jac_t[][6]);
void spatial_jacobian_transpose_full(const struct model_lpoe *lrob,
                const double q[], double pose[][4][4], double jac_t[][6]);

void jacobian_jdot(const struct model_lpoe_link lrob[], int n_screws,
		int n_joints, const double q[], const double qd[],
		double fk[4][4], double *jac, double *jdot);

void transform_jacobian(double jac_t[][6], int n, const double t[4][4],
                const double tb[4][4], const double ta[4][4],
                double fk[4][4], double jac_out[][6]);
void spatial_to_normal_jacobian(double spat_jac_t[][6], int n,
                const double t[4][4], double jac_out[][6]);

#endif
