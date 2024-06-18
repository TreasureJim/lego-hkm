/*
 * Copyright (C) 2018, 2019 Cognibotics
 *
 * Use or copying of this file or any part of it, without explicit
 * permission from Cognibotics is not allowed in any way.
 */
/*!
 * @file
 * @brief DH-based inverse kinematics.
 */
#ifndef DH_NOMINV_H
#define DH_NOMINV_H

/*!
 * Structure with DH model parameters
 *
 * The a, d, and alpha parameters must have the following form:
 * \code
 * a = [a1, a2, a3, 0, 0, 0]
 * d = [d1, 0, d3, d4, 0, d6]
 * alpha = [-pi/2, 0, pi/2, -pi/2, pi/2, 0]
 * \endcode
 */
struct model_dh {
	double a[6];  /*!< DH a-parameters. */
	double d[6];  /*!< DH d-parameters. */
	double alpha[6];  /*!< DH alpha-parameters. */
	double rot_dir[6];  /*!< Rotation directions for the joints. */
	double rot_offs[6];  /*!< DH rotation offset parameters. */
};

void dhmatrix(const struct model_dh dhrob, int n, double theta,
		double out[4][4]);

void xrot_ytr(double rx, double y, double out[4][4]);

void tf2euler_zyz_intrinsic(const double t44[4][4], double out[3]);

/* FIXME: Shape is weird. */
void inv_pos_dh(const struct model_dh dhrob, const double wcp[4][4],
		int overhead_pos, int elbow_down, double th123[4]);

/* FIXME: Shape is weird. */
void nom_inv_dh(const struct model_dh dhrob, const double tcp[4][4],
		int overhead_pos, int elbow_down, int neg_a5, double ax[7]);

#endif
