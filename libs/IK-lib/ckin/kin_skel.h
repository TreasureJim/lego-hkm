/*
 * Copyright (C) 2015, 2016, 2017, 2018, 2019 Cognibotics
 *
 * Use or copying of this file or any part of it, without explicit
 * permission from Cognibotics is not allowed in any way.
 */
/*!
 * @file
 * @brief LPOE-based iterative inverse kinematics.
 */
#include "c_lpoe/lpoe.h"

#ifndef KIN_SKEL_H
#define KIN_SKEL_H

#define KIN_SUCCESS 0  /*!< Inverse solution found. */
#define KIN_FAILURE_STEP_NAN 1  /*!< Inverse failed due to computation error. */
#define KIN_FAILURE_NO_CONVERGE 2  /*!< Inverse failed to reach tolerance. */
#define KIN_WARNING_REDUCED_TOLERANCE -1  /*!< Inverse kinematics convergence stopped before tolerance was reached. */
#define KIN_WARNING_VICINITY_OF_SINGULARITY -2  /*!< Inverse kinematics iterations passed through position close to a singularity, possible loss of accuracy. */



/*!
 * Statistics from iterative inverse kinematics.
 */
struct kin_skel_stats {
	unsigned long n_iter;  /*!< Number of iterations completed. */
	double err_pos;  /*!< Final position error, total distance.. */
	double err_rot;  /*!< Final orientation error, axis-angle angle. */
};

double kin_skel_norm(const double * const v, const unsigned long len);

int kin_skel_inv(const unsigned long  n,
		const struct model_lpoe *lrob,
		const double pose_tgt[4][4],
		const double q_start[],
		unsigned long iter_max,
		double tol_pos,
		double tol_rot,
		double q_res[],
		struct kin_skel_stats *stats);

#endif
