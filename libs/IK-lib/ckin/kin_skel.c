/*
 * Copyright (C) 2016, 2017, 2018, 2019 Cognibotics
 *
 * Use or copying of this file or any part of it, without explicit
 * permission from Cognibotics is not allowed in any way.
 */

#ifdef CB_SYSTEM_HEADER
#include CB_SYSTEM_HEADER
#else
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#endif

#include "c_lpoe/lpoe.h"
#include "common.h"
#include "lin_alg.h"

#include "kin_skel.h"

#define DEFAULT_ITER_MAX 100
#define DEFAULT_TOL_POS (1e-4 * 1e-3)
#define DEFAULT_TOL_ROT 1e-4

#define NUP 11
#define NUN 9
#define NORM_STEP_MAX 3
#define MAXITER_LAMBDA_CALC 10

/* Allow code to work without VLAs and dynamic memory allocation. */
#ifdef KIN_MAX_N
#define N KIN_MAX_N
#else
#define N n
#endif

/*!
 * Euclidean norm.
 *
 * @param[in] v Vector
 * @param[in] len Length of the vector
 * @returns The norm of the first `len` elements of `v`.
 */
double kin_skel_norm(const double * const v, const unsigned long len)
{
	double res = 0;
	unsigned long i;

	for (i = 0; i < len; i++)
		res += v[i] * v[i];
	return sqrt(res);
}

/*! Iterative inverse kinematics with compliance.
 *
 * @param[in] n DOF
 * @param[in] lrob Definition of the robot structure.
 * @param[in] pose_tgt Target flange pose.
 * @param[in] q_start Initial joint position.
 * @param[in] iter_max Maximum number of iterations.
 * @param[in] tol_pos Tolerance, position [m].
 * @param[in] tol_rot Tolerance, rotation [rad].
 * @param[out] q_res Joint positions.
 * @param[out] stats Debug information.
 */
int kin_skel_inv(const unsigned long n,
		const struct model_lpoe *model,
		const double pose_tgt[4][4],
		const double q_start[],
		unsigned long iter_max,
		double tol_pos,
		double tol_rot,
		double q_res[],
		struct kin_skel_stats *stats)
{
	int res = 0;            /* Return value of this function. */
	int converged;          /* Solution is within the desired tolerance. */
	int ret;

	double jac[N][N];

	double q_curr[N];   /* Current solution. */
	double q_step[N];   /* Current step. */
	double pose_curr[4][4]; /* Pose corresponding to current solution. */
	double twist_coord[6];

	double err_pos;
	double err_rot;
	double prev_err_pos = 0;
	double prev_err_rot = 0;

	int low_progress_count = 0;
	int low_progress_count2 = 0;

	unsigned long i_step;

	/* Set default parameters. */
	if (iter_max == 0)
		iter_max = DEFAULT_ITER_MAX;
	if (tol_pos == 0.0)
		tol_pos = DEFAULT_TOL_POS;
	if (tol_rot == 0.0)
		tol_rot = DEFAULT_TOL_ROT;

	/* Make sure outputs never contains garbage. */
	memset(q_res, 0, n * sizeof(q_res[0]));

	/* Initiate computation state. */
	memcpy(q_curr, q_start, sizeof(q_curr));
	converged = 0;

	spatial_jacobian(model, q_curr, pose_curr, jac);

	for (i_step = 0; i_step < iter_max; i_step++) {
		double tmp_calc = 0.0;
		unsigned long i_axis;
		double norm_step;
		unsigned long lambda_step = 0;
		double max_norm_step;
		double lambda = 1e-5;    /* Damping factor. */

		/* Compute the next step. */
		vee_log_diff(pose_curr, pose_tgt, twist_coord);
		ret = solve_66_gauss_elim(jac, twist_coord, q_step);

		/* Check that a solution was returned and calculate step norm */
		if (ret == SOLUTION_OK || ret == IN_VICINITY_OF_SINGULARITY || ret == CLOSE_TO_SINGULARITY) {
			tmp_calc = 0.0;
			for (i_axis = 0; i_axis < n; i_axis++)
				tmp_calc += pow(q_step[i_axis], 2);
			norm_step = sqrt(tmp_calc);
		} else {
			norm_step = NORM_STEP_MAX*2;
		}
		max_norm_step = fmax(1.0, NORM_STEP_MAX*(1.0 - ((double) i_step)/15.0));
		while (norm_step > max_norm_step && lambda_step < MAXITER_LAMBDA_CALC){
			lambda_step++;
			lambda *= NUP;
			ret = solve_66_6_damped_ls(jac, twist_coord, lambda, q_step);
			/* Check sol was returned + calculation of step norm. */
			if (ret == SOLUTION_OK || ret == IN_VICINITY_OF_SINGULARITY || ret == CLOSE_TO_SINGULARITY) {
				tmp_calc = 0.0;
				for (i_axis = 0; i_axis < n; i_axis++)
					tmp_calc += pow(q_step[i_axis], 2);
				norm_step = sqrt(tmp_calc);
			} else {
				norm_step = NORM_STEP_MAX*2;
			}
		}

		for (i_axis = 0; i_axis < n; i_axis++)
			q_curr[i_axis] += q_step[i_axis];

		spatial_jacobian(model, q_curr, pose_curr, jac);

	#ifdef KIN_SKEL_INV_TRACE
		KIN_SKEL_INV_TRACE(
				i_step,
				lambda,
				kin_skel_norm(q_step, n),
				err_pos,
				err_rot,
				q_curr);
	#endif
		err_pos = pos_diff(pose_curr, pose_tgt);
		err_rot = fabs(rot_diff(pose_curr, pose_tgt));
		if (err_pos < tol_pos && err_rot < tol_rot) {
			converged = 1;
			break;
		} else if (fabs(prev_err_pos-err_pos) < tol_pos*1e3 && fabs(prev_err_rot-err_rot) < tol_rot*1e3) {
			if (low_progress_count2++ > 10)
				break;
			if (fabs(prev_err_pos-err_pos) < tol_pos*10 && fabs(prev_err_rot-err_rot) < tol_rot*10) {
				if (low_progress_count++ > 5)
					break;
			}
		}
		prev_err_pos = err_pos;
		prev_err_rot = err_rot;
	}

	if (stats != NULL) {
		stats->n_iter = i_step+1;
		stats->err_pos = err_pos;
		stats->err_rot = err_rot;
	}

	if (converged) {
		memcpy(q_res, q_curr, sizeof(q_curr));
		res = 0;
	} else if (res == 0) {
		memcpy(q_res, q_curr, sizeof(q_curr));
		res = KIN_FAILURE_NO_CONVERGE;
	}

	return res;
}
