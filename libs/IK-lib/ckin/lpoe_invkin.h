/*
 * Copyright (C) 2018, 2019 Cognibotics
 *
 * Use or copying of this file or any part of it, without explicit
 * permission from Cognibotics is not allowed in any way.
 */
/*!
 * @file
 * @brief LPOE-based nominal inverse kinematics.
 */
#ifndef LPOE_NOMINV_H
#define LPOE_NOMINV_H

#include "lpoe.h"
#include "common.h"

/*!
 * Structure with parameters for a nominal LPOE model
 *
 * This structure may be generated using cblib
 */
struct model_lpoe_nom {
	struct model_lpoe model;  /*!< LPOE parameters. */
	double rot_dirs[6];  /*!< Rotation directions for the joints. */
	double wcp0[4][4];  /*!< Pose of wcp when all joints are zero. */
	double l1;  /*!< Precalculated distance 1 for j2j3 inv calc. */
	double l2;  /*!< Precalculated distance 2 for j2j3 inv calc. */
	double th1_0;  /*!< Precalculated angle 1 for j1j2j3 inv calc. */
	double th2_0;  /*!< Precalculated angle 2 for j1j2j3 inv calc. */
	int hollow_wrist;  /*!< 1 if Comau Hollow Wrist, 0 otherwise. */
};

void copy_translation_part(const double t44[4][4], double out[4][4]);

int inv_pos(const struct model_lpoe_nom *rob, const double wcp[4][4],
		int overhead_pos, int elbow_down, double th123[3]);

int nom_inv(const struct model_lpoe_nom *rob, const double tcp[4][4],
		int overhead_pos, int elbow_down, int neg_a5,
		double q_inv_out[6]);

int nom_inv_hw(const struct model_lpoe_nom *rob_hw, const double tcp[4][4],
		int overhead_pos, int elbow_down, int neg_a5, double q0[6],
		double tol, double initial_tol, double q_inv_out[6]);

int nom_inv_hw2(const struct model_lpoe_nom *rob_hw, const double tcp[4][4],
		int overhead_pos, int elbow_down, int neg_a5, double q0[6],
		double tol, double q_inv_out[6]);

void tcp2wcp_hw(const struct model_lpoe_nom *rob_hw, const double tcp[4][4],
		double q6, double wcp_out[4][4]);

int nom_inv_iteration_hw(const struct model_lpoe_nom *rob_hw,
		const double tcp[4][4], double q6, int overhead_pos, int elbow_down,
		int neg_a5, double q_inv_out[6], double errs[2]);

int nom_inv_ur(const struct model_lpoe_nom *rob, const double tcp[4][4], int overhead_pos,
		int elbow_down, int neg_a5, double q_inv_out[6]);

#endif
