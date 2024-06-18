/*
 * Copyright (C) 2018, 2019 Cognibotics
 *
 * Use or copying of this file or any part of it, without explicit
 * permission from Cognibotics is not allowed in any way.
 */
/*!
 * @file
 * @brief Common functions.
 */
#ifndef COMMON_H
#define COMMON_H

#define KIN_OK (0)  /*!< Nominal inverse solution found. */
#define KIN_UNREACHABLE (1)  /*!< Nominal inverse failed, pose unreachable. */
#define KIN_BAD_SOL (2)  /*!< Nominal inverse failed to reach desired tolerance. */

#define invphi ((sqrt(5)-1)/2)
#define invphi2 ((3-sqrt(5))/2)

void tf2euler_zyz_intrinsic(const double t44[4][4], double out[3]);
void tf2euler_zyx_intrinsic(const double t44[4][4], double out[3]);

void euler_zyz_intrinsic2tf(const double euler_zyz[3], double t44[4][4]);
void euler_zyx_intrinsic2tf(const double euler_zyx[3], double t44[4][4]);

void tf2quat(const double t44[4][4], double q[4]);
void quat2tf(const double q[4], double t44[4][4]);
void quat_mul(const double q1[4], const double q2[4], double prod[4]);
void quat_inv(const double q[4], double q_inv[4]);

int inv_pos_j2j3(const double wcp_arm[4][4], double l1, double l2,
		int elbow_down, double th23[2]);

void log_trans(const double t44[4][4], double out[4][4]);

void vee_log_diff(const double t44_curr[4][4], const double t44_target[4][4],
		double out[6]);

double pos_diff(const double t44_curr[4][4], const double t44_target[4][4]);

double rot_diff(const double t44_curr[4][4], const double t44_target[4][4]);

void logR(const double t44_in[4][4], double log_R[3][3]);

void tf2basepar(const double t44[4][4], double out[6]);
void basepar2tf(const double bp[6], double out[4][4]);

void tr2diff(const double t44_1[4][4], const double t44_2[4][4], double diffvec[6]);

void normdiff(const double t44_1[4][4], const double t44_2[4][4], double normvec[2]);

#endif
