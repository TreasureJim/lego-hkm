/*
 * Copyright (C) 2020 Cognibotics
 *
 * Use or copying of this file or any part of it, without explicit
 * permission from Cognibotics is not allowed in any way.
 */
/*!
 * @file
 * @brief Linear algebra related functions.
 */
#ifndef LIN_ALG_H
#define LIN_ALG_H

#define SOLUTION_OK 0
#define IN_VICINITY_OF_SINGULARITY 1
#define CLOSE_TO_SINGULARITY 2
#define SINGULAR_SYSTEM 3

int solve_9x9_gauss_elim(const double mat[9][9], const double y[9], double sol[9]);
int solve_9x9_9_damped_ls(const double mat[9][9], const double y[9],
		double lambda, double sol[9]);

int solve_12x12_gauss_elim(const double mat[12][12], const double y[12], double sol[12]);
int solve_12x12_12_damped_ls(const double mat[12][12], const double y[12],
		double lambda, double sol[12]);

int solve_13x13_gauss_elim(const double mat[13][13], const double y[13], double sol[13]);
int solve_13x13_13_damped_ls(const double mat[13][13], const double y[13],
		double lambda, double sol[13]);

int solve_15x15_gauss_elim(const double mat[15][15], const double y[15], double sol[15]);
int solve_15x15_15_damped_ls(const double mat[15][15], const double y[15],
		double lambda, double sol[15]);

double determinant_66(const double mat[6][6]);
int solve_66_gauss_elim(const double mat[6][6], const double y[6], double sol[6]);
int solve_66_6_damped_ls(const double mat[6][6], const double y[6], double lambda, double sol[6]);

int inv_33(const double mat[3][3], double out[3][3]);
int inv_22(const double mat[2][2], double out[2][2]);
int solve_33_3(const double mat[3][3], const double vec[3], double out[3]);
int solve_22_2(const double mat[2][2], const double vec[2], double out[2]);

#endif
