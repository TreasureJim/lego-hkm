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
#include <string.h>

#include "lin_alg.h"

#include "c_lpoe/lpoe.h"
#include "c_lpoe/matrix_math.h"

/*!
 * Calculate inverse of 3x3 matrix
 *
 * @param[in] mat 3x3 matrix.
 * @param[out] out Inverse of mat.
 */
int inv_33(const double mat[3][3], double out[3][3])
{
	double m[3][3];
	double det_mat;
	m[0][0] = mat[1][1]*mat[2][2] - mat[1][2]*mat[2][1];
	m[0][1] = mat[1][0]*mat[2][2] - mat[1][2]*mat[2][0];
	m[0][2] = mat[1][0]*mat[2][1] - mat[1][1]*mat[2][0];
	det_mat = mat[0][0]*m[0][0] - mat[0][1]*m[0][1] + mat[0][2]*m[0][2];
	if (fabs(det_mat) < 1e-10)
		return 1;
	double inv_det_mat, sign;
	int i, j;
	inv_det_mat = 1.0/det_mat;
	m[1][0] = mat[0][1]*mat[2][2] - mat[0][2]*mat[2][1];
	m[1][1] = mat[0][0]*mat[2][2] - mat[0][2]*mat[2][0];
	m[1][2] = mat[0][0]*mat[2][1] - mat[0][1]*mat[2][0];
	m[2][0] = mat[0][1]*mat[1][2] - mat[0][2]*mat[1][1];
	m[2][1] = mat[0][0]*mat[1][2] - mat[0][2]*mat[1][0];
	m[2][2] = mat[0][0]*mat[1][1] - mat[0][1]*mat[1][0];
	sign = 1.0;
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			out[i][j] = sign*inv_det_mat*m[j][i];
			sign *= -1.0;
		}
	}
	return 0;
}


/*!
 * Calculate inverse of 2x2 matrix
 *
 * @param[in] mat 2x2 matrix.
 * @param[out] out Inverse of mat.
 */
int inv_22(const double mat[2][2], double out[2][2])
{
	double det_mat;
	det_mat = mat[0][0]*mat[1][1] - mat[0][1]*mat[1][0];
	if (fabs(det_mat) < 1e-10)
		return 1;
	double inv_det_mat = 1.0/det_mat;
	out[0][0] = inv_det_mat*mat[1][1];
	out[0][1] = -inv_det_mat*mat[0][1];
	out[1][0] = -inv_det_mat*mat[1][0];
	out[1][1] = inv_det_mat*mat[0][0];
	return 0;
}


/*!
 * Calculate solution x to linear system mat*out = vec
 *
 * @param[in] mat 3x3 matrix.
 * @param[in] vec 3-vector.
 * @param[out] out Solution.
 */
int solve_33_3(const double mat[3][3], const double vec[3], double out[3])
{
	double inv[3][3];
	double tmp1[3][3];
	double tmp2[3][3];
	int failure, iter;
	double lambda;

	/* First try to calculate inverse */
	failure = inv_33(mat, inv);
	if (failure == 0)
		mul33_3(inv, vec, out);

	/* Increase lambda until a valid damped inverse has been calculated */
	iter = 0;
	lambda = 0.00001;
	while (iter < 10 && failure > 0) {
		mul33_transpose2(mat, mat, tmp1);
		tmp1[0][0] += fmax(tmp1[0][0], 0.001)*lambda;
		tmp1[1][1] += fmax(tmp1[1][1], 0.001)*lambda;
		tmp1[2][2] += fmax(tmp1[2][2], 0.001)*lambda;
		failure = inv_33(tmp1, tmp2);
		if (failure == 0) {
			mul33_transpose(tmp2, mat, inv);
			mul33_3(inv, vec, out);
		}
		iter++;
		lambda *= 10;
	}
	return failure;
}


/*!
 * Calculate solution x to linear system mat*out = vec
 *
 * @param[in] mat 2x2 matrix.
 * @param[in] vec 2-vector.
 * @param[out] out Solution.
 */
int solve_22_2(const double mat[2][2], const double vec[2], double out[2])
{
	double inv[2][2];
	double tmp1[2][2];
	double tmp2[2][2];
	int failure, iter;
	double lambda;

	/* First try to calculate inverse */
	failure = inv_22(mat, inv);
	if (failure == 0) {
		out[0] = inv[0][0]*vec[0] + inv[0][1]*vec[1];
		out[1] = inv[1][0]*vec[0] + inv[1][1]*vec[1];
	}

	/* Increase lambda until a valid damped inverse has been calculated */
	iter = 0;
	lambda = 0.00001;
	while (iter < 10 && failure > 0) {
		mul22_transpose2(mat, mat, tmp1);
		tmp1[0][0] += fmax(tmp1[0][0], 0.001)*lambda;
		tmp1[1][1] += fmax(tmp1[1][1], 0.001)*lambda;
		failure = inv_22(tmp1, tmp2);
		if (failure == 0) {
			mul22_transpose(tmp2, mat, inv);
			out[0] = inv[0][0]*vec[0] + inv[0][1]*vec[1];
			out[1] = inv[1][0]*vec[0] + inv[1][1]*vec[1];
		}
		iter++;
		lambda *= 10;
	}

	return failure;
}


/*!
 * Swap rows in matrix
 *
 * @param[in,out] mat Array representing 6x6 matrix
 * @param[in] r1 Index of the first row to swap
 * @param[in] r2 Index of the second row to swap
 */
void row_swap(double mat[6][6], int r1, int r2)
{
	double tmp[6];

	memcpy(tmp, mat[r1], 6 * sizeof(double));
	memcpy(mat[r1], mat[r2], 6 * sizeof(double));
	memcpy(mat[r2], tmp, 6 * sizeof(double));
}

/*!
 * Swap rows in matrix
 *
 * @param[in,out] mat Array representing 9x9 matrix
 * @param[in] r1 Index of the first row to swap
 * @param[in] r2 Index of the second row to swap
 */
void row_swap_9x9(double mat[9][9], int r1, int r2)
{
	double tmp[9];

	memcpy(tmp, mat[r1], 9 * sizeof(double));
	memcpy(mat[r1], mat[r2], 9 * sizeof(double));
	memcpy(mat[r2], tmp, 9 * sizeof(double));
}


/*!
 * Swap rows in matrix
 *
 * @param[in,out] mat Array representing 12x12 matrix
 * @param[in] r1 Index of the first row to swap
 * @param[in] r2 Index of the second row to swap
 */
void row_swap_12x12(double mat[12][12], int r1, int r2)
{
	double tmp[12];

	memcpy(tmp, mat[r1], 12 * sizeof(double));
	memcpy(mat[r1], mat[r2], 12 * sizeof(double));
	memcpy(mat[r2], tmp, 12 * sizeof(double));
}


/*!
 * Swap rows in matrix
 *
 * @param[in,out] mat Array representing 13x13 matrix
 * @param[in] r1 Index of the first row to swap
 * @param[in] r2 Index of the second row to swap
 */
void row_swap_13x13(double mat[13][13], int r1, int r2)
{
	double tmp[13];

	memcpy(tmp, mat[r1], 13 * sizeof(double));
	memcpy(mat[r1], mat[r2], 13 * sizeof(double));
	memcpy(mat[r2], tmp, 13 * sizeof(double));
}


/*!
 * Swap rows in matrix
 *
 * @param[in,out] mat Array representing nxn matrix
 * @param[in] r1 Index of the first row to swap
 * @param[in] r2 Index of the second row to swap
 */
void row_swap_15x15(double mat[15][15], int r1, int r2)
{
	double tmp[15];

	memcpy(tmp, mat[r1], 15 * sizeof(double));
	memcpy(mat[r1], mat[r2], 15 * sizeof(double));
	memcpy(mat[r2], tmp, 15 * sizeof(double));
}


/*!
 * Column operation
 *
 *  rowstart is used to avoid calculation for some rows (to save computation time)
 *
 * mat[rowstart:][c1] += fac*mat[rowstart:][c2]
 *
 * @param[in,out] mat Array representing 6x6 matrix
 * @param[in] c1 Index of the column to operate on
 * @param[in] c2 Index of the column to add to column c1
 * @param[in] fac Factor to multiply column c2 with before adding to c1
 * @param[in] rowstart First row index to operate on for column c1 (leave rows before rowstart unaffected)
 */
void col_op(double mat[6][6], int c1, int c2, double fac, int rowstart)
{
	int i;

	for (i = rowstart; i < 6; i++) {
		mat[i][c1] += fac*mat[i][c2];
	}
}


/*!
 * Calculation of determinant of 6x6 matrix using Gaussian elimination
 *
 * Returns determinant of the matrix
 *
 * Will return 0.0 if any diagonal element after LU-factorization is smaller than 1e-12
 *
 * @param[in] mat Array representing 6x6 matrix
 */
double determinant_66(const double mat[6][6])
{
	int i, j, largest;
	double m_[6][6];
	double fac, det;

	memcpy(m_, mat, 6 * 6 * sizeof(double));
	det = 1.0;
	for (i = 0; i < 5; i++) {
		/* Find largest element */
		largest = i;
		for (j = largest + 1; j < 6; j++) {
			if (fabs(m_[j][i]) > fabs(m_[largest][i])) {
				largest = j;
			}
		}
		/* If largest value is zero, the determinant is zero */
		if (fabs(m_[largest][i]) < 1e-12) {
			return 0.0;
		}
		/* swap row if necessary (swapping a row changes the sign) */
		if (largest > i) {
			row_swap(m_, i, largest);
			det *= -1;
		}

		for (j = i + 1; j < 6; j++) {
			fac = -m_[i][j]/m_[i][i];
			col_op(m_, j, i, fac, i+1);
		}

		det *= m_[i][i];
	}
	det *= m_[5][5];
	return det;
}


/*!
 * Row operation on linear equation system represented by mat*x = y
 *
 * colstart is used to avoid calculation for some columns (to save computation time)
 *
 * mat[r1][colstart:] += fac*mat[r2][colstart:]
 * also y[r1] += fac*y[r2]
 *
 * @param[in,out] mat Array representing 6x6 matrix
 * @param[in] y Array representing right hand side of linear equations system
 * @param[in] r1 Index of row to operate on
 * @param[in] r2 Index of row to add to row r1
 * @param[in] fac Factor to multiply with row r2 before adding to row r1
 * @param[in] colstart Index of first column in row r1 to change
 */
void row_op(double mat[6][6], double y[6], int r1, int r2, double fac, int colstart)
{
	int i;

	for (i = colstart; i < 6; i++) {
		mat[r1][i] += fac*mat[r2][i];
	}
	y[r1] += fac*y[r2];
}

/*! Row operation for 9x9 matrix, see further doc for row_op */
void row_op_9x9(double mat[9][9], double y[9], int r1, int r2,
		double fac, int colstart)
{
	int i;

	for (i = colstart; i < 9; i++)
		mat[r1][i] += fac*mat[r2][i];
	y[r1] += fac*y[r2];
}


/*! Row operation for 12x12 matrix, see further doc for row_op */
void row_op_12x12(double mat[12][12], double y[12], int r1, int r2,
		double fac, int colstart)
{
	int i;

	for (i = colstart; i < 12; i++)
		mat[r1][i] += fac*mat[r2][i];
	y[r1] += fac*y[r2];
}


/*! Row operation for 13x13 matrix, see further doc for row_op */
void row_op_13x13(double mat[13][13], double y[13], int r1, int r2,
		double fac, int colstart)
{
	int i;

	for (i = colstart; i < 13; i++)
		mat[r1][i] += fac*mat[r2][i];
	y[r1] += fac*y[r2];
}


/*! Row operation for nxn matrix, see further doc for row_op */
void row_op_15x15(double mat[15][15], double y[15], int r1, int r2,
		double fac, int colstart)
{
	int i;

	for (i = colstart; i < 15; i++)
		mat[r1][i] += fac*mat[r2][i];
	y[r1] += fac*y[r2];
}


/*!
 * Solve 6x6 linear equation system using Gaussian elimination
 *
 * The function aims to find sol in mat*sol=y
 *
 * 1 will be returned if the equation system is singular
 *
 * @param[in] mat Array representing 6x6 matrix
 * @param[in] y Array representing right hand side of linear equations system
 * @param[out] sol Calculated solution
 */
int solve_66_gauss_elim(const double mat[6][6], const double y[6], double sol[6])
{
	int i, j, largest;
	double m_[6][6];
	double fac;
	double smallest_pivot = 1.0;

	memcpy(m_, mat, 6 * 6 * sizeof(double));
	memcpy(sol, y, 6 * sizeof(double));

	for (i = 0; i < 5; i++) {
		/* Find largest element */
		largest = i;
		for (j = largest + 1; j < 6; j++) {
			if (fabs(m_[j][i]) > fabs(m_[largest][i])) {
				largest = j;
			}
		}
		/* If largest value is zero, the determinant is zero -> no solution */
		if (fabs(m_[largest][i]) < 1e-12) {
			return SINGULAR_SYSTEM;
		} else if (fabs(m_[largest][i]) < smallest_pivot) {
			smallest_pivot = fabs(m_[largest][i]);
		}
		/* swap row if necessary */
		if (largest > i) {
			row_swap(m_, i, largest);
			fac = sol[i];
			sol[i] = sol[largest];
			sol[largest] = fac;
		}

		for (j = i + 1; j < 6; j++) {
			fac = -m_[j][i]/m_[i][i];
			row_op(m_, sol, j, i, fac, i+1);
		}
	}

	/* Check that also the element in the lower right corner is non-zero */
	if (fabs(m_[5][5]) < 1e-12) {
		return SINGULAR_SYSTEM;
	} else if (fabs(m_[5][5]) < smallest_pivot) {
		smallest_pivot = fabs(m_[5][5]);
	}

	/* Back substitution to find solution */
	sol[5] /= m_[5][5];
	for (i = 4; i >= 0; i--) {
		for (j = 5; j > i; j--) {
			sol[i] -= m_[i][j]*sol[j];
		}
		sol[i] /= m_[i][i];
	}

	if (smallest_pivot < 4e-3)
		return CLOSE_TO_SINGULARITY;
	if (smallest_pivot < 1e-2)
		return IN_VICINITY_OF_SINGULARITY;
	return SOLUTION_OK;
}

/*!
 * Solve 9x9 linear equation system using Gaussian elimination
 *
 * The function aims to find sol in mat*sol=y
 *
 * 1 will be returned if the equation system is singular
 *
 * @param[in] mat Pointer to array representing nxn matrix
 * @param[in] y Pointer to Array representing right hand side of linear equations system
 * @param[in] n Size of system to solve
 * @param[out] sol Calculated solution
 */
int solve_9x9_gauss_elim(const double mat[9][9], const double y[9], double sol[9])
{
	int i, j, largest;
	double m_[9][9];
	double fac;

	int n = 9;

	memcpy(m_, mat, n * n * sizeof(double));
	memcpy(sol, y, n * sizeof(double));

	for (i = 0; i < n-1; i++) {
		/* Find largest element */
		largest = i;
		for (j = largest+1; j < n; j++)
			if (fabs(m_[j][i]) > fabs(m_[largest][i]))
				largest = j;

		/* If largest value is zero, the determinant is zero -> no solution */
		if (fabs(m_[largest][i]) < 1e-12)
			return 1;

		/* swap row if necessary */
		if (largest > i) {
			row_swap_9x9(m_, i, largest);
			fac = sol[i];
			sol[i] = sol[largest];
			sol[largest] = fac;
		}

		for (j = i+1; j < n; j++) {
			fac = -m_[j][i]/m_[i][i];
			row_op_9x9(m_, sol, j, i, fac, i+1);
		}
	}

	/* Check that also the element in the lower right corner is non-zero */
	if (fabs(m_[n-1][n-1]) < 1e-12)
		return 1;

	/* Back substitution to find solution */
	sol[n-1] /= m_[n-1][n-1];
	for (i = n-2; i >= 0; i--) {
		for (j = n-1; j > i; j--)
			sol[i] -= m_[i][j]*sol[j];
		sol[i] /= m_[i][i];
	}

	return 0;
}


/*!
 * Solve 12x12 linear equation system using Gaussian elimination
 *
 * The function aims to find sol in mat*sol=y
 *
 * 1 will be returned if the equation system is singular
 *
 * @param[in] mat Pointer to array representing nxn matrix
 * @param[in] y Pointer to Array representing right hand side of linear equations system
 * @param[in] n Size of system to solve
 * @param[out] sol Calculated solution
 */
int solve_12x12_gauss_elim(const double mat[12][12], const double y[12], double sol[12])
{
	int i, j, largest;
	double m_[12][12];
	double fac;

	int n = 12;

	memcpy(m_, mat, n * n * sizeof(double));
	memcpy(sol, y, n * sizeof(double));

	for (i = 0; i < n-1; i++) {
		/* Find largest element */
		largest = i;
		for (j = largest+1; j < n; j++)
			if (fabs(m_[j][i]) > fabs(m_[largest][i]))
				largest = j;

		/* If largest value is zero, the determinant is zero -> no solution */
		if (fabs(m_[largest][i]) < 1e-12)
			return 1;

		/* swap row if necessary */
		if (largest > i) {
			row_swap_12x12(m_, i, largest);
			fac = sol[i];
			sol[i] = sol[largest];
			sol[largest] = fac;
		}

		for (j = i+1; j < n; j++) {
			fac = -m_[j][i]/m_[i][i];
			row_op_12x12(m_, sol, j, i, fac, i+1);
		}
	}

	/* Check that also the element in the lower right corner is non-zero */
	if (fabs(m_[n-1][n-1]) < 1e-12)
		return 1;

	/* Back substitution to find solution */
	sol[n-1] /= m_[n-1][n-1];
	for (i = n-2; i >= 0; i--) {
		for (j = n-1; j > i; j--)
			sol[i] -= m_[i][j]*sol[j];
		sol[i] /= m_[i][i];
	}

	return 0;
}


/*!
 * Solve 13x13 linear equation system using Gaussian elimination
 *
 * The function aims to find sol in mat*sol=y
 *
 * 1 will be returned if the equation system is singular
 *
 * @param[in] mat Pointer to array representing nxn matrix
 * @param[in] y Pointer to Array representing right hand side of linear equations system
 * @param[in] n Size of system to solve
 * @param[out] sol Calculated solution
 */
int solve_13x13_gauss_elim(const double mat[13][13], const double y[13], double sol[13])
{
	int i, j, largest;
	double m_[13][13];
	double fac;

	int n = 13;

	memcpy(m_, mat, n * n * sizeof(double));
	memcpy(sol, y, n * sizeof(double));

	for (i = 0; i < n-1; i++) {
		/* Find largest element */
		largest = i;
		for (j = largest+1; j < n; j++)
			if (fabs(m_[j][i]) > fabs(m_[largest][i]))
				largest = j;

		/* If largest value is zero, the determinant is zero -> no solution */
		if (fabs(m_[largest][i]) < 1e-12)
			return 1;

		/* swap row if necessary */
		if (largest > i) {
			row_swap_13x13(m_, i, largest);
			fac = sol[i];
			sol[i] = sol[largest];
			sol[largest] = fac;
		}

		for (j = i+1; j < n; j++) {
			fac = -m_[j][i]/m_[i][i];
			row_op_13x13(m_, sol, j, i, fac, i+1);
		}
	}

	/* Check that also the element in the lower right corner is non-zero */
	if (fabs(m_[n-1][n-1]) < 1e-12)
		return 1;

	/* Back substitution to find solution */
	sol[n-1] /= m_[n-1][n-1];
	for (i = n-2; i >= 0; i--) {
		for (j = n-1; j > i; j--)
			sol[i] -= m_[i][j]*sol[j];
		sol[i] /= m_[i][i];
	}

	return 0;
}


/*!
 * Solve 15x15 linear equation system using Gaussian elimination
 *
 * The function aims to find sol in mat*sol=y
 *
 * 1 will be returned if the equation system is singular
 *
 * @param[in] mat Pointer to array representing nxn matrix
 * @param[in] y Pointer to Array representing right hand side of linear equations system
 * @param[out] sol Calculated solution
 */
int solve_15x15_gauss_elim(const double mat[15][15], const double y[15], double sol[15])
{
	int i, j, largest;
	double m_[15][15];
	double fac;

	int n = 15;

	// memcpy(m_, mat, n * n * sizeof(double));
	for (i = 0; i < n; i++) {
		for (j = 0; j < n; j++) {
			m_[i][j] = mat[i][j];
		}
	}
	memcpy(sol, y, n * sizeof(double));

	for (i = 0; i < n-1; i++) {
		/* Find largest element */
		largest = i;
		for (j = largest+1; j < n; j++)
			if (fabs(m_[j][i]) > fabs(m_[largest][i]))
				largest = j;

		/* If largest value is zero, the determinant is zero -> no solution */
		if (fabs(m_[largest][i]) < 1e-12)
			return 1;

		/* swap row if necessary */
		if (largest > i) {
			row_swap_15x15(m_, i, largest);
			fac = sol[i];
			sol[i] = sol[largest];
			sol[largest] = fac;
		}

		for (j = i+1; j < n; j++) {
			fac = -m_[j][i]/m_[i][i];
			row_op_15x15(m_, sol, j, i, fac, i+1);
		}
	}

	/* Check that also the element in the lower right corner is non-zero */
	if (fabs(m_[n-1][n-1]) < 1e-12)
		return 1;

	/* Back substitution to find solution */
	sol[n-1] /= m_[n-1][n-1];
	for (i = n-2; i >= 0; i--) {
		for (j = n-1; j > i; j--)
			sol[i] -= m_[i][j]*sol[j];
		sol[i] /= m_[i][i];
	}

	return 0;
}


/*!
 * Solve 6x6 linear equation system using damped least squares
 *
 * The damped pseudo-inverse is given as
 * mat_pseudoinv = inv(mat^T*mat + lambda*diag(vals))^-1 * mat^T
 *
 * where
 * vals = max(diag(mat^T*mat), 1.0)
 *
 * The function tries to calculate the solution to the linear equation system
 * (mat^T*mat + lambda*diag(vals))*sol=mat^T*y
 *
 * 1 will be returned if the equation system is singular
 *
 * lambda less than 1e-10 will be treated as zero
 *
 * @param[in] mat Array representing 6x6 matrix
 * @param[in] y Array representing right hand side of linear equations system
 * @param[in] lambda Damping factor
 * @param[out] sol Calculated solution
 */
int solve_66_6_damped_ls(const double mat[6][6], const double y[6], double lambda, double sol[6])
{
	double jtj[6][6];
	double vec[6];
	double tmp;
	int i, j, k;

	if (lambda < 1e-10) {
		return solve_66_gauss_elim(mat, y, sol);
	}

	memset(jtj, 0, 36 * sizeof(double));
	for (i = 0; i < 6; i++)
		for (j = 0; j < 6; j++)
			for (k = 0; k < 6; k++)
				jtj[i][j] += mat[k][i] * mat[k][j];

	for (i = 0; i < 6; i++) {
		tmp = jtj[i][i];
		if (tmp < 1.0) {
			tmp = 1.0;
		}
		jtj[i][i] += lambda*tmp;
		vec[i] = 0.0;
		for (j = 0; j < 6; j++) {
			vec[i] += mat[j][i]*y[j];
		}
	}

	return solve_66_gauss_elim(jtj, vec, sol);
}


/*!
 * Solve 9x9 linear equation system using damped least squares
 *
 * See further doc on solve_66_6_damped_ls
 */
int solve_9x9_9_damped_ls(const double mat[9][9], const double y[9],
		double lambda, double sol[9])
{
	double jtj[9][9];
	double vec[9];
	double tmp;
	int i, j, k;

	if (lambda < 1e-10)
		return solve_9x9_gauss_elim(mat, y, sol);

	memset(jtj, 0, 9 * 9 * sizeof(double));
	for (i = 0; i < 9; i++)
		for (j = 0; j < 9; j++)
			for (k = 0; k < 9; k++)
				jtj[i][j] += mat[k][i] * mat[k][j];

	for (i = 0; i < 9; i++) {
		tmp = jtj[i][i];
		if (tmp < 1.0) {
			tmp = 1.0;
		}
		jtj[i][i] += lambda*tmp;
		vec[i] = 0.0;
		for (j = 0; j < 9; j++)
			vec[i] += mat[j][i]*y[j];
	}

	return solve_9x9_gauss_elim(jtj, vec, sol);
}


/*!
 * Solve 12x12 linear equation system using damped least squares
 *
 * See further doc on solve_66_6_damped_ls
 */
int solve_12x12_12_damped_ls(const double mat[12][12], const double y[12],
		double lambda, double sol[12])
{
	double jtj[12][12];
	double vec[12];
	double tmp;
	int i, j, k;

	if (lambda < 1e-10)
		return solve_12x12_gauss_elim(mat, y, sol);

	memset(jtj, 0, 12 * 12 * sizeof(double));
	for (i = 0; i < 12; i++)
		for (j = 0; j < 12; j++)
			for (k = 0; k < 12; k++)
				jtj[i][j] += mat[k][i] * mat[k][j];

	for (i = 0; i < 12; i++) {
		tmp = jtj[i][i];
		if (tmp < 1.0) {
			tmp = 1.0;
		}
		jtj[i][i] += lambda*tmp;
		vec[i] = 0.0;
		for (j = 0; j < 12; j++)
			vec[i] += mat[j][i]*y[j];
	}

	return solve_12x12_gauss_elim(jtj, vec, sol);
}


/*!
 * Solve 13x13 linear equation system using damped least squares
 *
 * See further doc on solve_66_6_damped_ls
 */
int solve_13x13_13_damped_ls(const double mat[13][13], const double y[13],
		double lambda, double sol[13])
{
	double jtj[13][13];
	double vec[13];
	double tmp;
	int i, j, k;

	if (lambda < 1e-10)
		return solve_13x13_gauss_elim(mat, y, sol);

	memset(jtj, 0, 13 * 13 * sizeof(double));
	for (i = 0; i < 13; i++)
		for (j = 0; j < 13; j++)
			for (k = 0; k < 13; k++)
				jtj[i][j] += mat[k][i] * mat[k][j];

	for (i = 0; i < 13; i++) {
		tmp = jtj[i][i];
		if (tmp < 1.0) {
			tmp = 1.0;
		}
		jtj[i][i] += lambda*tmp;
		vec[i] = 0.0;
		for (j = 0; j < 13; j++)
			vec[i] += mat[j][i]*y[j];
	}

	return solve_13x13_gauss_elim(jtj, vec, sol);
}


/*!
 * Solve 15x15 linear equation system using damped least squares
 *
 * See further doc on solve_66_6_damped_ls
 */
int solve_15x15_15_damped_ls(const double mat[15][15], const double y[15],
		double lambda, double sol[15])
{
	double jtj[15][15];
	double vec[15];
	double tmp;
	int i, j, k;

	if (lambda < 1e-10)
		return solve_15x15_gauss_elim(mat, y, sol);

	memset(jtj, 0, 15 * 15 * sizeof(double));
	for (i = 0; i < 15; i++)
		for (j = 0; j < 15; j++)
			for (k = 0; k < 15; k++)
				jtj[i][j] += mat[k][i] * mat[k][j];

	for (i = 0; i < 15; i++) {
		tmp = jtj[i][i];
		if (tmp < 1.0) {
			tmp = 1.0;
		}
		jtj[i][i] += lambda*tmp;
		vec[i] = 0.0;
		for (j = 0; j < 15; j++)
			vec[i] += mat[j][i]*y[j];
	}

	return solve_15x15_gauss_elim(jtj, vec, sol);
}
