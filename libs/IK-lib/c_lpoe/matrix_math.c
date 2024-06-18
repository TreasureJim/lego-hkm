/*
 * Copyright (C) 2017, 2018, 2019, 2020 Cognibotics
 *
 * Use or copying of this file or any part of it, without explicit
 * permission from Cognibotics is not allowed in any way.
 */
/*!
 * @file
 * @brief A collection of matrix and vector operations
 */
#ifdef CB_SYSTEM_HEADER
#include CB_SYSTEM_HEADER
#else
#include <stdlib.h>
#include <string.h>
#include <math.h>
#endif

#include "matrix_math.h"


/*!
 * Compute multiplication of 3x3 matrix with scalar value
 *
 * @param[in] a left operand, 3x3-matrix.
 * @param[in] b scalar factor.
 * @param[out] out Product.
 */
void mul33_1(const double a[3][3], double b, double out[3][3]){
	int i1, i2;

	for( i1 = 0; i1 < 3; i1++ ){
		for( i2 = 0; i2 < 3; i2++ ){
			out[i1][i2] = b*a[i1][i2];
		}
	}
}


/*!
 * Compute addition of two 3x3 matrices
 *
 * @param[in] a first term, 3x3-matrix.
 * @param[in] b second term, 3x3-matrix.
 * @param[out] out Sum, 3x3-matrix.
 */
void add33(const double a[3][3], const double b[3][3], double out[3][3]){
	int i1, i2;

	for( i1 = 0; i1 < 3; i1++ ){
		for( i2 = 0; i2 < 3; i2++ ){
			out[i1][i2] = a[i1][i2] + b[i1][i2];
		}
	}
}


/*!
 * Compute the product of two 3x3 matrices, second matrix is transposed.
 *
 * @param[in] a Left operand.
 * @param[in] b Right operand.
 * @param[out] out Product.
 */
void mul33_transpose(const double a[3][3], const double b[3][3], double out[3][3])
{
	int i, j, k;

	memset(out, 0, 9 * sizeof(double));
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			for (k = 0; k < 3; k++)
				out[i][j] += a[i][k] * b[j][k];
}


/*!
 * Compute the product of a 3x3 matrix with rotation part of t44, rotation matrix is transposed.
 *
 * @param[in] a Left operand.
 * @param[in] b Right operand.
 * @param[out] out Product.
 */
void mul33_t44_rot_transpose(const double a[3][3], const double b[4][4], double out[3][3])
{
	int i, j, k;

	memset(out, 0, 9 * sizeof(double));
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			for (k = 0; k < 3; k++)
				out[i][j] += a[i][k] * b[j][k];
}


/*!
 * Compute the product of two 3x3 matrices, first matrix is transposed.
 *
 * @param[in] a Left operand.
 * @param[in] b Right operand.
 * @param[out] out Product.
 */
void mul33_transpose2(const double a[3][3], const double b[3][3], double out[3][3])
{
	int i, j, k;

	memset(out, 0, 9 * sizeof(double));
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			for (k = 0; k < 3; k++)
				out[i][j] += a[k][i] * b[k][j];
}


/*!
 * Compute the product of the rotation part of a t44 with a 3x3 matrix
   rotation matrix is transposed.
 *
 * @param[in] a Left operand.
 * @param[in] b Right operand.
 * @param[out] out Product.
 */
void mul_t44_rot_transpose_33(const double a[4][4], const double b[3][3], double out[3][3])
{
	int i, j, k;

	memset(out, 0, 9 * sizeof(double));
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			for (k = 0; k < 3; k++)
				out[i][j] += a[k][i] * b[k][j];
}


/*!
 * Compute the product of two 2x2 matrices, second matrix is transposed.
 *
 * @param[in] a Left operand.
 * @param[in] b Right operand.
 * @param[out] out Product.
 */
void mul22_transpose(const double a[2][2], const double b[2][2], double out[2][2])
{
	int i, j, k;

	memset(out, 0, 4 * sizeof(double));
	for (i = 0; i < 2; i++)
		for (j = 0; j < 2; j++)
			for (k = 0; k < 2; k++)
				out[i][j] += a[i][k] * b[j][k];
}


/*!
 * Compute the product of two 2x2 matrices, first matrix is transposed.
 *
 * @param[in] a Left operand.
 * @param[in] b Right operand.
 * @param[out] out Product.
 */
void mul22_transpose2(const double a[2][2], const double b[2][2], double out[2][2])
{
	int i, j, k;

	memset(out, 0, 4 * sizeof(double));
	for (i = 0; i < 2; i++)
		for (j = 0; j < 2; j++)
			for (k = 0; k < 2; k++)
				out[i][j] += a[k][i] * b[k][j];
}


/*!
 * Compute simplified t44 multiplication, where second t44 has identity rotation matrix
 * and is represented by a three element position vector.
 *
 * t44_1 = a = [R, p; 0, 1]
 * t44_2 = [I, b;0, 1]
 * out = t44_1 * t44_2 = [R, R*b + p; 0, 1]
 *
 * @param[in] a left operand, t44 matrix.
 * @param[in] b Right operand, position vector of t44 matrix.
 * @param[out] out Product.
 */
void mul44_3(const double a[4][4], const double b[3], double out[4][4])
{
	int j, k;
	memset(out, 0, 16 * sizeof(double));
	for (j = 0; j < 3; j++) {
		out[j][3] = a[j][3];
		for (k = 0; k < 3; k++){
			out[j][3] += a[j][k] * b[k];
			out[j][k] = a[j][k];
		}
	}
	out[3][3] = 1.0;
}


/*!
 * Compute the difference of the positions in two t44s.
 *
 * @param[in] a Left t44.
 * @param[in] b Right t44.
 * @param[out] out Difference.
 */
void diff_t44_pos(const double a[4][4], const double b[4][4], double out[3])
{
	int j;

	for (j = 0; j < 3; j++)
		out[j] = a[j][3] - b[j][3];
}


/*!
 * Compute norm of a 3 element vector
 *
 * @param[in] a 3-element vector.
 * return norm
 */
double norm3(const double a[3])
{
	double res = 0;

	res = mul3_3(a, a);
	return sqrt(res);
}


/*!
 * Create an NxN identity matrix.
 *
 * @param[in] n The number of rows and columns.
 * @param[out] out Pointer to array.
 */
void eye(int n, double *out)
{
	int i, j;

	for (i = 0; i < n; i++)
		for (j = 0; j < n; j++)
			*(out + (i*n + j)) = ((i==j) ? 1 : 0);
}

/*!
 * Compute the cross product
 *
 * @param[in] a Left operand.
 * @param[in] b Right operand.
 * @param[out] out Cross product.
 */
void xprod(const double a[3], const double b[3], double out[3])
{
	out[0] = a[1]*b[2] - a[2]*b[1];
	out[1] = a[2]*b[0] - a[0]*b[2];
	out[2] = a[0]*b[1] - a[1]*b[0];
}

/*!
 * Compute the product of two 4x4 matrices.
 *
 * @param[in] a Left operand.
 * @param[in] b Right operand.
 * @param[out] out Product.
 */
void mul44(const double a[4][4], const double b[4][4], double out[4][4])
{
	int i, j, k;

	memset(out, 0, 16 * sizeof(double));
	for (i = 0; i < 4; i++)
		for (j = 0; j < 4; j++)
			for (k = 0; k < 4; k++)
				out[i][j] += a[i][k] * b[k][j];
}

/*!
 * Compute the product of two 4x4 matrices, first matrix is transposed.
 *
 * This means that the product a^T*b is calculated.
 *
 * @param[in] a Left operand.
 * @param[in] b Right operand.
 * @param[out] out Product.
 */
void mul44_transpose1(const double a[4][4], const double b[4][4], double out[4][4])
{
	int i, j, k;

	memset(out, 0, 16 * sizeof(double));
	for (i = 0; i < 4; i++)
		for (j = 0; j < 4; j++)
			for (k = 0; k < 4; k++)
				out[i][j] += a[k][i] * b[k][j];
}

/*!
 * Compute the product of two 4x4 matrices, second matrix is transposed.
 *
 * This means that the product a*b^T is calculated.
 *
 * @param[in] a Left operand.
 * @param[in] b Right operand.
 * @param[out] out Product.
 */
void mul44_transpose2(const double a[4][4], const double b[4][4], double out[4][4])
{
	int i, j, k;

	memset(out, 0, 16 * sizeof(double));
	for (i = 0; i < 4; i++)
		for (j = 0; j < 4; j++)
			for (k = 0; k < 4; k++)
				out[i][j] += a[i][k] * b[j][k];
}

/*!
 * Compute the product of two 3x3 matrices.
 *
 * @param[in] a Left operand.
 * @param[in] b Right operand.
 * @param[out] out Product.
 */
void mul33(const double a[3][3], const double b[3][3], double out[3][3])
{
	int i, j, k;

	memset(out, 0, 9 * sizeof(double));
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			for (k = 0; k < 3; k++)
				out[i][j] += a[i][k] * b[k][j];
}

/*!
 * Compute the product of the rotation part of a t44 with a 3x3 matrix.
 *
 * @param[in] a Left operand.
 * @param[in] b Right operand.
 * @param[out] out Product.
 */
void mul_t44_rot_33(const double a[4][4], const double b[3][3], double out[3][3])
{
	int i, j, k;

	memset(out, 0, 9 * sizeof(double));
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			for (k = 0; k < 3; k++)
				out[i][j] += a[i][k] * b[k][j];
}

/*!
 * Compute the product of a 3x3 matrix and a 3 element vector.
 *
 * @param[in] a Left operand.
 * @param[in] b Right operand.
 * @param[out] out Product.
 */
void mul33_3(const double a[3][3], const double b[3], double out[3])
{
	int j, k;

	for (j = 0; j < 3; j++) {
		out[j] = 0;
		for (k = 0; k < 3; k++)
			out[j] += a[j][k] * b[k];
	}
}

/*!
 * Compute the product of a 3x3 matrix and a 3 element vector.
 * First transpose the 3x3 matrix.
 *
 * @param[in] a Left operand.
 * @param[in] b Right operand.
 * @param[out] out Product.
 */
void mul33transposed_3(const double a[3][3], const double b[3], double out[3])
{
	int j, k;

	for (j = 0; j < 3; j++) {
		out[j] = 0;
		for (k = 0; k < 3; k++)
			out[j] += a[k][j] * b[k];
	}
}

/*!
 * Compute the product of two 3 element vectors.
 *
 * @param[in] a Left operand.
 * @param[in] b Right operand.
 * return Product
 */
double mul3_3(const double a[3], const double b[3])
{
	double res = 0;
	int i;

	for (i = 0; i < 3; i++)
		res += a[i] * b[i];

	return res;
}


/*!
 * Compute the product of a 6x6 matrix and a 6 element vector.
 *
 * @param[in] a Left operand.
 * @param[in] b Right operand.
 * @param[out] out Product.
 */
void mul66_6(const double a[6][6], const double b[6], double out[6])
{
	int j, k;

	for (j = 0; j < 6; j++) {
		out[j] = 0;
		for (k = 0; k < 6; k++)
			out[j] += a[j][k] * b[k];
	}
}

/*!
 * Compute the product of a 6x6 matrix (transposed) and a 6 element vector.
 *
 * @param[in] a Left operand.
 * @param[in] b Right operand.
 * @param[out] out Product.
 */
void mul66transposed_6(const double a[6][6], const double b[6], double out[6])
{
	int j, k;

	for (j = 0; j < 6; j++) {
		out[j] = 0;
		for (k = 0; k < 6; k++)
			out[j] += a[k][j] * b[k];
	}
}

/*!
 * Compute the product of two 6x6 matrices.
 *
 * @param[in] a Left operand.
 * @param[in] b Right operand.
 * @param[out] out Product.
 */
void mul66(const double a[6][6], const double b[6][6], double out[6][6])
{
	int i, j, k;

	memset(out, 0, 36 * sizeof(double));
	for (i = 0; i < 6; i++)
		for (j = 0; j < 6; j++)
			for (k = 0; k < 6; k++)
				out[i][j] += a[i][k] * b[k][j];
}

/*!
 * Compute the product of two 6x6 matrices, second matrix is transposed.
 *
 * @param[in] a Left operand.
 * @param[in] b Right operand.
 * @param[out] out Product.
 */
void mul66_transpose(const double a[6][6], const double b[6][6], double out[6][6])
{
	int i, j, k;

	memset(out, 0, 36 * sizeof(double));
	for (i = 0; i < 6; i++)
		for (j = 0; j < 6; j++)
			for (k = 0; k < 6; k++)
				out[i][j] += a[i][k] * b[j][k];
}

/*!
 * Compute the product of two 6x6 matrices, first matrix is transposed.
 *
 * @param[in] a Left operand.
 * @param[in] b Right operand.
 * @param[out] out Product.
 */
void mul66_transpose2(const double a[6][6], const double b[6][6], double out[6][6])
{
	int i, j, k;

	memset(out, 0, 36 * sizeof(double));
	for (i = 0; i < 6; i++)
		for (j = 0; j < 6; j++)
			for (k = 0; k < 6; k++)
				out[i][j] += a[k][i] * b[k][j];
}


/*!
 * Compute the inner product of two 6-vectors.
 *
 * @param[in] a First vector.
 * @param[in] b Second vector.
 * return inner product of and b
 */
double inner_prod66(const double a[6], const double b[6])
{
	int i;
	double res = 0;

	for (i = 0; i < 6; i++)
		res += a[i]*b[i];
	return res;
}


/*!
 * Compute the product of a the rotation matrix of a t44 and a 3 element vector.
 *
 * @param[in] a Left operand.
 * @param[in] b Right operand.
 * @param[out] out Product.
 */
void mul_t44_rot_3(const double a[4][4], const double b[3], double out[3])
{
	int j, k;
	for (j = 0; j < 3; j++) {
		out[j] = 0;
		for (k = 0; k < 3; k++)
			out[j] += a[j][k] * b[k];
	}
}


/*!
 * Compute the product of the rotation matrix of a t44 and the position part of a t44.
 *
 * @param[in] a Left operand.
 * @param[in] b Right operand.
 * @param[out] out Product.
 */
void mul_t44_rot_t44_p(const double a[4][4], const double b[4][4], double out[3])
{
	int j, k;
	for (j = 0; j < 3; j++) {
		out[j] = 0;
		for (k = 0; k < 3; k++)
			out[j] += a[j][k] * b[k][3];
	}
}

/*!
 * Compute the product of a the rotation matrix of a t44 and a 3 element vector.
 * First transpose the rotation matrix.
 *
 * @param[in] a Left operand.
 * @param[in] b Right operand.
 * @param[out] out Product.
 */
void mul_t44_rot_transposed_3(const double a[4][4], const double b[3], double out[3])
{
	int j, k;
	for (j = 0; j < 3; j++) {
		out[j] = 0;
		for (k = 0; k < 3; k++)
			out[j] += a[k][j] * b[k];
	}
}

/*!
 * Compute the product hat(a) * hat(b).
 *
 * @param[in] a Left operand.
 * @param[in] b Right operand.
 * @param[out] out Product.
 */
void hat_hat_mul(const double a[3], const double b[3], double out[3][3])
{
	int j, k;
	double a1b1 = a[0]*b[0];
	double a2b2 = a[1]*b[1];
	double a3b3 = a[2]*b[2];
	out[0][0] = -a2b2 - a3b3;
	out[1][1] = -a1b1 - a3b3;
	out[2][2] = -a1b1 - a2b2;
	for (j = 0; j < 3; j++) {
		for (k = j+1; k < 3; k++) {
			out[j][k] = b[j] * a[k];
			out[k][j] = b[k] * a[j];
		}
	}
}

