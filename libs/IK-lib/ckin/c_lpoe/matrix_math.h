/*
 * Copyright (C) 2017, 2018, 2019, 2020 Cognibotics
 *
 * Use or copying of this file or any part of it, without explicit
 * permission from Cognibotics is not allowed in any way.
 */
/*!
 * @file
 * @brief A collection of Matrix operations
 */
#ifndef MATRIX_MATH_H
#define MATRIX_MATH_H

/* 2x2 matrices */
void mul22_transpose(const double a[2][2], const double b[2][2], double out[2][2]);
void mul22_transpose2(const double a[2][2], const double b[2][2], double out[2][2]);

/* 3x3 matrices */
void add33(const double a[3][3], const double b[3][3], double out[3][3]);
void mul33_transpose(const double a[3][3], const double b[3][3], double out[3][3]);
void mul33_transpose2(const double a[3][3], const double b[3][3], double out[3][3]);
void mul33_1(const double a[3][3], double b, double out[3][3]);
void mul33(const double a[3][3], const double b[3][3], double out[3][3]);
void mul33_3(const double a[3][3], const double b[3], double out[3]);
void mul33transposed_3(const double a[3][3], const double b[3], double out[3]);

/* 4x4 matrices */
void mul44_3(const double a[4][4], const double b[3], double out[4][4]);
void diff_t44_pos(const double a[4][4], const double b[4][4], double out[3]);
void mul44(const double a[4][4], const double b[4][4], double out[4][4]);

/* rot part of t44 */
void mul_t44_rot_3(const double a[4][4], const double b[3], double out[3]);
void mul_t44_rot_transposed_3(const double a[4][4], const double b[3], double out[3]);
void mul_t44_rot_33(const double a[4][4], const double b[3][3], double out[3][3]);
void mul33_t44_rot_transpose(const double a[3][3], const double b[4][4], double out[3][3]);
void mul_t44_rot_transpose_33(const double a[4][4], const double b[3][3], double out[3][3]);

/* 6x6 matrices */
void mul66_6(const double a[6][6], const double b[6], double out[6]);
void mul66transposed_6(const double a[6][6], const double b[6], double out[6]);
void mul66(const double a[6][6], const double b[6][6], double out[6][6]);
void mul66_transpose(const double a[6][6], const double b[6][6], double out[6][6]);
void mul66_transpose2(const double a[6][6], const double b[6][6], double out[6][6]);

/* General matrices */
void eye(int n, double *out);

/* Vector operations */
double mul3_3(const double a[3], const double b[3]);
double norm3(const double a[3]);
void xprod(const double a[3], const double b[3], double out[3]);
double inner_prod66(const double a[6], const double b[6]);
void hat_hat_mul(const double a[3], const double b[3], double out[3][3]);


#endif
