/*
 * Copyright (C) 2018, 2019 Cognibotics
 *
 * Use or copying of this file or any part of it, without explicit
 * permission from Cognibotics is not allowed in any way.
 */
#ifdef CB_SYSTEM_HEADER
#include CB_SYSTEM_HEADER
#else
#include <math.h>
#include <string.h>
#endif

#include "common.h"
#include "c_lpoe/lpoe.h"
#include "c_lpoe/matrix_math.h"

/*!
 * Compute Euler zyz intrinsic angles from a t44
 *
 * Reference: cb.tfm.tf2euler_zyz_intrinsic
 *
 * @param[in] t44 t44 to convert to Euler angles
 * @param[out] out Euler angles
 */
void tf2euler_zyz_intrinsic(const double t44[4][4], double out[3])
{
	double eps = 1e-14;

	if (t44[2][2] > (1-eps)) {
		out[0] = 0;
		out[1] = 0;
		out[2] = atan2(t44[1][0], t44[1][1]);
	} else if (t44[2][2] < -(1-eps)) {
		out[0] = 0;
		out[1] = M_PI;
		out[2] = -atan2(-t44[1][0], t44[1][1]);
	} else {
		out[0] = atan2(t44[1][2], t44[0][2]);
		out[1] = acos(t44[2][2]);
		out[2] = atan2(t44[2][1], -t44[2][0]);
	}
}


/*!
 * Compute Euler zyx intrinsic angles from a t44
 *
 * Reference: cb.tfm.tf2euler_zyx_intrinsic
 *
 * @param[in] t44 t44 to convert to Euler angles
 * @param[out] out Euler angles
 */
void tf2euler_zyx_intrinsic(const double t44[4][4], double out[3])
{
	double eps = 1e-14;

	if (t44[2][0] < -(1 - eps)) {
		out[0] = 0;
		out[1] = M_PI/2;
		out[2] = atan2(t44[0][1], t44[0][2]);
	} else if (t44[2][0] > (1 - eps)) {
		out[0] = 0;
		out[1] = -M_PI/2;
		out[2] = atan2(-t44[0][1], -t44[0][2]);
	} else {
		out[0] = atan2(t44[1][0], t44[0][0]);
		out[1] = asin(-t44[2][0]);
		out[2] = atan2(t44[2][1], t44[2][2]);
	}
}


/*!
 * Compute t44 from Euler ZYZ intrinsic angles
 *
 * Reference: cb.tfm.euler_zyz_intrinsic2tf
 *
 * @param[in] euler_zyz Euler angles
 * @param[out] t44 t44 for given Euler angles
 */
void euler_zyz_intrinsic2tf(const double euler_zyz[3], double t44[4][4])
{
	double z_rot[4][4];
	double y_rot[4][4];
	double tmp[4][4];

	rot_z(euler_zyz[0], z_rot);
	rot_y(euler_zyz[1], y_rot);
	mul44(z_rot, y_rot, tmp);
	rot_z(euler_zyz[2], z_rot);
	mul44(tmp, z_rot, t44);
}


/*!
 * Compute t44 from Euler ZYX intrinsic angles
 *
 * Reference: cb.tfm.euler_zyx_intrinsic2tf
 *
 * @param[in] euler_zyx Euler angles
 * @param[out] t44 t44 for given Euler angles
 */
void euler_zyx_intrinsic2tf(const double euler_zyx[3], double t44[4][4])
{
	double zx_rot[4][4];
	double y_rot[4][4];
	double tmp[4][4];

	rot_z(euler_zyx[0], zx_rot);
	rot_y(euler_zyx[1], y_rot);
	mul44(zx_rot, y_rot, tmp);
	rot_x(euler_zyx[2], zx_rot);
	mul44(tmp, zx_rot, t44);
}


/*!
 * Compute quaternion from t44 rotation matrix
 *
 * @param[in] t44 t44 matrix
 * @param[out] q quaternion
 */
void tf2quat(const double t44[4][4], double q[4])
{
	double tr = t44[0][0] + t44[1][1] + t44[2][2];  /* trace */
	double s;

	if (tr > 0) {
		s = sqrt(tr + 1)*2;
		q[0] = 0.25 * s;
		q[1] = (t44[2][1] - t44[1][2])/s;
		q[2] = (t44[0][2] - t44[2][0])/s;
		q[3] = (t44[1][0] - t44[0][1])/s;
	} else if (t44[0][0] > t44[1][1] && t44[0][0] > t44[2][2]) {
		s = sqrt(1 + t44[0][0] - t44[1][1] - t44[2][2])*2;
		q[0] = (t44[2][1] - t44[1][2])/s;
		q[1] = 0.25 * s;
		q[2] = (t44[0][1] + t44[1][0])/s;
		q[3] = (t44[0][2] + t44[2][0])/s;
	} else if (t44[1][1] > t44[2][2]) {
		s = sqrt(1 + t44[1][1] - t44[0][0] - t44[2][2])*2;
		q[0] = (t44[0][2] - t44[2][0])/s;
		q[1] = (t44[0][1] + t44[1][0])/s;
		q[2] = 0.25 * s;
		q[3] = (t44[1][2] + t44[2][1])/s;
	} else {
		s = sqrt(1 + t44[2][2] - t44[0][0] - t44[1][1])*2;
		q[0] = (t44[1][0] - t44[0][1])/s;
		q[1] = (t44[0][2] + t44[2][0])/s;
		q[2] = (t44[1][2] + t44[2][1])/s;
		q[3] = 0.25 * s;
	}
}


/*!
 * Compute t44 (rotation matrix)  from quaternion
 *
 * @param[in] q quaternion
 * @param[out] t44 t44 matrix
 */
void quat2tf(const double q[4], double t44[4][4])
{
	eye(4, (double *) t44);

	t44[0][0] = 1 - 2*pow(q[2], 2) - 2*pow(q[3], 2);
	t44[0][1] = 2*(q[1]*q[2] - q[3]*q[0]);
	t44[0][2] = 2*(q[1]*q[3] + q[2]*q[0]);

	t44[1][0] = 2*(q[1]*q[2] + q[3]*q[0]);
	t44[1][1] = 1 - 2*pow(q[1], 2) - 2*pow(q[3], 2);
	t44[1][2] = 2*(q[2]*q[3] - q[1]*q[0]);

	t44[2][0] = 2*(q[1]*q[3] - q[2]*q[0]);
	t44[2][1] = 2*(q[2]*q[3] + q[1]*q[0]);
	t44[2][2] = 1 - 2*pow(q[1], 2) - 2*pow(q[2], 2);
}


/*!
 * Quaternion multiplication
 *
 * @param[in] q1 quaternion
 * @param[in] q2 quaternion
 * @param[out] prod Product of the two quaternions
 */
void quat_mul(const double q1[4], const double q2[4], double prod[4])
{
	prod[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
	prod[1] = q1[1]*q2[0] + q1[0]*q2[1] + q1[2]*q2[3] - q1[3]*q2[2];
	prod[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
	prod[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}


/*!
 * Quaternion inverse
 *
 * @param[in] q quaternion
 * @param[out] q_inv Inverse of quaternion
 */
void quat_inv(const double q[4], double q_inv[4])
{
	q_inv[0] = q[0];
	q_inv[1] = -q[1];
	q_inv[2] = -q[2];
	q_inv[3] = -q[3];
}


/*!
 * nominal inverse kinematics j2j3
 *
 * ref: cb.kin.model_lpoe.SixAxis._inv_pos_j2j3()
 *
 * @param[in] wcp_arm T44 representing the wrist center point (in the plane of j2 and j3).
 * @param[in] l1 Length of link between j2 and j3.
 * @param[in] l2 Length of link between j3 and wcp.
 * @param[in] elbow_down Should be 1 for the elbow down solution, 0 otherwise.
 * @param[out] th23 Values for joints 2 and 3.
 */
int inv_pos_j2j3(const double wcp_arm[4][4], double l1, double l2,
		int elbow_down, double th23[2])
{
	double x = wcp_arm[0][3];
	double z = wcp_arm[2][3];
	double dist2 = pow(x, 2) + pow(z, 2);
	double dist = sqrt(dist2);
	double th2[2];
	double th3[2];
	double sols[2][3];
	int count = 0;
	double x_test;
	double z_test;

	int unreach = 0;

	if (dist > (l1 + l2) || dist < fabs(l1 - l2))
		unreach = 1;

	if (!unreach) {
		double a = dist2 + pow(l1, 2) - pow(l2, 2);
		double px[2], pz[2];
		if (fabs(x) > fabs(z)) {
			double b = (a*z) / dist2;
			double c = (pow(a, 2) - pow(2*x*l1, 2)) / (4*dist2);
			double tmp = pow(b, 2)/4 - c;
			double tmp_sqrt = 0.0;
			if (tmp >= 0)
				tmp_sqrt = sqrt(tmp);
			pz[0] = b/2 + tmp_sqrt;
			pz[1] = b/2 - tmp_sqrt;
			px[0] = (0.5*a - z*pz[0]) / x;
			px[1] = (0.5*a - z*pz[1]) / x;
		} else {
			double b = (a*x)/dist2;
			double c = (pow(a, 2) - pow(2*z*l1, 2)) / (4*dist2);
			double tmp = pow(b, 2)/4 - c;
			double tmp_sqrt = 0.0;
			if (tmp >= 0)
				tmp_sqrt = sqrt(tmp);
			px[0] = b/2 + tmp_sqrt;
			px[1] = b/2 - tmp_sqrt;
			pz[0] = (0.5*a - x*px[0]) / z;
			pz[1] = (0.5*a - x*px[1]) / z;
		}

		th2[0] = atan2(-pz[0], px[0]);
		th2[1] = atan2(-pz[1], px[1]);
		th3[0] = atan2(pz[0] - z, x - px[0]) - th2[0];
		th3[1] = atan2(pz[1] - z, x - px[1]) - th2[1];

		x_test = l1*cos(th2[0]) + l2*cos(th2[0] + th3[0]);
		z_test = -l1*sin(th2[0]) - l2*sin(th2[0] + th3[0]);
		if (fabs(x - x_test) < 1e-7 && fabs(z - z_test) < 1e-7) {
			double angle1 = atan2(px[0], pz[0]) + atan2(x - px[0], pz[0] - z);
			if (angle1 < 0)
				angle1 += 2*M_PI;
			sols[0][0] = th2[0];
			sols[0][1] = th3[0];
			sols[0][2] = angle1;
			count++;
		}
		/* Check if the solutions are different, otherwise don't add the second. */
		if ((fabs(th2[0] - th2[1]) + fabs(th3[0] - th3[1])) > 1e-6) {
			x_test = l1*cos(th2[1]) + l2*cos(th2[1] + th3[1]);
			z_test = -l1*sin(th2[1]) - l2*sin(th2[1] + th3[1]);
			if (fabs(x - x_test) < 1e-7 && fabs(z - z_test) < 1e-7) {
				double angle1 = atan2(px[1], pz[1]) + atan2(x - px[1], pz[1] - z);
				if (angle1 < 0)
					angle1 += 2*M_PI;
				sols[count][0] = th2[1];
				sols[count][1] = th3[1];
				sols[count][2] = angle1;
				count++;
			}
		}

		if (count >= 1) {
			if (count == 1 || (elbow_down == 0 && sols[1][2] > sols[0][2])
					|| (elbow_down == 1 && sols[1][2] < sols[0][2])) {
				th23[0] = sols[0][0];
				th23[1] = sols[0][1];
			} else {
				th23[0] = sols[1][0];
				th23[1] = sols[1][1];
			}
		} else {
			unreach = 1;
		}
	}
	if (unreach) {
		th23[0] = 0;
		th23[1] = 0;
	}
	return (unreach) ? KIN_UNREACHABLE : KIN_OK;
}

/*!
 * Compute the logarithm of a t44
 *
 * Reference: cb.tfm.log_trans
 *
 * @param[in] t44 t44 to take logarithm of
 * @param[out] out logarithm
 */
void log_trans(const double t44[4][4], double out[4][4])
{
	double trace_r = t44[0][0] + t44[1][1] + t44[2][2];
	double temp = 0.5*(trace_r - 1);
	double theta = 0.0;
	double omega_hat_fact = 0.5;
	double omega_hat[3][3];
	double w_norm;
	double factor = 0;
	double A_inv[3][3];
	double temp3[3];
	double p[3];
	int k1;
	int k2;

	memset(out, 0, 16*sizeof(double));

	if (fabs(temp) <= 1.0) {
		theta = acos(temp);
	} else if(temp < -1.0) {
		theta = M_PI;
	}

	if (theta > 1e-8)
		omega_hat_fact = theta/(2*sin(theta));

	/* omega_hat = omega_hat_fact*(r-r.T) */
	for (k1 = 0; k1 < 3; k1++) {
		for (k2 = 0; k2 < 3; k2++) {
			omega_hat[k1][k2] = omega_hat_fact*(t44[k1][k2] - t44[k2][k1]);
		}
	}

	/* w_norm = sp.Matrix.norm(cb.operator.vee_skew(omega_hat)) */
	w_norm = sqrt(pow(omega_hat[0][1], 2) + pow(omega_hat[0][2], 2) + pow(omega_hat[1][2], 2));

	if (w_norm >= 1e-6) {
		double sw = sin(w_norm);
		double cw = 1 + cos(w_norm);
		factor = (2*sw - w_norm*cw)/(2 * pow(w_norm, 2) * sw);
	}

	/* A_inv = sp.eye(3) - 0.5*omega_hat + factor*omega_hat*omega_hat */
	mul33(omega_hat, omega_hat, A_inv);
	for (k1 = 0; k1 < 3; k1++) {
		for (k2 = 0; k2 < 3; k2++) {
			A_inv[k1][k2] *= factor;
			A_inv[k1][k2] -= 0.5*omega_hat[k1][k2];
		}
		A_inv[k1][k1] += 1;
	}

	/* out = [ omega_hat , A_inv*p ; 0 , 0] */
	p[0] = t44[0][3];
	p[1] = t44[1][3];
	p[2] = t44[2][3];
	mul33_3(A_inv, p, temp3);
	for (k1 = 0; k1 < 3; k1++) {
		out[k1][3] = temp3[k1];
		for (k2 = 0; k2 < 3; k2++) {
			out[k1][k2] = omega_hat[k1][k2];
		}
	}
}

/*!
 * Compute the difference between two t44s
 *
 * Reference: cb.tfm.pos_diff
 *
 * @param[in] t44_curr Current t44
 * @param[in] t44_target Target t44
 * @param[out] out difference vector
 */
void vee_log_diff(const double t44_curr[4][4], const double t44_target[4][4],
		double out[6])
{
	double diff[4][4];
	double temp[4][4];
	double log_diff[4][4];

	memset(out, 0, 6*sizeof(double));
	inv_t44(t44_curr, temp);
	mul44(t44_target, temp, diff);
	log_trans(diff, log_diff);
	vee(log_diff, out);
}

/*!
 * Compute the position difference between two t44s
 *
 * @param[in] t44_curr Current t44
 * @param[in] t44_target Target t44
 */
double pos_diff(const double t44_curr[4][4], const double t44_target[4][4])
{
	double tmp = 0.0;
	int i;

	for (i = 0; i < 3; i++)
		tmp += pow(t44_curr[i][3] - t44_target[i][3], 2);
	return sqrt(tmp);
}

/*!
 * Compute the orientation difference between two t44s as a angle
 *
 * @param[in] t44_curr Current t44
 * @param[in] t44_target Target t44
 */
double rot_diff(const double t44_curr[4][4], const double t44_target[4][4])
{
	double r44[4][4];
	double diff[4][4];
	double temp[4][4];
	double tmp = 0.0;
	double rot_frob_norm;
	int i;
	int j;

	inv_t44(t44_target, temp);
	mul44(t44_curr, temp, diff);
	log_trans(diff, r44);
	for (i = 0; i<3; i++)
		for (j = 0; j<3; j++)
			tmp += pow(r44[i][j], 2);
	rot_frob_norm = sqrt(tmp);
	return rot_frob_norm/sqrt(2.0);
}

/*!
 * Compute the logarithm of rotation matrix
 *
 * Reference: cb.tfm.np_logR + cb.tfm.np_logR_special
 *
 * @param[in] t44_in T44 with rotation matrix to logarithmize
 * @param[out] log_R Logarithm of input R
 */
void logR(const double t44_in[4][4], double log_R[3][3])
{
	double rtrace, rt, theta;

	rtrace = t44_in[0][0] + t44_in[1][1] + t44_in[2][2];
	rt = rtrace - 1;
	if (rt >= 2) {
		theta = 0.0;
	} else if (rt <= -2) {
		theta = M_PI;
	} else {
		theta = acos(rt/2);
	}

	if (fabs(theta - M_PI) > 1e-5) {
		double omega_hat_fact = 0.5;
		int i, j;

		if (theta > 1e-8) {
			omega_hat_fact = theta/(2*sin(theta));
		}
		for (i = 0; i < 3; i++) {
			for (j = 0; j < 3; j++) {
				log_R[i][j] = (t44_in[i][j] - t44_in[j][i])*omega_hat_fact;
			}
		}
	} else {
		double qs = sqrt(fabs(rtrace + 1))/2;
		double kx = t44_in[2][1] - t44_in[1][2];
		double ky = t44_in[0][2] - t44_in[2][0];
		double kz = t44_in[1][0] - t44_in[0][1];
		double kx1, ky1, kz1;
		double add_fac = -1;
		double divider;

		if (t44_in[0][0] >= t44_in[1][1] && t44_in[0][0] >= t44_in[2][2]) {
			kx1 = t44_in[0][0] - t44_in[1][1] - t44_in[2][2] + 1;
			ky1 = t44_in[1][0] + t44_in[0][1];
			kz1 = t44_in[2][0] + t44_in[0][2];
			if (kx >= 0) {
				add_fac = 1;
			}
		} else if (t44_in[1][1] >= t44_in[2][2]) {
			kx1 = t44_in[1][0] + t44_in[0][1];
			ky1 = t44_in[1][1] - t44_in[0][0] - t44_in[2][2] + 1;
			kz1 = t44_in[2][1] + t44_in[1][2];
			if (ky >= 0) {
				add_fac = 1;
			}
		} else {
			kx1 = t44_in[2][0] + t44_in[0][2];
			ky1 = t44_in[2][1] + t44_in[1][2];
			kz1 = t44_in[2][2] - t44_in[0][0] - t44_in[1][1] + 1;
			if (kz >= 0) {
				add_fac = 1;
			}
		}
		kx += add_fac*kx1;
		ky += add_fac*ky1;
		kz += add_fac*kz1;
		divider = sqrt(kx*kx + ky*ky + kz*kz);
		if (divider < 1e-6) {
			divider = 1;
		}
		if (qs >= 1) {
			theta = 0;
		} else {
			theta = 2*acos(qs);
		}
		theta /= divider;
		hat(kx*theta, ky*theta, kz*theta, log_R);
	}
}

/*!
 * Transform from t44 to basepars
 *
 * Reference: cb.tfm.t44_to_basepar
 *
 * @param[in] t44 t44 matrix.
 * @param[out] out basepar vector.
 */
void tf2basepar(const double t44[4][4], double out[6]){
        double log_r[3][3];

        logR(t44, log_r);
        out[0] = t44[0][3];
        out[1] = t44[1][3];
        out[2] = t44[2][3];
        out[3] = log_r[2][1];
        out[4] = log_r[0][2];
        out[5] = log_r[1][0];
}

/*!
 * Transform from basepars to t44
 *
 * Reference: cb.tfm.basepar_to_t44
 *
 * @param[in] out basepar vector.
 * @param[out] t44 t44 matrix.
 */
void basepar2tf(const double bp[6], double out[4][4]){
        double rmat[3][3];
        int i1, i2;

        eye(4, (double *) out);
        twist_R(bp, 1.0, rmat);
        for( i1 = 0; i1 < 3; i1++ ){
                out[i1][3] = bp[i1];
                for( i2 = 0; i2 < 3; i2++ ){
                        out[i1][i2] = rmat[i1][i2];
                }
        }
}

/*!
 * Compute the difference of two transformation matrices
 *
 * Reference: cb.tfm.tr2diff
 *
 * @param[in] t44_1 First trannsformation matrix
 * @param[in] t44_2 Second trannsformation matrix
 * @param[out] diffvec Difference in the form of a six element vector
 */
void tr2diff(const double t44_1[4][4], const double t44_2[4][4], double diffvec[6])
{
	double temp1[4][4];
	double temp2[4][4];
	double log_r[3][3];
	double rot_vec[3];
	double tmp[3];
	int i1, i2;

	inv_t44(t44_1, temp1);
	mul44(temp1, t44_2, temp2);
	logR(temp2, log_r);
	rot_vec[0] = log_r[2][1];
	rot_vec[1] = log_r[0][2];
	rot_vec[2] = log_r[1][0];

	for (i1 = 0; i1 < 3; i1++) {
		tmp[i1] = 0;
		for (i2 = 0; i2 < 3; i2++) {
			tmp[i1] += t44_1[i1][i2]*rot_vec[i2];
		}
	}

	diffvec[0] = t44_2[0][3] - t44_1[0][3];
	diffvec[1] = t44_2[1][3] - t44_1[1][3];
	diffvec[2] = t44_2[2][3] - t44_1[2][3];
	diffvec[3] = tmp[0];
	diffvec[4] = tmp[1];
	diffvec[5] = tmp[2];
}

/*!
 * Compute a normed difference of two transformation matrices
 *
 * @param[in] t44_1 First trannsformation matrix
 * @param[in] t44_2 Second trannsformation matrix
 * @param[out] normvec Array with position norm diff and orientation norm diff
 */
void normdiff(const double t44_1[4][4], const double t44_2[4][4], double normvec[2])
{
	double diffvec[6];

	tr2diff(t44_1, t44_2, diffvec);
	normvec[0] = sqrt(diffvec[0]*diffvec[0] + diffvec[1]*diffvec[1]
			+ diffvec[2]*diffvec[2]);
	normvec[1] = sqrt(diffvec[3]*diffvec[3] + diffvec[4]*diffvec[4]
			+ diffvec[5]*diffvec[5]);
}
