#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(EXTRA_CALC) && !defined(CB_SYSTEM_HEADER)
#include <stdint.h>
#endif

#ifdef CB_SYSTEM_HEADER
#include CB_SYSTEM_HEADER
#else
#include <math.h>
#endif

#ifndef _POSIX_C_SOURCE
#define M_PI 3.14159265358979323846   /* pi */
#define M_PI_2 1.57079632679489661923 /* pi/2 */
#define M_PI_4 0.78539816339744830962 /* pi/4 */
#endif

#include "c_lpoe/lpoe.h"
#include "c_lpoe/matrix_math.h"
#include "ckin/lin_alg.h"
#include "ckin/common.h"
#include "kinematics.h"
#include "ax4_kinematics.h"

#define D2R(deg) (deg * M_PI / 180)

static void copy_44_33(double a[4][4], double b[3][3])
{
	for (size_t y = 0; y < 3; y++) {
		for (size_t x = 0; x < 3; x++) {
			b[y][x] = a[y][x];
		}
	}
}

static void transpose(double *m, double *out, size_t s)
{
	for (size_t m_y = 0; m_y < s; m_y++) {
		for (size_t m_x = 0; m_x < s; m_x++) {
			// TRACE("out[%zu][%zu] = m[%zu][%zu]\n", m_y, m_x, m_x, m_y);
			*(out + (m_y * s) + m_x) = *(m + (m_x * s) + m_y);
		}
	}
}

#ifdef EXTRA_CALC
static double norm(const double *a, size_t s)
{
	double res = 0;
	for (size_t i = 0; i < s; i++) {
		// TRACE("[%zu]: %.16f * %.16f\n", i, a[i]);
		res += a[i] * a[i];
	}
	return sqrt(res);
}

/**
 * An attempt at an implementation of numpy.sign
 *
 * The sign function returns -1 if x < 0, 0 if x==0, 1 if x > 0.
 * nan is returned for nan inputs.
 */
static double sign(double x)
{
    if (x < 0)
    {
        return -1;
    }
    else if (x == 0)
    {
        return 0;
    }
    else if (x > 0)
    {
        return 1;
    }

    union { double d; uint64_t i; } u;
    /* ....AAAABBBBCCCCDDDD */
    u.i = 0x7FF8000000000000; /* IEEE 754 quiet NaN bitpattern for 64-bit double type */
    return u.d;
}

#endif

/**
 * Return inverse of homogeneous transformation
 */
static void trans_inv(double m[4][4], double ret[4][4])
{
	eye(4, (double *)ret);

	// r = m[:3, :3].T
	double m33[3][3];
	copy_44_33(m, m33);
	double r[3][3];
	transpose((double *)m33, (double *)r, 3);

	// ret[:3, :3] = r

	for (size_t y = 0; y < 3; y++) {
		for (size_t x = 0; x < 3; x++) {
			ret[y][x] = r[y][x];
			r[y][x] = r[y][x] * -1.0; // prepare for next operation
		}
	}

	// ret[:3, 3] = -r.dot(m[:3, 3])
	double m3[3] = {m[0][3], m[1][3], m[2][3]};
	double ret3[3];
	mul33_3(r, m3, ret3);

	ret[0][3] = ret3[0];
	ret[1][3] = ret3[1];
	ret[2][3] = ret3[2];
}

/**
 * Inverse kinematics from cartesian space to joint space
 *
 * @param pos Position array of the TCP
 * @param orient Orientation of the TCP
 * @param q Joint angles
 * @return Negative on error
 */
int inv(const struct agile_pkm_model *rob, const double pos[3], const double orient, double q[4])
{
	double pos_[3];
	double tmp1, tmp2;
	double L2proj;
	double delta, gamma;
	double q2, q4;
	double q1s[2];
	double q2s[2];
	int valid_q2[2];
	double inv_elbow[4][4];
	double tf_tcp[4][4];
	double tcp_in_elbow[4][4];
	int ret = -1;

	pos_[0] = pos[0];
	pos_[1] = pos[1];
	pos_[2] = pos[2] + rob->a2 - rob->L0;

	tmp1 = pos_[2] / (rob->L2 - rob->d2);
	if (fabs(tmp1) > 1 + 1e-10) {
		return -1;
	} else if (tmp1 >= 1) {
		q[2] = M_PI_2;
	} else if (tmp1 <= -1) {
		q[2] = -M_PI_2;
	} else {
		q[2] = asin(tmp1);
	}

	L2proj = cos(q[2]) * (rob->L2 - rob->d2) + rob->d2;

	// circle-circle intersection to find q1
	tmp1 = rob->ax3y * pos[0] - rob->L1 * pos[1];
	tmp2 = rob->L1 * pos[0] + rob->ax3y * pos[1];
	delta = atan2(tmp2, tmp1);

	tmp1 = (pow(L2proj, 2) - pow(rob->L1, 2) - pow(rob->ax3y, 2) - pow(pos[0], 2) - pow(pos[1], 2)) / (2.0 * sqrt(pow(tmp1, 2) + pow(tmp2, 2)));
	if (fabs(tmp1) > 1 + 1e-10) {
		return -1;
	} else if (tmp1 >= 1) {
		gamma = M_PI_2;
	} else if (tmp1 <= -1) {
		gamma = -M_PI_2;
	} else {
		gamma = asin(tmp1);
	}

	q1s[0] = fmod((delta + gamma + M_PI), (2 * M_PI)) - M_PI;
	q1s[1] = fmod((delta + 2 * M_PI - gamma), (2 * M_PI)) - M_PI;

	for (int k = 0; k < 2; k++) {
		// Try to move j1 to within bounds
		if (q1s[k] < rob->joint_lims[0][0]) {
			q1s[k] += 2 * M_PI;
		} else if (q1s[k] > rob->joint_lims[0][1]) {
			q1s[k] -= 2 * M_PI;
		}
	}

	// Find q2
	for (size_t i = 0; i < 2; i++) {
		rot_z(-q1s[i], inv_elbow);
		inv_elbow[0][3] = -rob->L1;
		inv_elbow[1][3] = -rob->ax3y;

		rot_z(orient, tf_tcp);
		tf_tcp[0][3] = pos_[0];
		tf_tcp[1][3] = pos_[1];
		tf_tcp[2][3] = pos_[2];

		mul44(inv_elbow, tf_tcp, tcp_in_elbow);

		q2 = -atan2(tcp_in_elbow[0][3], tcp_in_elbow[1][3]);

		// Discard solution if outside joint limit interval
		if (q2 < rob->joint_lims[1][0] || q2 > rob->joint_lims[1][1]) {
			valid_q2[i] = 0;
		} else {
			q2s[i] = q2;
			valid_q2[i] = 1;
		}
	}

	for (size_t i = 0; i < 2; i++) {
		// If q1 outside bounds, discard solution
		if (q1s[i] < rob->joint_lims[0][0] || q1s[i] > rob->joint_lims[0][1]) {
			continue;
		}
		if (!valid_q2[i]) {
			continue;
		}

		q4 = orient - q1s[i] - q2s[i];

		if (q4 < rob->joint_lims[3][0]) {
			q4 += 2.0 * M_PI;
		} else if (q4 > rob->joint_lims[3][1]) {
			q4 -= 2.0 * M_PI;
		}
		if (q4 > rob->joint_lims[3][1] || q4 < rob->joint_lims[3][0]) {
			continue;
		}

		q[0] = q1s[i];
		q[1] = q2s[i];
		q[3] = q4;

		ret = 0;
	}

	return ret;
}

/**
 * Inverse kinematics for joints 1, 2 and 3, taking a TCP position and a wrist joint angle.
 *
 * @param pos Position array of the TCP
 * @param q4 Wrist joint angle
 * @param tool x, y, z dimensions of the tool
 * @param q Joint angles excluding wrist joint
 * @return Negative on error
 */
int inv_posonly(const struct agile_pkm_model *rob, const double pos[3], const double q4, const double tool[3], double q[4])
{
	double pos_[3];
	double tool_[3];
	double tmp1, tmp2;
	double L2proj, tcpproj;
	double delta, gamma;
	double q2;
	double q1s[2];
	double q2s[2];
	int valid_q2[2];
	double inv_elbow[4][4];
	double tf_tcp[4][4];
	double tcp_in_elbow[4][4];
	int ret = -1;

	pos_[0] = pos[0];
	pos_[1] = pos[1];
	pos_[2] = pos[2] + rob->a2 - rob->L0 - tool[2];

	tool_[0] = cos(q4) * tool[0] - sin(q4) * tool[1];
	tool_[1] = sin(q4) * tool[0] + cos(q4) * tool[1];
	tool_[2] = tool[2];

	tmp1 = pos_[2] / (rob->L2 - rob->d2);

	if (fabs(tmp1) > 1 + 1e-10) {
		return -1;
	} else if (tmp1 >= 1) {
		q[2] = M_PI_2;
	} else if (tmp1 <= -1) {
		q[2] = -M_PI_2;
	} else {
		q[2] = asin(tmp1);
	}

	L2proj = cos(q[2]) * (rob->L2 - rob->d2) + rob->d2;
	tcpproj = sqrt(pow(tool_[0], 2) + pow(L2proj + tool_[1], 2));

	// circle-circle intersection to find q1
	tmp1 = rob->ax3y * pos[0] - rob->L1 * pos[1];
	tmp2 = rob->L1 * pos[0] + rob->ax3y * pos[1];
	delta = atan2(tmp2, tmp1);

	tmp1 = (pow(tcpproj, 2) - pow(rob->L1, 2) - pow(rob->ax3y, 2) - pow(pos[0], 2) - pow(pos[1], 2)) / (2.0 * sqrt(pow(tmp1, 2) + pow(tmp2, 2)));
	if (fabs(tmp1) > 1 + 1e-10) {
		return -1;
	} else if (tmp1 >= 1) {
		gamma = M_PI_2;
	} else if (tmp1 <= -1) {
		gamma = -M_PI_2;
	} else {
		gamma = asin(tmp1);
	}

	q1s[0] = fmod((delta + gamma + M_PI), (2 * M_PI)) - M_PI;
	q1s[1] = fmod((delta + 2 * M_PI - gamma), (2 * M_PI)) - M_PI;

	for (size_t i = 0; i < 2; i++) {
		rot_z(-q1s[i], inv_elbow);
		inv_elbow[0][3] = -rob->L1;
		inv_elbow[1][3] = -rob->ax3y;

		eye(4, (double *)tf_tcp);
		tf_tcp[0][3] = pos_[0];
		tf_tcp[1][3] = pos_[1];
		tf_tcp[2][3] = pos_[2];

		mul44(inv_elbow, tf_tcp, tcp_in_elbow);

		tmp1 = atan2(tcp_in_elbow[0][3], tcp_in_elbow[1][3]);
		tmp2 = atan2(tool_[0], L2proj + tool_[1]);

		q2 = tmp2 - tmp1;

		// Discard solution if outside joint limit interval
		if (q2 < rob->joint_lims[1][0] || q2 > rob->joint_lims[1][1]) {
			valid_q2[i] = 0;
		} else {
			q2s[i] = q2;
			valid_q2[i] = 1;
		}
	}

	for (size_t i = 0; i < 2; i++) {
		if (valid_q2[i] == 0) {
			continue;
		}

		q[0] = q1s[i];
		q[1] = q2s[i];

		ret = 0;
	}

	q[3] = q4;

	return ret;
}

/**
 * Forward kinematics from joint space to cartesian space
 *
 * @param q Joint angles
 * @param tcp_out T44 matrix of TCP
 * @param orient_angle Orientation angle of TCP
 * @return Negative on error
 */
int fwd(const struct agile_pkm_model *rob, const double q[4], double tcp_out[4][4], double *orient_angle)
{
	double tf1[4][4];
	double tf3[4][4];
	double tf5[4][4];
	double tfW[4][4];
	double tmp44[4][4];
	double tf_rot[4][4];

	eye(4, (double *)tf1);
	tf1[2][3] = rob->L0;

	rot_z(q[0], tf_rot);
	mul44(tf1, tf_rot, tcp_out);

	eye(4, (double *)tf3);
	tf3[0][3] = rob->L1;
	tf3[1][3] = rob->ax3y;
	mul44(tcp_out, tf3, tmp44);

	rot_z(q[1], tf_rot);
	mul44(tmp44, tf_rot, tcp_out);

	rot_x(q[2], tf_rot);
	mul44(tcp_out, tf_rot, tmp44);

	eye(4, (double *)tf5);
	tf5[1][3] = rob->L2 - rob->d2;
	mul44(tmp44, tf5, tcp_out);

	rot_x(-q[2], tf_rot);
	mul44(tcp_out, tf_rot, tmp44);

	eye(4, (double *)tfW);
	tfW[1][3] = rob->d2;
	tfW[2][3] = -rob->a2;
	mul44(tmp44, tfW, tcp_out);

	rot_z(q[3], tf_rot);
	mul44(tcp_out, tf_rot, tmp44);

	memcpy(tcp_out, tmp44, 4 * 4 * sizeof(double));

	*orient_angle = q[0] + q[1] + q[3];
	return 0;
}

int fwd_elbow(const struct agile_pkm_model *rob, const double q1, double elbow_out[4][4])
{
	double s = sin(q1);
	double c = cos(q1);
	eye(4, (double *)elbow_out);

	elbow_out[0][0] = c;
	elbow_out[0][1] = -s;
	elbow_out[1][0] = s;
	elbow_out[1][1] = c;

	elbow_out[0][3] = rob->L1 * c - rob->ax3y * s;
	elbow_out[1][3] = rob->L1 * s + rob->ax3y * c;
	elbow_out[2][3] = rob->L0;
	return 0;
}

/**
 * Transformation from joint space to drive space
 *
 * @param joints Joint angles
 * @param drives Drive angles
 */
int joint_to_drive(const struct agile_pkm_model *rob, const double joints[4], double drives[4])
{
	double q4_angle;
	double up_arm_vec[3];
	double p_[3];
	double q2_list[2];
	double tmp, tmp1, tmp2, gamma, angle;
	double tf_rc_mm[4][4];

	drives[0] = joints[0];
	drives[2] = atan2(sin(joints[2]), cos(joints[1]) * cos(joints[2]));
	q4_angle = joints[3];

	up_arm_vec[0] = -sin(joints[1]) * cos(joints[2]);
	up_arm_vec[1] = cos(joints[1]) * cos(joints[2]);
	up_arm_vec[2] = sin(joints[2]);
	p_[0] = up_arm_vec[0] * rob->L3 + rob->L1;
	p_[1] = up_arm_vec[1] * rob->L3 + rob->ax3y;
	p_[2] = up_arm_vec[2] * rob->L3;

	tmp2 = pow(rob->ax2x - p_[0], 2) + pow(rob->ax2y - p_[1], 2);
	tmp1 = tmp2 + pow(rob->ax2z - p_[2], 2);

	gamma = atan2(rob->ax2x - p_[0], rob->ax2y - p_[1]);
	tmp = (pow(rob->Lpar, 2) - tmp1 - pow(rob->L3b, 2)) /
		(2.0 * rob->L3b * sqrt(tmp2));
	if (fabs(tmp) > 1.0 + 1e-10) {
		return -1;
	} else if (fabs(tmp) >= 1) {
		if (tmp > 0)
			angle = M_PI_2;
		else
			angle = -M_PI_2;
	} else {
		angle = asin(tmp);
	}

	q2_list[0] = fmod(-gamma + angle + M_PI, 2 * M_PI) - 3.0 / 2.0 * M_PI;
	q2_list[1] = fmod(-gamma + 2 * M_PI - angle, 2 * M_PI) - 3.0 / 2.0 * M_PI;

	if (rob->bh) {
		tmp = rob->L2 - rob->d2;
		rot_z(joints[1], tf_rc_mm);
		tf_rc_mm[0][3] = (up_arm_vec[0] * tmp - sin(joints[1]) * rob->d2) * 1000;
		tf_rc_mm[1][3] = (up_arm_vec[1] * tmp + cos(joints[1]) * rob->d2) * 1000;
		tf_rc_mm[2][3] = up_arm_vec[2] * tmp * 1000;
		if (ax4_inv(rob->bh, q4_angle, tf_rc_mm, &drives[3]) < 0)
			return -1;
	} else {
		drives[3] = q4_angle;
	}

	/* Discard solution if outside limits */
	if (q2_list[0] >= -M_PI_2 && q2_list[0] <= M_PI_2) {
		drives[1] = q2_list[0] - rob->q2_offs;
		return 0;
	} else if (q2_list[1] >= -M_PI_2 && q2_list[1] <= M_PI_2) {
		drives[1] = q2_list[1] - rob->q2_offs;
		return 0;
	}
	return -1;
}
/**
 * Transformation from drive space to joint space
 *
 * @param drives Drive angles
 * @param joints Joint angles
 * @param res_q_extra Extra angles, can be NULL
 * @return Negative on error
 */
#ifdef EXTRA_CALC
int drive_to_joint_full(const struct agile_pkm_model *rob, const double drives[4], double joints[4], res_q_extra_t *res_q_extra, ax4_fwd_ret_t *q4_angle)
#else
int drive_to_joint_full(const struct agile_pkm_model *rob, const double drives[4], double joints[4], res_q_extra_t *res_q_extra)
#endif
{
	double tf2_0[4][4];
	double tf2_q[4][4];
	double tf2_1[4][4];
	double tf2_2[4][4];
	double tf2[4][4];
	double tf3_0[4][4];
	double tf3_q[4][4];
	double tf3[4][4];
	double tf3_tmp[4][4];
	double tf2_in_3[4][4];
	double tmp, tmp1, tmp2, angle, delta;
	double q23_list[2];
	int i;

	/* Serial forward kinematics */
	eye(4, (double *)tf2_0);
	tf2_0[0][3] = rob->ax2x;
	tf2_0[1][3] = rob->ax2y;
	tf2_0[2][3] = rob->ax2z;
	rot_z(drives[1] + M_PI_2 + rob->q2_offs, tf2_q);
	eye(4, (double *)tf2_1);
	tf2_1[0][3] = rob->L3b;
	mul44(tf2_q, tf2_1, tf2_2);
	mul44(tf2_0, tf2_2, tf2);

	eye(4, (double *)tf3_0);
	tf3_0[0][3] = rob->L1;
	tf3_0[1][3] = rob->ax3y;
	tf3_0[2][3] = 0.0;
	rot_x(drives[2], tf3_q);
	mul44(tf3_0, tf3_q, tf3);

	/* Express tf2 in the coordinate frame of tf3 */
	trans_inv(tf3, tf3_tmp);
	mul44(tf3_tmp, tf2, tf2_in_3);

	/* Find p4 as intersection between circle from p2 and circle from p3
	    Radius of circle from p2 projected into xy-plane defined by tf3 */
	tmp2 = pow(tf2_in_3[0][3], 2) + pow(tf2_in_3[1][3], 2);
	tmp1 = tmp2 + pow(tf2_in_3[2][3], 2);
	tmp = (pow(rob->Lpar, 2) - tmp1 - pow(rob->L3, 2)) /
		(2.0 * rob->L3 * sqrt(tmp2));

	if (fabs(tmp) > 1.0 + 1e-10) {
		return -1;
	} else if (fabs(tmp) >= 1) {
		if (tmp > 0)
			angle = M_PI_2;
		else
			angle = -M_PI_2;
	} else {
		angle = asin(tmp);
	}

	delta = atan2(tf2_in_3[1][3], tf2_in_3[0][3]);

	q23_list[0] = fmod(delta + angle + M_PI, 2 * M_PI) - M_PI;
	q23_list[1] = fmod(delta + 2 * M_PI - angle, 2 * M_PI) - M_PI;

	for (i = 0; i < 2; i++) {
		/* Discard solution if it is behind p3 */
		if (fabs(q23_list[i]) > M_PI_2)
			continue;

		joints[0] = drives[0];
		joints[1] = atan2(sin(q23_list[i]), cos(drives[2]) * cos(q23_list[i]));
		joints[2] = asin(sin(drives[2]) * cos(q23_list[i]));

		if (rob->bh) {
			double tf_rc[4][4];
			double delta_p1[3];
			double delta_p2[3];

			rot_z(drives[1], tf_rc);
			tmp = rob->L2 - rob->d2;
			delta_p1[0] = -tmp * sin(drives[1]) * cos(drives[2]);
			delta_p1[1] = tmp * cos(drives[1]) * cos(drives[2]);
			delta_p1[2] = tmp * sin(drives[2]);
			delta_p2[0] = -sin(drives[1]) * rob->d2;
			delta_p2[1] = cos(drives[1]) * rob->d2;
			delta_p2[2] = 0;
			tf_rc[0][3] = (delta_p1[0] + delta_p2[0]) * 1000;
			tf_rc[1][3] = (delta_p1[1] + delta_p2[1]) * 1000;
			tf_rc[2][3] = (delta_p1[2] + delta_p2[2]) * 1000;

#ifdef EXTRA_CALC
			if (ax4_fwd(rob->bh, drives[3], tf_rc, q4_angle) < 0)
				continue;
			joints[3] = q4_angle->q4;
#else
			ax4_fwd_ret_t q4_angle;
			if (ax4_fwd(rob->bh, drives[3], tf_rc, &q4_angle) < 0)
				continue;
			joints[3] = q4_angle.q4;
#endif
		} else {
			joints[3] = drives[3];
#ifdef EXTRA_CALC
			q4_angle->q4 = drives[3];
#endif
		}

		if (res_q_extra) {
			double v1[2], v2[2];
			double alpha1, alpha2, qx, qy;
			double tf4_[4][4];
			double tmp44[4][4];
			double q23 = asin(sin(joints[1]) * cos(joints[2]));

			rot_z(q23, tf3_tmp);

			mul44(tf3, tf3_tmp, tmp44);

			eye(4, (double *)tf3_tmp);
			tf3_tmp[1][3] = rob->L3;

			mul44(tmp44, tf3_tmp, tf4_);
			v1[0] = tf2[0][0];
			v1[1] = tf2[1][0];
			v2[0] = tf4_[0][3] - tf2[0][3];
			v2[1] = tf4_[1][3] - tf2[1][3];
			tmp1 = v1[0] * v2[0] + v1[1] * v2[1];
			tmp2 = v1[0] * v2[1] - v1[1] * v2[0];
			alpha1 = atan2(tmp2, tmp1);

			tmp = (tf4_[2][3] - tf2[2][3]) / rob->Lpar;
			if (fabs(tmp) > 1.0 + 1e-10) {
				return -1;
			} else if (fabs(tmp) >= 1) {
				if (tmp > 0)
					alpha2 = -M_PI_2;
				else
					alpha2 = M_PI_2;
			} else {
				alpha2 = -asin(tmp);
			}

			qx = asin(-tf4_[2][1]);
			qy = atan2(tf4_[2][0], tf4_[2][2]);

			res_q_extra->q23 = q23;
			res_q_extra->qx = qx;
			res_q_extra->qy = qy;
			res_q_extra->alpha1 = alpha1;
			res_q_extra->alpha2 = alpha2;

#ifdef EXTRA_CALC
			double beta1, beta2;
			double rod_twist, rod_rot, rod_tilt, rod_rot_z;
			double tf4_in_2_[4][4];
			double tf4_in_2_2_[4][4];
			double tf2_in_4_[4][4];

			trans_inv(tf4_, tmp44);
			mul44(tmp44, tf2, tf2_in_4_);

			v1[0] = tf2_in_4_[0][3];
			v1[1] = tf2_in_4_[2][3];
			tmp1 = sqrt(v1[0] * v1[0] + v1[1] * v1[1]);
			tmp2 = tf2_in_4_[1][3];
			beta1 = -(M_PI + atan2(v1[1], v1[0]));
			beta2 = atan2(tmp1, tmp2);

			if (drives[2] > 0) {
				v1[0] = tf4_[0][1];
				v1[1] = tf4_[1][1];
				rod_rot_z = (atan2(norm(&(tf4_[2][1]), 1), norm(v1, 2)));
			} else {
				v1[0] = tf4_[0][1];
				v1[1] = tf4_[1][1];
				rod_rot_z = (-atan2(norm(&(tf4_[2][1]), 1), norm(v1, 2)));
			}

			trans_inv(tf2, tmp44);
			mul44(tmp44, tf4_, tf4_in_2_);

			v1[0] = tf4_in_2_[0][3];
			v1[1] = tf4_in_2_[1][3];
			rod_tilt = -atan2(tf4_in_2_[2][3], norm(v1, 2));

			rod_twist = -atan2(tf2_in_4_[2][3], tf2_in_4_[0][3]);

			if (fabs(rod_twist) > M_PI_2) {
				rod_twist += -sign(rod_twist) * M_PI;
			}

			trans_inv(tf2_2, tmp44);
			mul44(tmp44, tf4_, tf4_in_2_2_);

			v1[0] = tf4_in_2_2_[0][3];
			v1[1] = tf4_in_2_2_[1][3];
			tmp = norm(v1, 2);
			rod_rot = acos((pow(rob->Lpar * cos(rod_tilt), 2) + pow(rob->L3b, 2) - pow(tmp, 2)) / (2 * rob->Lpar * cos(rod_tilt) * rob->L3b)) - M_PI_2;

			res_q_extra->beta1 = beta1;
			res_q_extra->beta2 = beta2;
			res_q_extra->rod_twist = rod_twist;
			res_q_extra->rod_rot = rod_rot;
			res_q_extra->rod_tilt = rod_tilt;
			res_q_extra->rod_rot_z = rod_rot_z;
			res_q_extra->q4_angle = joints[3];
			if (i == 0)
				res_q_extra->sign = 1;
			else
				res_q_extra->sign = -1;
#endif
		}

		return 0;
	}
	return -1;
}

int drive_to_joint(const struct agile_pkm_model *rob, const double drives[4], double joints[4])
{
	int ret;
#ifdef EXTRA_CALC
	ax4_fwd_ret_t q4_angle;
	ret = drive_to_joint_full(rob, drives, joints, NULL, &q4_angle);
#else
	ret = drive_to_joint_full(rob, drives, joints, NULL);
#endif
	return ret;
}

/**
 * Inverse kinematics from cartesian space to drive space
 *
 * @param pos Position array of the TCP
 * @param orient_angle Orientation angle of the TCP
 * @param q Drive angles
 * @return Negative on error
 */
int cart_to_drive(const struct agile_pkm_model *rob, const double pos[3], const double orient_angle, double q[4])
{
	double joints[4];
	int ret;

	ret = inv(rob, pos, orient_angle, joints);
	if (ret)
		return ret;

	ret = joint_to_drive(rob, joints, q);
	return ret;
}

/**
 * Inverse kinematics from cartesian space to drive space
 *
 * @param pos Position array of the TCP
 * @param orient_angle Orientation angle of the TCP
 * @param q Drive angles
 * @return Negative on error
 */
int cart_to_drive_posonly(const struct agile_pkm_model *rob, const double pos[3], const double q4, const double tool[3], double q[4])
{
	double joints[4];
	int ret;

	ret = inv_posonly(rob, pos, q4, tool, joints);
	if (ret)
		return ret;

	ret = joint_to_drive(rob, joints, q);
	return ret;
}

/**
 * Forward kinematics from drive space to cartesian space
 *
 * @param q Drive angles
 * @param tcp_out T44 matrix of TCP
 * @param orient_angle Orientation angle of TCP
 * @return Negative on error
 */
int drive_to_cart(const struct agile_pkm_model *rob, const double q[4], double tcp_out[4][4], double *orient_angle)
{
	double joints[4];
	int ret;

	ret = drive_to_joint(rob, q, joints);
	if (ret)
		return ret;

	ret = fwd(rob, joints, tcp_out, orient_angle);
	return ret;
}

/**
 * Does not support backhoe.
*/
static int calculate_derivatives_drive_to_joint(const struct agile_pkm_model *rob, const double q[4], double th[5], double dth_dq[5][4], double d2th_dq2[4][5][4])
{
	double c2, s2, c3, s3;
	double s23, c23, sx, cx;
	double qx, th2;
	double p2x, p2y, p2z;
	double f1, f2, f3;
	double delta, angle;
	double q23;
	double tmp1, tmp2;
	double sign;
	double g1, g2, g3, g4;

	c2 = cos(q[1] + M_PI_2 + rob->q2_offs);
	s2 = sin(q[1] + M_PI_2 + rob->q2_offs);
	c3 = cos(q[2]);
	s3 = sin(q[2]);

	p2x = c2 * rob->L3b + rob->ax2x - rob->L1;
	p2y = s2 * c3 * rob->L3b + c3 * (rob->ax2y - rob->ax3y) + s3 * rob->ax2z;
	p2z = -s2 * s3 * rob->L3b - s3 * (rob->ax2y - rob->ax3y) + c3 * rob->ax2z;

	f1 = pow(p2x, 2) + pow(p2y, 2);
	f2 = f1 + pow(p2z, 2);
	f3 = sqrt(f1);

	double inv_g12 = 1 / (2 * rob->L3 * f3);
	g1 = (pow(rob->Lpar, 2) - f2 - pow(rob->L3, 2)) * inv_g12;
	if (fabs(g1) > 1.0 + 1e-10) {
		return -1;
	} else if (fabs(g1) >= 1) {
		if (g1 > 0)
			angle = M_PI_2;
		else
			angle = -M_PI_2;
	} else {
		angle = asin(g1);
	}
	delta = atan2(p2y, p2x);

	tmp1 = fmod(delta + angle + M_PI, 2 * M_PI) - M_PI;
	tmp2 = fmod(delta - angle, 2 * M_PI) - M_PI;
	if (fabs(tmp2) < fabs(tmp1)) {
		q23 = tmp2;
		sign = -1;
	} else {
		q23 = tmp1;
		sign = 1;
	}

	s23 = sin(q23);
	c23 = cos(q23);
	g3 = -s3 * c23;

	qx = asin(g3);
	sx = sin(qx);
	cx = cos(qx);

	g4 = s23 / cx;
	th2 = asin(g4);

	th[0] = q[0];
	th[1] = th2;
	th[2] = -qx;
	th[3] = qx;
	th[4] = q[3];

	double dp2x_dq2 = -s2 * rob->L3b;
	double dp2y_dq2 = c2 * c3 * rob->L3b;
	double dp2y_dq3 = p2z;
	double dp2z_dq2 = -c2 * s3 * rob->L3b;
	double dp2z_dq3 = -p2y;

	g2 = p2y / p2x;
	double dg2_dq2 = (dp2y_dq2 - dp2x_dq2 * g2) / p2x;
	double dg2_dq3 = dp2y_dq3 / p2x;

	double dg11_dq2 = -2 * (p2x * dp2x_dq2 + p2y * dp2y_dq2 + p2z * dp2z_dq2);
	double dg11_dq3 = -2 * (p2y * dp2y_dq3 + p2z * dp2z_dq3);

	double f3_1 = 2 * rob->L3 / f3;
	double dg12_dq2 = (p2x * dp2x_dq2 + p2y * dp2y_dq2) * f3_1;
	double dg12_dq3 = p2y * dp2y_dq3 * f3_1;

	double dg1_dq2 = (dg11_dq2 - g1 * dg12_dq2) / (2 * rob->L3 * f3);
	double dg1_dq3 = (dg11_dq3 - g1 * dg12_dq3) / (2 * rob->L3 * f3);

	double ddelta_tmp = 1 / (1 + pow(g2, 2));
	double ddelta_dq2 = ddelta_tmp * dg2_dq2;
	double ddelta_dq3 = ddelta_tmp * dg2_dq3;

	double dangle_tmp = 1 / sqrt(1 - pow(g1, 2));
	double dangle_dq2 = dangle_tmp * dg1_dq2;
	double dangle_dq3 = dangle_tmp * dg1_dq3;

	double dq23_dq2 = ddelta_dq2 + sign * dangle_dq2;
	double dq23_dq3 = ddelta_dq3 + sign * dangle_dq3;

	double dg3_dq2 = s3 * s23 * dq23_dq2;
	double dg3_dq3 = -c3 * c23 + s3 * s23 * dq23_dq3;

	double dqx_tmp = 1 / sqrt(1 - pow(g3, 2));
	double dqx_dq2 = dqx_tmp * dg3_dq2;
	double dqx_dq3 = dqx_tmp * dg3_dq3;

	double dg4_dq2 = c23 / cx * dq23_dq2 + s23 * sx / pow(cx, 2) * dqx_dq2;
	double dg4_dq3 = c23 / cx * dq23_dq3 + s23 * sx / pow(cx, 2) * dqx_dq3;

	double dth2_tmp = 1 / sqrt(1 - pow(g4, 2));
	double dth2_dq2 = dth2_tmp * dg4_dq2;
	double dth2_dq3 = dth2_tmp * dg4_dq3;

	memset(dth_dq, 0, 4 * 5 * sizeof(double));
	dth_dq[0][0] = 1;
	dth_dq[1][1] = dth2_dq2;
	dth_dq[1][2] = dth2_dq3;
	dth_dq[2][1] = -dqx_dq2;
	dth_dq[2][2] = -dqx_dq3;
	dth_dq[3][1] = dqx_dq2;
	dth_dq[3][2] = dqx_dq3;
	dth_dq[4][3] = 1;

	double d2p2x_dq22 = -c2 * rob->L3b;
	double d2p2y_dq22 = -s2 * c3 * rob->L3b;
	double d2p2y_dq23 = -c2 * s3 * rob->L3b;
	double d2p2y_dq33 = -s2 * c3 * rob->L3b - c3 * (rob->ax2y - rob->ax3y) - s3 * rob->ax2z;
	double d2p2z_dq22 = s2 * s3 * rob->L3b;
	double d2p2z_dq23 = -c2 * c3 * rob->L3b;
	double d2p2z_dq33 = s2 * s3 * rob->L3b + s3 * (rob->ax2y - rob->ax3y) - c3 * rob->ax2z;

	double f3_2 = -1 / (4 * pow(rob->L3, 2));
	tmp1 = pow(dp2x_dq2, 2) + p2x * d2p2x_dq22 + pow(dp2y_dq2, 2) + p2y * d2p2y_dq22;
	double d2g11_dq22 = -2 * (tmp1 + pow(dp2z_dq2, 2) + p2z * d2p2z_dq22);
	double d2g12_dq22 = f3_1 * (f3_2 * pow(dg12_dq2, 2) + tmp1);
	tmp1 = dp2y_dq2 * dp2y_dq3 + p2y * d2p2y_dq23;
	double d2g11_dq23 = -2 * (tmp1 + dp2z_dq2 * dp2z_dq3 + p2z * d2p2z_dq23);
	double d2g12_dq23 = f3_1 * (f3_2 * dg12_dq2 * dg12_dq3 + tmp1);
	tmp1 = pow(dp2y_dq3, 2) + p2y * d2p2y_dq33;
	double d2g11_dq33 = -2 * (tmp1 + pow(dp2z_dq3, 2) + p2z * d2p2z_dq33);
	double d2g12_dq33 = f3_1 * (f3_2 * pow(dg12_dq3, 2) + tmp1);

	double d2g1_dq22 = inv_g12 * (d2g11_dq22 - 2 * dg12_dq2 * dg1_dq2 - d2g12_dq22 * g1);
	double d2g1_dq23 = inv_g12 * (d2g11_dq23 - dg12_dq2 * dg1_dq3 - dg12_dq3 * dg1_dq2
		- d2g12_dq23 * g1);
	double d2g1_dq33 = inv_g12 * (d2g11_dq33 - 2 * dg12_dq3 * dg1_dq3 - d2g12_dq33 * g1);

	double d2g2_dq22 = (d2p2y_dq22 - d2p2x_dq22 * g2 - 2 * dp2x_dq2 * dg2_dq2) / p2x;
	double d2g2_dq23 = (d2p2y_dq23 - dp2x_dq2 * dg2_dq3) / p2x;
	double d2g2_dq33 = d2p2y_dq33 / p2x;

	double d2delta_dq22 = ddelta_tmp * d2g2_dq22 - 2 * g2 * pow(ddelta_dq2, 2);
	double d2delta_dq23 = ddelta_tmp * d2g2_dq23 - 2 * g2 * ddelta_dq2 * ddelta_dq3;
	double d2delta_dq33 = ddelta_tmp * d2g2_dq33 - 2 * g2 * pow(ddelta_dq3, 2);

	double d2angle_dq22 = dangle_tmp * (g1 * pow(dangle_dq2, 2) + d2g1_dq22);
	double d2angle_dq23 = dangle_tmp * (g1 * dangle_dq2 * dangle_dq3 + d2g1_dq23);
	double d2angle_dq33 = dangle_tmp * (g1 * pow(dangle_dq3, 2) + d2g1_dq33);

	double d2q23_dq22 = d2delta_dq22 + sign * d2angle_dq22;
	double d2q23_dq23 = d2delta_dq23 + sign * d2angle_dq23;
	double d2q23_dq33 = d2delta_dq33 + sign * d2angle_dq33;

	tmp1 = c23 * pow(dq23_dq2, 2) + s23 * d2q23_dq22;
	double d2g3_dq22 = s3 * tmp1;
	double d2g3_dq23 = c3 * s23 * dq23_dq2 + s3 * c23 * dq23_dq2 * dq23_dq3 + s3 * s23 * d2q23_dq23;
	double d2g3_dq33 = s3 * c23 + 2 * c3 * s23 * dq23_dq3 + s3 * c23 * pow(dq23_dq3, 2) + s3 * s23 * d2q23_dq33;

	double d2qx_dq22 = dqx_tmp * (d2g3_dq22 + g3 * pow(dqx_dq2, 2));
	double d2qx_dq23 = dqx_tmp * (d2g3_dq23 + g3 * dqx_dq2 * dqx_dq3);
	double d2qx_dq33 = dqx_tmp * (d2g3_dq33 + g3 * pow(dqx_dq3, 2));

	tmp1 = 1 / pow(cx, 2);
	tmp2 = 1 / pow(cx, 3);
	double d2g4_dq22 = -s23 / cx * pow(dq23_dq2, 2) + 2 * c23 * sx * tmp1 * dq23_dq2 * dqx_dq2 + c23 / cx * d2q23_dq22 +
		s23 * (pow(sx, 2) + 1) * tmp2 * pow(dqx_dq2, 2) + s23 * sx * tmp1 * d2qx_dq22;
	double d2g4_dq23 = -s23 / cx * dq23_dq2 * dq23_dq3 + c23 * sx * tmp1 * dq23_dq2 * dqx_dq3 + c23 / cx * d2q23_dq23 +
		c23 * sx * tmp1 * dqx_dq2 * dq23_dq3 + s23 * (pow(sx, 2) + 1) * tmp2 * dqx_dq2 * dqx_dq3 + s23 * sx * tmp1 * d2qx_dq23;
	double d2g4_dq33 = -s23 / cx * pow(dq23_dq3, 2) + 2 * c23 * sx * tmp1 * dq23_dq3 * dqx_dq3 + c23 / cx * d2q23_dq33 +
		s23 * (pow(sx, 2) + 1) * tmp2 * pow(dqx_dq3, 2) + s23 * sx * tmp1 * d2qx_dq33;

	double d2th2_tmp = g4 / pow((1 - pow(g4, 2)), 3.0 / 2.0);
	double d2th2_dq22 = d2th2_tmp * pow(dg4_dq2, 2) + dth2_tmp * d2g4_dq22;
	double d2th2_dq23 = d2th2_tmp * dg4_dq2 * dg4_dq3 + dth2_tmp * d2g4_dq23;
	double d2th2_dq33 = d2th2_tmp * pow(dg4_dq3, 2) + dth2_tmp * d2g4_dq33;

	memset(d2th_dq2, 0, 4 * 5 * 4 * sizeof(double));
	d2th_dq2[1][1][1] = d2th2_dq22;
	d2th_dq2[1][1][2] = d2th2_dq23;
	d2th_dq2[1][2][1] = -d2qx_dq22;
	d2th_dq2[1][2][2] = -d2qx_dq23;
	d2th_dq2[1][3][1] = d2qx_dq22;
	d2th_dq2[1][3][2] = d2qx_dq23;

	d2th_dq2[2][1][1] = d2th2_dq23;
	d2th_dq2[2][1][2] = d2th2_dq33;
	d2th_dq2[2][2][1] = -d2qx_dq23;
	d2th_dq2[2][2][2] = -d2qx_dq33;
	d2th_dq2[2][3][1] = d2qx_dq23;
	d2th_dq2[2][3][2] = d2qx_dq33;
	return 0;
}

/**
 * Inverse of matrix with structure
 *
 * 			|a00 a01 a02 0|
 * a 	=	|a10 a11 a12 0|
 * 			|a20 a21 a22 0|
 * 			|a30 a31 0 a33|
*/
static void inv_44_special(const double a[4][4], double b[4][4])
{
	double a33[3][3];
	double b33[3][3];
	int i, j;

	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			a33[i][j] = a[i][j];
		}
	}

	inv_33(a33, b33);
	memset(b, 0, 4 * 4 * sizeof(double));
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			b[i][j] = b33[i][j];
		}
	}

	// Handle the case where a is transposed
	if (a[0][3] != 0 || a[1][3] != 0 || a[2][3] != 0) {
		for (i = 0; i < 3; i++) {
			for (j = 0; j < 3; j++) {
				b[i][3] -= a[j][3] * b[i][j] / a[3][3];
			}
		}
	} else {
		for (i = 0; i < 3; i++) {
			for (j = 0; j < 3; j++) {
				b[3][i] -= a[3][j] * b[j][i] / a[3][3];
			}
		}
	}
	b[3][3] = 1 / a[3][3];
}

/**
 * Inverse of matrix with structure
 *
 * 			|1  0   0  0|
 * a 	=	|0 a11 a12 0|
 * 			|0 a21 a22 0|
 * 			|0  0   0  1|
*/
static void inv_44_special2(const double a[4][4], double b[4][4])
{
	double det = a[1][1] * a[2][2] - a[1][2] * a[2][1];
	memset(b, 0, 4 * 4 * sizeof(double));
	b[0][0] = 1;
	b[1][1] = a[2][2] / det;
	b[1][2] = -a[1][2] / det;
	b[2][1] = -a[2][1] / det;
	b[2][2] = a[1][1] / det;
	b[3][3] = 1;
}

static void inv_vel_acc(const struct agile_pkm_model *rob, const double q[4], const double vel[4], const double acc[4], const double *tool, double qd[4], double qdd[4])
{
	int i, j, i_, j_;
	double q_[5];
	double qd_[5];
	double fk[4][4];
	double tmp66[6][6];
	double tmp4[4];
	double jact1[5][6];
	double jac[4][4];
	double jac_inv[4][4];
	double jdot1[6][5];
	double _tool[3];
	// If tool is not null, we're doing posonly transform
	if (tool == NULL) {
		_tool[0] = 0;
		_tool[1] = 0;
		_tool[2] = 0;
	} else {
		_tool[0] = tool[0];
		_tool[1] = tool[1];
		_tool[2] = tool[2];
	}
	struct model_lpoe_link lrob[] = {
		{
			.s = {0, 0, 0, 0, 0, 0},
			.trans = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}
		},
		{
			.s = {0, 0, 0, 0, 0, 1},
			.trans = {{1, 0, 0, rob->L1}, {0, 1, 0, rob->ax3y}, {0, 0, 1, 0}, {0, 0, 0, 1}}
		},
		{
			.s = {0, 0, 0, 0, 0, 1},
			.trans = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}
		},
		{
			.s = {0, 0, 0, 1, 0, 0},
			.trans = {{1, 0, 0, 0}, {0, 1, 0, rob->L2 - rob->d2}, {0, 0, 1, 0}, {0, 0, 0, 1}}
		},
		{
			.s = {0, 0, 0, 1, 0, 0},
			.trans = {{1, 0, 0, 0}, {0, 1, 0, rob->d2}, {0, 0, 1, -rob->a2}, {0, 0, 0, 1}}
		},
		{
			.s = {0, 0, 0, 0, 0, 1},
			.trans = {{1, 0, 0, _tool[0]}, {0, 1, 0, _tool[1]}, {0, 0, 1, _tool[2]}, {0, 0, 0, 1}}
		},
	};

	struct model_lpoe rob_ = {
		.n_joints = 5,
		.n_screw = 6,
		.lrob = lrob
	};

	q_[0] = q[0];
	q_[1] = q[1];
	q_[2] = q[2];
	q_[3] = -q[2];
	q_[4] = q[3];
	spatial_jacobian_transpose(&rob_, q_, fk, tmp66);
	spatial_to_normal_jacobian(tmp66, 5, fk, jact1);

	for (i = 0; i < 3; i++) {
		j_ = 0;
		for (j = 0; j < 4; j++) {
			if (j == 3) {
				j_++;
			}
			jac[i][j] = jact1[j_][i];
			if (j == 2) {
				jac[i][j] -= jact1[j_ + 1][i];
			}
			j_++;
		}
	}
	if (tool == NULL) {
		jac[3][0] = 1;
		jac[3][1] = 1;
		jac[3][2] = 0;
	} else {
		for (j = 0; j < 3; j++)
			jac[3][j] = 0;
	}
	jac[3][3] = 1;

	inv_44_special(jac, jac_inv);

	// qd = jac_inv * vel
	for (i = 0; i < 4; i++) {
		qd[i] = 0;
		for (j = 0; j < 4; j++) {
			qd[i] += jac_inv[i][j] * vel[j];
		}
	}

	qd_[0] = qd[0];
	qd_[1] = qd[1];
	qd_[2] = qd[2];
	qd_[3] = -qd[2];
	qd_[4] = qd[3];

	jacobian_jdot(lrob, 6, 5, q_, qd_, fk, (double *)tmp66, (double *)jdot1);

	// tmp = jdot*qd
	i_ = 0;
	for (i = 0; i < 4; i++) {
		if (i == 3) {
			i_ += 2;
		}
		j_ = 0;
		tmp4[i] = 0;
		for (j = 0; j < 4; j++) {
			if (j == 3) {
				j_++;
			}
			tmp4[i] += jdot1[i_][j_] * qd[j];
			if (j == 2) {
				tmp4[i] -= jdot1[i_][j_ + 1] * qd[j];
			}
			j_++;
		}
		i_++;
	}

	// qd = jac_inv * (acc - jdot * vel)
	for (i = 0; i < 4; i++) {
		qdd[i] = 0;
		for (j = 0; j < 4; j++) {
			qdd[i] += jac_inv[i][j] * (acc[j] - tmp4[j]);
		}
	}
}

int inv_with_vel_acc(const struct agile_pkm_model *rob, const double pos[3], const double orient_angle, const double vel[4], const double acc[4], double q[4], double qd[4], double qdd[4])
{
	int ret = inv(rob, pos, orient_angle, q);
	if (ret)
		return ret;
	inv_vel_acc(rob, q, vel, acc, NULL, qd, qdd);
	return 0;
}

int inv_posonly_with_vel_acc(const struct agile_pkm_model *rob, const double pos[3], const double vel[3], const double acc[3], const double q4, const double q4d, const double q4dd, const double tool[3], double q[4], double qd[4], double qdd[4])
{
	int ret = inv_posonly(rob, pos, q4, tool, q);
	if (ret)
		return ret;
	double _vel[4] = {vel[0], vel[1], vel[2], q4d};
	double _acc[4] = {acc[0], acc[1], acc[2], q4dd};
	inv_vel_acc(rob, q, _vel, _acc, tool, qd, qdd);
	return 0;
}

int fwd_with_vel_acc(const struct agile_pkm_model *rob, const double q[4], const double qd[4], const double qdd[4], double tcp_out[4][4], double *orient_angle, double vel[4], double acc[4])
{
	int ret, i, j;
	double q_[5];
	double qd_[5];
	double qdd_[5];
	double jac[6][5];
	double jdot[6][5];
	double fk[6][4][4];
	struct model_lpoe_link lrob[] = {
		{
			.s = {0, 0, 0, 0, 0, 0},
			.trans = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}
		},
		{
			.s = {0, 0, 0, 0, 0, 1},
			.trans = {{1, 0, 0, rob->L1}, {0, 1, 0, rob->ax3y}, {0, 0, 1, 0}, {0, 0, 0, 1}}
		},
		{
			.s = {0, 0, 0, 0, 0, 1},
			.trans = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}
		},
		{
			.s = {0, 0, 0, 1, 0, 0},
			.trans = {{1, 0, 0, 0}, {0, 1, 0, rob->L2 - rob->d2}, {0, 0, 1, 0}, {0, 0, 0, 1}}
		},
		{
			.s = {0, 0, 0, 1, 0, 0},
			.trans = {{1, 0, 0, 0}, {0, 1, 0, rob->d2}, {0, 0, 1, -rob->a2}, {0, 0, 0, 1}}
		},
		{
			.s = {0, 0, 0, 0, 0, 1},
			.trans = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}
		},
	};

	ret = fwd(rob, q, tcp_out, orient_angle);
	if (ret)
		return ret;

	q_[0] = q[0];
	q_[1] = q[1];
	q_[2] = q[2];
	q_[3] = -q[2];
	q_[4] = q[3];
	// spatial_jacobian_transpose_full_std(lrob, 6, q_, fk, tmp66);
	// spatial_to_normal_jacobian(tmp66, 5, fk[5], jact);

	qd_[0] = qd[0];
	qd_[1] = qd[1];
	qd_[2] = qd[2];
	qd_[3] = -qd[2];
	qd_[4] = qd[3];

	jacobian_jdot(lrob, 6, 5, q_, qd_, fk[5], (double *)jac, (double *)jdot);
	for (i = 0; i < 4; i++) {
		vel[i] = 0;
		for (j = 0; j < 5; j++) {
			if (i == 3) {
				vel[i] += jac[i + 2][j] * qd_[j];
			} else {
				vel[i] += jac[i][j] * qd_[j];
			}
		}
	}

	qdd_[0] = qdd[0];
	qdd_[1] = qdd[1];
	qdd_[2] = qdd[2];
	qdd_[3] = -qdd[2];
	qdd_[4] = qdd[3];

	for (i = 0; i < 4; i++) {
		acc[i] = 0;
		for (j = 0; j < 5; j++) {
			if (i == 3) {
				acc[i] += jac[i + 2][j] * qdd_[j] + jdot[i + 2][j] * qd_[j];
			} else {
				acc[i] += jac[i][j] * qdd_[j] + jdot[i][j] * qd_[j];
			}
		}
	}
	return 0;
}

int fwd_elbow_with_vel_acc(const struct agile_pkm_model *rob, const double q1, const double qd1, const double qdd1, double elbow_out[4][4], double elbow_vel[4], double elbow_acc[4])
{
	int ret = fwd_elbow(rob, q1, elbow_out);
	if (ret)
		return ret;

	double s = sin(q1);
	double c = cos(q1);
	double tmp1 = (rob->L1 * s + rob->ax3y * c);
	double tmp2 = (rob->L1 * c - rob->ax3y * s);

	elbow_vel[0] = -tmp1 * qd1;
	elbow_vel[1] = tmp2 * qd1;
	elbow_vel[2] = 0;
	elbow_vel[3] = qd1;

	elbow_acc[0] = -tmp1 * qdd1 - tmp2 * qd1 * qd1;
	elbow_acc[1] = tmp2 * qdd1 - tmp1 * qd1 * qd1;
	elbow_acc[2] = 0;
	elbow_acc[3] = qdd1;
	return 0;
}

int joint_to_drive_with_vel_acc(const struct agile_pkm_model *rob, const double joint_pos[4], const double joint_vel[4], const double joint_acc[4], double drive_pos[4], double drive_vel[4], double drive_acc[4])
{
	int i, j, k;
	double th[5];
	double dth_dq[5][4];
	double d2th_dq2[4][5][4];
	double tmp4[4];
	double jac[4][4];
	double jac_inv[4][4];
	double jdot[4][4];

	int ret = joint_to_drive(rob, joint_pos, drive_pos);
	if (ret)
		return ret;

	calculate_derivatives_drive_to_joint(rob, drive_pos, th, dth_dq, d2th_dq2);

	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			if (i == 3) {
				jac[i][j] = dth_dq[i + 1][j];
			} else {
				jac[i][j] = dth_dq[i][j];
			}
		}
	}

	inv_44_special2(jac, jac_inv);

	for (i = 0; i < 4; i++) {
		drive_vel[i] = 0;
		for (j = 0; j < 4; j++) {
			drive_vel[i] += jac_inv[i][j] * joint_vel[j];
		}
	}

	// jdot = sum([d2th_dq2[j, :, :] * qd[j] for j in 0:3])
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			jdot[i][j] = 0;
			for (k = 0; k < 4; k++) {
				if (i == 3) {
					jdot[i][j] += d2th_dq2[k][i + 1][j] * drive_vel[k];
				} else {
					jdot[i][j] += d2th_dq2[k][i][j] * drive_vel[k];
				}
			}
		}
	}

	// tmp = jdot*qd
	for (i = 0; i < 4; i++) {
		tmp4[i] = 0;
		for (j = 0; j < 4; j++) {
			tmp4[i] += jdot[i][j] * drive_vel[j];
		}
	}

	// qdd = jac_inv * (acc - jdot*qd)
	for (i = 0; i < 4; i++) {
		drive_acc[i] = 0;
		for (j = 0; j < 4; j++) {
			drive_acc[i] += jac_inv[i][j] * (joint_acc[j] - tmp4[j]);
		}
	}
	return 0;
}

int drive_to_joint_with_vel_acc(const struct agile_pkm_model *rob, const double drive_pos[4], const double drive_vel[4], const double drive_acc[4], double joint_pos[4], double joint_vel[4], double joint_acc[4])
{
	int i, j, k;
	double th[5];
	double dth_dq[5][4];
	double d2th_dq2[4][5][4];
	double jdot[4][4];

	int ret = drive_to_joint(rob, drive_pos, joint_pos);
	if (ret)
		return ret;

	calculate_derivatives_drive_to_joint(rob, drive_pos, th, dth_dq, d2th_dq2);

	for (i = 0; i < 4; i++) {
		joint_vel[i] = 0;
		for (j = 0; j < 4; j++) {
			if (i == 3) {
				joint_vel[i] += dth_dq[i + 1][j] * drive_vel[j];
			} else {
				joint_vel[i] += dth_dq[i][j] * drive_vel[j];
			}
		}
	}

	// jdot = sum([d2th_dq2[j, :, :] * qd[j] for j in 0:3])
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			jdot[i][j] = 0;
			for (k = 0; k < 4; k++) {
				if (i == 3) {
					jdot[i][j] += d2th_dq2[k][i + 1][j] * drive_vel[k];
				} else {
					jdot[i][j] += d2th_dq2[k][i][j] * drive_vel[k];
				}
			}
		}
	}

	// joint_acc = jac * drive_acc + jdot*drive_vel
	for (i = 0; i < 4; i++) {
		joint_acc[i] = 0;
		for (j = 0; j < 4; j++) {
			if (i == 3) {
				joint_acc[i] += dth_dq[i + 1][j] * drive_acc[j] + jdot[i][j] * drive_vel[j];
			} else {
				joint_acc[i] += dth_dq[i][j] * drive_acc[j] + jdot[i][j] * drive_vel[j];
			}
		}
	}

	return 0;
}

int cart_to_drive_with_vel_acc(const struct agile_pkm_model *rob, const double pos[3], const double orient_angle, const double vel[4], const double acc[4], double q[4], double qd[4], double qdd[4])
{
	double joint_pos[4];
	double joint_vel[4];
	double joint_acc[4];
	int ret;

	ret = inv_with_vel_acc(rob, pos, orient_angle, vel, acc, joint_pos, joint_vel, joint_acc);
	if (ret)
		return ret;

	ret = joint_to_drive_with_vel_acc(rob, joint_pos, joint_vel, joint_acc, q, qd, qdd);
	return ret;
}

int drive_to_cart_with_vel_acc(const struct agile_pkm_model *rob, const double q[4], const double qd[4], const double qdd[4], double tcp_out[4][4], double *orient_angle, double vel[4], double acc[4])
{
	double joint_pos[4];
	double joint_vel[4];
	double joint_acc[4];
	int ret;

	ret = drive_to_joint_with_vel_acc(rob, q, qd, qdd, joint_pos, joint_vel, joint_acc);
	if (ret)
		return ret;

	ret = fwd_with_vel_acc(rob, joint_pos, joint_vel, joint_acc, tcp_out, orient_angle, vel, acc);
	return ret;
}

#ifdef EXTRA_CALC
/*!
Inverse of 3x3 matrix with element (2, 0) being zero

Note that the input/output is 4x4, but only the upper left
3x3 submatrix is considered
*/
int inv_33_special(const double a[4][4], double a_inv[4][4])
{
        double det_a = a[0][0]*(a[1][1]*a[2][2] - a[2][1]*a[1][2])
                        - a[1][0]*(a[0][1]*a[2][2] - a[0][2]*a[2][1]);
        if (fabs(det_a) < 1e-10)
                return -1;

        double det_a_inv = 1/det_a;

        a_inv[0][0] = det_a_inv*(a[1][1]*a[2][2] - a[1][2]*a[2][1]);
        a_inv[0][1] = det_a_inv*(a[0][2]*a[2][1] - a[2][2]*a[0][1]);
        a_inv[0][2] = det_a_inv*(a[0][1]*a[1][2] - a[0][2]*a[1][1]);
        a_inv[1][0] = -det_a_inv*a[1][0]*a[2][2];
        a_inv[1][1] = det_a_inv*a[0][0]*a[2][2];
        a_inv[1][2] = det_a_inv*(a[0][2]*a[1][0] - a[1][2]*a[0][0]);
        a_inv[2][0] = det_a_inv*a[1][0]*a[2][1];
        a_inv[2][1] = -det_a_inv*a[0][0]*a[2][1];
        a_inv[2][2] = det_a_inv*(a[0][0]*a[1][1] - a[0][1]*a[1][0]);
        return 0;
}

/**
 * Implementation using numeric differentiation. Does support backhoe.
*/
int cart_to_drive_with_vel_acc_num(const struct agile_pkm_model *rob, const double pos[3], const double orient, const double v[4], const double a[4], double q_out[4], double qd_out[4], double qdd_out[4])
{
        int ret;
        double fwdres[4][4];
        double fwdnom[4];
        double ori_res;
        double jac[4][4];
        int i, j, k;
        double f_plus[4][4];
        double f_minus[4][4];
        double d[4];
        double jac_inv[4][4];
        double tmp, tmp2, tmp3, tmp4;
        double gacc[4][4][4];
        double gdot[4][4];
        double gvec[4];

        ret = cart_to_drive(rob, pos, orient, q_out);
        if (ret != 0) {
                return ret;
        }

        ret = drive_to_cart(rob, q_out, fwdres, &ori_res);
        if (ret != 0)
                return ret;
        fwdnom[0] = fwdres[0][3];
        fwdnom[1] = fwdres[1][3];
        fwdnom[2] = fwdres[2][3];
        fwdnom[3] = ori_res;

        memset(jac, 0, 4 * 4 * sizeof(double));
        jac[0][0] = -pos[1];
        jac[1][0] = pos[0];
        jac[3][0] = 1;

        if (!rob->bh)
                jac[3][3] = 1;

        memcpy(d, q_out, 4 * sizeof(double));
        for (i = 0; i < 4; i++) {
                if (i < 3)
                        d[i] = q_out[i] + DELTA1;
                else
                        d[i] = q_out[i] + DELTA1*10;
                ret = drive_to_cart(rob, d, fwdres, &ori_res);
                if (ret != 0)
                        return ret;
                for (j = 0; j < 3; j++)
                        f_plus[j][i] = fwdres[j][3];
                f_plus[3][i] = ori_res;
                if (i < 3)
                        d[i] = q_out[i] - DELTA1;
                else
                        d[i] = q_out[i] - DELTA1*10;
                ret = drive_to_cart(rob, d, fwdres, &ori_res);
                if (ret != 0)
                        return ret;
                for (j = 0; j < 3; j++)
                        f_minus[j][i] = fwdres[j][3];
                f_minus[3][i] = ori_res;
                d[i] = q_out[i];
                if (i > 0 && rob->bh) {
			for (j = 1; j < 4; j++) {
				if (i < 3)
					jac[j][i] = (f_plus[j][i] - f_minus[j][i])/(2*DELTA1);
				else
					jac[j][i] = (f_plus[j][i] - f_minus[j][i])/(2*DELTA1*10);

			}
                }
        }

        ret = inv_33_special(jac, jac_inv);
        if (ret != 0 || fabs(jac[3][3]) < 1e-10)
                return ret;

        tmp = 1/jac[3][3];
        for (i = 0; i < 3; i++) {
                jac_inv[i][3] = 0;
                jac_inv[3][i] = 0;
                for (j = 0; j < 3; j++)
                        jac_inv[3][i] -= jac[3][j]*jac_inv[j][i];
                jac_inv[3][i] *= tmp;
        }
        jac_inv[3][3] = tmp;

        for (i = 0; i < 4; i++) {
                qd_out[i] = 0;
                for (j = 0; j < 4; j++)
                        qd_out[i] += jac_inv[i][j]*v[j];
        }

        memset(gacc, 0, 4 * 4 * 4 * sizeof(double));
        memset(gdot, 0, 4 * 4 * sizeof(double));
        tmp = 1/pow(DELTA1, 2);
        tmp3 = 1/pow(DELTA1*10, 2);
        tmp4 = tmp3*10;
        gacc[3][3][3] = (f_plus[3][3]*tmp3 - 2*fwdnom[3]*tmp3 + f_minus[3][3]*tmp3);
        for (i = 0; i < 4; i++) {
                if (i < 3) {
                        for (j = 0; j < 4; j++) {
                                gacc[i][j][i] = (f_plus[j][i]*tmp - 2*fwdnom[j]*tmp
                                                + f_minus[j][i]*tmp);
                        }
                }
                for (j = i + 1; j < 4; j++) {
                        memcpy(d, q_out, 4 * sizeof(double));
                        d[i] += DELTA1;
                        if (j < 3) {
                                d[j] += DELTA1;
                        } else {
                                d[j] += DELTA1*10;
                        }
                        ret = drive_to_cart(rob, d, fwdres, &ori_res);
                        if (ret != 0)
                                return ret;
                        for (k = 0; k < 3; k++) {
                                if (j == 3 || i == 3)
                                        tmp2 = (fwdres[k][3]*tmp4 - f_plus[k][j]*tmp4
                                                        - f_plus[k][i]*tmp4 + fwdnom[k]*tmp4);
                                else
                                        tmp2 = (fwdres[k][3]*tmp - f_plus[k][j]*tmp
                                                        - f_plus[k][i]*tmp + fwdnom[k]*tmp);
                                gacc[j][k][i] += tmp2;
                                gacc[i][k][j] += tmp2;
                        }
                        if (j == 3 || i == 3)
                                tmp2 = (ori_res*tmp4 - f_plus[3][j]*tmp4
                                                - f_plus[3][i]*tmp4 + fwdnom[3]*tmp4);
                        else
                                tmp2 = (ori_res*tmp - f_plus[3][j]*tmp
                                                - f_plus[3][i]*tmp + fwdnom[3]*tmp);
                        gacc[j][3][i] += tmp2;
                        gacc[i][3][j] += tmp2;
                }
                for (j = 0; j < 4; j++)
                        for (k = 0; k < 4; k++)
                                gdot[j][k] += gacc[i][j][k]*qd_out[i];
        }
        for (i = 0; i < 4; i++) {
                gvec[i] = 0;
                for (j = 0; j < 4; j++)
                        gvec[i] += gdot[i][j]*qd_out[j];
        }
        for (i = 0; i < 4; i++) {
                qdd_out[i] = 0;
                for (j = 0; j < 4; j++)
                        qdd_out[i] += jac_inv[i][j]*(a[j] - gvec[j]);
        }
        return 0;
}
#endif
