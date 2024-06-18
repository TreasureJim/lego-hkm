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
#endif

static int bh_wrist_fwd(const struct agile_pkm_backhoe *bh, double pos_b2_mm[3], double tf_rot_center_mm[4][4], bh_wrist_fwd_ret_t *out)
{
	if (out == NULL) {
		return -1;
	}

	struct agile_pkm_wrist_backhoe *wbh = bh->wbh;
#ifdef EXTRA_CALC
	struct agile_pkm_elbow_backhoe *ebh = bh->ebh;
#endif

	double tf_inv[4][4];
	eye(4, (double *)tf_inv);

	double tf_rot_center_mm_33[3][3];
	copy_44_33(tf_rot_center_mm, tf_rot_center_mm_33);
	double tf_rot_center_mm_33T[3][3];
	transpose((double *)tf_rot_center_mm_33, (double *)tf_rot_center_mm_33T, 3);

	for (size_t y = 0; y < 3; y++) {
		for (size_t x = 0; x < 3; x++) {
			tf_inv[y][x] = tf_rot_center_mm_33T[y][x];
		}
	}

	double tf_rot_center_mm_c3[3] = {tf_rot_center_mm[0][3], tf_rot_center_mm[1][3], tf_rot_center_mm[2][3]};
	double tmp3[3];
	mul33_3(tf_rot_center_mm_33T, tf_rot_center_mm_c3, tmp3);
	tf_inv[0][3] = -tmp3[0];
	tf_inv[1][3] = -tmp3[1];
	tf_inv[2][3] = -tmp3[2];

	double p_b2_rc[3];
	copy_44_33(tf_inv, tf_rot_center_mm_33);
	mul33_3(tf_rot_center_mm_33, pos_b2_mm, p_b2_rc);
	p_b2_rc[0] += tf_inv[0][3];
	p_b2_rc[1] += tf_inv[1][3];
	p_b2_rc[2] += tf_inv[2][3];

	double sum = pow(p_b2_rc[0], 2);
	sum += pow(p_b2_rc[1], 2);
	sum += pow(p_b2_rc[2], 2);

	double sin_gamma = (pow(wbh->La1, 2) + sum - pow(bh->L_rod, 2)) /
		(2.0 * wbh->La1 * sqrt(pow(p_b2_rc[0], 2) + pow(p_b2_rc[1], 2)));

	if (fabs(sin_gamma) > 1.0) {
		return -1;
	}

	double gamma = asin(sin_gamma);
	double delta = atan2(p_b2_rc[0], p_b2_rc[1]);
	// # alpha_c1 = -delta + gamma
	double alpha_c2 = -delta + M_PI - gamma;

	double alpha_c = alpha_c2;
	if (alpha_c > wbh->alpha_c_max[0])
		alpha_c -= M_PI * 2;
	if (alpha_c > wbh->alpha_c_max[1] || alpha_c < wbh->alpha_c_min) {
		return -1;
	}

#ifdef EXTRA_CALC
	double pb[] = {wbh->La1 * cos(alpha_c), wbh->La1 * sin(alpha_c), 0};
	double pb_l[3] = {pb[0] - p_b2_rc[0], pb[1] - p_b2_rc[1], pb[2] - p_b2_rc[2]};

	double tf_rot_center_mm33[3][3];
	copy_44_33(tf_rot_center_mm, tf_rot_center_mm33);
	mul33_3(tf_rot_center_mm33, pb, tmp3);

	double pb_Lb_ebh[3] = {tf_rot_center_mm[0][3] + tmp3[0] + ebh->dxb,
				   tf_rot_center_mm[1][3] + tmp3[1] + ebh->dy,
				   tf_rot_center_mm[2][3] + tmp3[2] + ebh->z_offs};
	pb_Lb_ebh[2] = 0;

	double pb_l_proj[] = {pb_l[0], pb_l[1], 0};
	double r2 = atan2(pb_l[2], norm(pb_l_proj, 3));

	double pb_l_ = cos(r2) * bh->L_rod;
	double tmp = pow(pb_Lb_ebh[0], 2) + pow(pb_Lb_ebh[1], 2) + pow(pb_Lb_ebh[2], 2);
	double r1 = acos((pow(ebh->Lb2, 2) + pow(pb_l_, 2) - tmp) / (2 * ebh->Lb2 * pb_l_)) - M_PI_2;
#endif

	double t1 = (wbh->La2 - wbh->Lc2) * cos(alpha_c) - wbh->Lc3 * cos(alpha_c + wbh->c_ang);
	double t2 = wbh->Lh + (wbh->La2 - wbh->Lc2) * sin(alpha_c) - wbh->Lc3 * sin(alpha_c + wbh->c_ang);
	sin_gamma = (pow(wbh->Ld, 2) - pow(t1, 2) - pow(t2, 2) - pow(wbh->Le2, 2)) /
		(2.0 * wbh->Le2 * sqrt(pow(t1, 2) + pow(t2, 2)));
	if (fabs(sin_gamma) > 1) {
		return -1;
	}
	gamma = asin(sin_gamma);
	delta = atan2(t1, t2);
	double alpha_e_1 = -delta + gamma;
	// # alpha_e_2 = -delta + np.pi - gamma  # this solution can be discarded

	double alpha_e = alpha_e_1;
	if (alpha_e < wbh->alpha_e_min[0])
		alpha_e += M_PI * 2;
	if (alpha_e < wbh->alpha_e_min[1] || alpha_e > wbh->alpha_e_max) {
		return -1;
	}

#ifdef EXTRA_CALC
	double pc1_a[2] = {wbh->Lc3 * cos(alpha_c + wbh->c_ang), -wbh->Lc3 * sin(alpha_c + wbh->c_ang)};
	double pc1 = norm(pc1_a, 2);
	double pe1_a[2] = {wbh->Lc2 * cos(alpha_c) + -wbh->La2 * cos(alpha_c) - wbh->Le2 * cos(alpha_e), wbh->Lc2 * sin(alpha_c) + -wbh->Lh - wbh->La2 * sin(alpha_c) - wbh->Le2 * sin(alpha_e)};
	double pe1 = norm(pe1_a, 2);
	// double pe1_pc1_diff = pe1 - pc1;
	// double Ld_2 = norm(&pe1_pc1_diff, 1);
	double alpha_d = M_PI - acos((pow(pc1, 2) + pow(wbh->Ld, 2) - pow(pe1, 2)) / (2 * pc1 * wbh->Ld)) - (M_PI_2 - wbh->c_ang);
#endif

	t1 = -wbh->Le1 * cos(alpha_e - wbh->e_ang) + wbh->La2 * cos(alpha_c);
	t2 = -wbh->Le1 * sin(alpha_e - wbh->e_ang) + wbh->La2 * sin(alpha_c);
	sin_gamma = (pow(wbh->Lf, 2) - pow(t1, 2) - pow(t2, 2) - pow(wbh->Lg, 2)) /
		(2.0 * wbh->Lg * sqrt(pow(t1, 2) + pow(t2, 2)));
	if (fabs(sin_gamma) > 1) {
		return -1;
	}
	gamma = asin(sin_gamma);
	delta = atan2(t2, t1);
	if (delta > 0.0) {
		delta -= 2.0 * M_PI;
	}
	// # q4_1 = delta + gamma - q4_offs
	double q4_2 = delta + M_PI - gamma;
	double q4 = q4_2;
	q4 -= wbh->q4_offs;

#ifdef EXTRA_CALC
	double pe2[2] = {-wbh->La2 * cos(alpha_c), -wbh->La2 * sin(alpha_c)};
	double pg[2] = {wbh->Lg * sin(q4), -wbh->Lg * cos(q4)};

	double a_a[2] = {pg[0] - pe2[0], pg[1] - pe2[1]};
	double a = norm(a_a, 2);
	double b = wbh->Lf;
	double c = wbh->Le1;

	double alpha_f = acos((pow(b, 2) + pow(c, 2) - pow(a, 2)) / (2 * b * c));
	alpha_f = M_PI - alpha_f - (M_PI_2 - wbh->e_ang);
#endif

	out->q4 = q4;
#ifdef EXTRA_CALC
	out->r1 = -r1;
	out->r2 = -r2;
	out->alpha_c = alpha_c;
	out->alpha_d = alpha_d;
	out->alpha_ec_diff = alpha_e - alpha_c;
	out->alpha_f = -alpha_f;
#endif

	return 0;
}

static int bh_wrist_inv(const struct agile_pkm_wrist_backhoe *wbh, double q4, double out[3])
{

	if (q4 < wbh->q4_angle_lims[0] || q4 > wbh->q4_angle_lims[1]) {
		return -1;
	}

	double q4_ = q4 + wbh->q4_offs;
	double alpha_c = wbh->alpha_c_pol[1] + q4_ * wbh->alpha_c_pol[0];
	double alpha_e = wbh->alpha_e_pol[1] + q4_ * wbh->alpha_e_pol[0];

	int success = 0;

	for (size_t k = 0; k < 20; k++) {
		double sa = sin(alpha_c);
		double ca = cos(alpha_c);
		double sa_ = sin(alpha_c + wbh->c_ang);
		double ca_ = cos(alpha_c + wbh->c_ang);
		double se = sin(alpha_e);
		double ce = cos(alpha_e);
		double se_ = sin(alpha_e - wbh->e_ang);
		double ce_ = cos(alpha_e - wbh->e_ang);

		double t1 = wbh->Le1 * ce_ - wbh->La2 * ca - wbh->Lg * sin(q4_);
		double t2 = wbh->Le1 * se_ - wbh->La2 * sa + wbh->Lg * cos(q4_);
		double t3 = (wbh->La2 - wbh->Lc2) * ca - wbh->Lc3 * ca_ + wbh->Le2 * ce;
		double t4 = wbh->Lh + (wbh->La2 - wbh->Lc2) * sa - wbh->Lc3 * sa_ + wbh->Le2 * se;

		double f[] = {pow(t1, 2) + pow(t2, 2) - pow(wbh->Lf, 2), pow(t3, 2) + pow(t4, 2) - pow(wbh->Ld, 2)};

		double max = 0;
		for (size_t i = 0; i < sizeof(f) / sizeof(*f); i++) {
			double a = fabs(f[i]);
			max = fmax(a, max);
		}

		if (max < 1e-10) {
			success = 1;
			break;
		}

		double jf[2][2] = {{wbh->La2 * (t1 * sa - t2 * ca),
					wbh->Le1 * (t2 * ce_ - t1 * se_)},
				   {(wbh->La2 - wbh->Lc2) * (t4 * ca - t3 * sa) + wbh->Lc3 * (t3 * sa_ - t4 * ca_),
					wbh->Le2 * (t4 * ce - t3 * se)}};
		for (size_t y = 0; y < 2; y++) {
			for (size_t x = 0; x < 2; x++) {
				jf[y][x] *= 2.0;
			}
		}

		double dx_tmp[2];
		if (solve_22_2(jf, f, dx_tmp) != 0) {
			return -1;
		}

		double dx[2];
		dx[0] = -dx_tmp[0];
		dx[1] = -dx_tmp[1];

		alpha_c += dx[0];
		alpha_e += dx[1];
	}

	if (!success) {
		return -1;
	}

	double pos_pb_mm[] = {wbh->La1 * cos(alpha_c), wbh->La1 * sin(alpha_c), wbh->La1 * 0.0};

	out[0] = pos_pb_mm[0];
	out[1] = pos_pb_mm[1];
	out[2] = pos_pb_mm[2];

	return 0;
}

typedef struct {
	double pos_out_mm[3];
#ifdef EXTRA_CALC
	double bs_ang;
	double a_extra;
	double c_ang;
	double theta_b;
#endif
} bh_elbow_fwd_ret_t;

static int bh_elbow_fwd(const struct agile_pkm_elbow_backhoe *ebh, double delta_b, bh_elbow_fwd_ret_t *out)
{
	double bs = ebh->bs0 + delta_b;
	double cos_a1 = (pow(ebh->ah, 2) + pow(ebh->La1, 2) - pow(bs, 2)) / (2.0 * ebh->ah * ebh->La1);

	if (fabs(cos_a1) > 1) {
		return -1;
	}

	double a1 = acos(cos_a1);
#ifdef EXTRA_CALC
	double cos_bs = (pow(ebh->ah, 2) + pow(bs, 2) - pow(ebh->La1, 2)) / (2.0 * ebh->ah * bs);
	double bs_ang = acos(cos_bs);
	double a_extra = ebh->a0 + a1;
#endif

	double a = ebh->a0 + ebh->a_ang + a1;
	double t1 = ebh->dx + ebh->La2 * cos(a);
	double t2 = -ebh->dy + ebh->La2 * sin(a);
	double t = pow(t1, 2) + pow(t2, 2);
	double sin_gamma = (pow(ebh->Lc, 2) - t - pow(ebh->Lb1, 2)) / (2.0 * ebh->Lb1 * sqrt(t));

	if (fabs(sin_gamma) > 1) {
		return -1;
	}

	double delta = atan2(t2, t1);

	if (delta < 0.0) {
		delta += 2.0 * M_PI;
	}
	double gamma = asin(sin_gamma);

	double theta_b = delta + gamma;

#ifdef EXTRA_CALC
	double pb1[] = {ebh->dx + ebh->Lb1 * sin(theta_b), ebh->dy + ebh->Lb1 * cos(theta_b)};
	double pb1h = norm(pb1, 2);
	double cos_c_ang = (pow(ebh->La2, 2) + pow(ebh->Lc, 2) - pow(pb1h, 2)) / (2 * ebh->La2 * ebh->Lc);
	double c_ang = -acos(cos_c_ang) + ebh->a_ang;
#endif

	out->pos_out_mm[0] = -ebh->dxb + ebh->Lb2 * sin(theta_b);
	out->pos_out_mm[1] = -ebh->dy - ebh->Lb2 * cos(theta_b);
	out->pos_out_mm[2] = -ebh->z_offs;
#ifdef EXTRA_CALC
	out->bs_ang = -bs_ang;
	out->a_extra = a_extra;
	out->c_ang = c_ang;
	out->theta_b = theta_b;
#endif

	return 0;
}

static int bh_elbow_inv(const struct agile_pkm_elbow_backhoe *ebh, double L_rod, double pos_pb_mm[3], double tf_rot_center_mm[4][4], double *out)
{
	if (out == NULL) {
		return -1;
	}

	double tf_rot_center_mm_33[3][3];
	double tf_rot_center_mm_c3[3] = {tf_rot_center_mm[0][3],
					 tf_rot_center_mm[1][3],
					 tf_rot_center_mm[2][3]};

	copy_44_33(tf_rot_center_mm, tf_rot_center_mm_33);

	// # pb_cc : point p_b specified in cardan center coordinate frame
	double tmp[3];
	mul33_3(tf_rot_center_mm_33, pos_pb_mm, tmp);
	double pb_cc[3] = {tmp[0] + tf_rot_center_mm_c3[0],
			   tmp[1] + tf_rot_center_mm_c3[1],
			   tmp[2] + tf_rot_center_mm_c3[2]};

	double sin_gamma = (pow(L_rod, 2) - pow(ebh->dxb + pb_cc[0], 2) -
		pow(ebh->dy + pb_cc[1], 2) -
		pow(ebh->z_offs + pb_cc[2], 2) - pow(ebh->Lb2, 2)) /
		(2.0 * ebh->Lb2 * sqrt(pow(ebh->dxb + pb_cc[0], 2) + pow(ebh->dy + pb_cc[1], 2)));

	if (fabs(sin_gamma) > 1) {
		return -1;
	}
	double gamma = asin(sin_gamma);
	double delta = atan2(ebh->dy + pb_cc[1], ebh->dxb + pb_cc[0]);
	double theta_b1 = delta - gamma;
	// # theta_b2 = delta + gamma - np.pi

	double theta_b = theta_b1;

	double t1 = ebh->dx + ebh->Lb1 * sin(theta_b);
	double t2 = -ebh->dy - ebh->Lb1 * cos(theta_b);
	double t = pow(t1, 2) + pow(t2, 2);
	sin_gamma = (pow(ebh->Lc, 2) - t - pow(ebh->La2, 2)) / (2.0 * ebh->La2 * sqrt(t));

	if (fabs(sin_gamma) > 1) {
		return -1;
	}

	delta = atan2(t1, t2);
	gamma = asin(sin_gamma);

	// a_1 = -delta + gamma
	double a = -delta + M_PI - gamma;
	double a1 = a - ebh->a0 - ebh->a_ang;

	double b1 = sqrt(pow(ebh->ah, 2) + pow(ebh->La1, 2) - 2.0 * ebh->ah * ebh->La1 * cos(a1));
	double delta_b = b1 - ebh->bs0;

	*out = delta_b;

	return 0;
}

int ax4_fwd(const struct agile_pkm_backhoe *bh, double delta_ballscrew, double tf_rot_center[4][4], ax4_fwd_ret_t *out)
{
	bh_elbow_fwd_ret_t bh_elbow_fwd_ret;
	if (bh_elbow_fwd(bh->ebh, delta_ballscrew, &bh_elbow_fwd_ret) < 0) {
		return -1;
	}

	bh_wrist_fwd_ret_t bh_wrist_fwd_ret;
	if (bh_wrist_fwd(bh, bh_elbow_fwd_ret.pos_out_mm, tf_rot_center, &bh_wrist_fwd_ret) < 0) {
		return -1;
	}

#ifdef EXTRA_CALC
	out->r1 = bh_wrist_fwd_ret.r1;
	out->r2 = bh_wrist_fwd_ret.r2;
	out->alpha_c = bh_wrist_fwd_ret.alpha_c;
	out->alpha_d = bh_wrist_fwd_ret.alpha_d;
	out->alpha_ec_diff = bh_wrist_fwd_ret.alpha_ec_diff;
	out->alpha_f = bh_wrist_fwd_ret.alpha_f;

	out->bs_ang = bh_elbow_fwd_ret.bs_ang;
	out->a_extra = bh_elbow_fwd_ret.a_extra;
	out->c_ang = bh_elbow_fwd_ret.c_ang;
	out->theta_b = bh_elbow_fwd_ret.theta_b;
#endif
	out->q4 = bh_wrist_fwd_ret.q4;

	return 0;
}

int ax4_inv(const struct agile_pkm_backhoe *bh, double q4, double tf_rot_center[4][4], double *out)
{
	if (out == NULL) {
		return -1;
	}

	double pos_pb_mm[3];

	if (bh_wrist_inv(bh->wbh, q4, pos_pb_mm) < 0) {
		return -1;
	}

	double delta_ballscrew;

	if (bh_elbow_inv(bh->ebh, bh->L_rod, pos_pb_mm, tf_rot_center, &delta_ballscrew) < 0) {
		return -1;
	}

	*out = delta_ballscrew;

	return 0;
}