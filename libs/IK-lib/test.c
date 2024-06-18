#include <stdio.h>
#include <math.h>
#ifndef _POSIX_C_SOURCE
#define M_PI		3.14159265358979323846	/* pi */
#define M_PI_2		1.57079632679489661923	/* pi/2 */
#define M_PI_4		0.78539816339744830962	/* pi/4 */
#endif
#include "kinematics.h"
#include "kin_model.h"

int main(void)
{
#if 0
	if (ax4_main() < 0)
	{
		printf("ax4_main FAILED\n");
		return -1;
	}
#endif
	size_t times;
	for (times = 0; times < 1; times++)
	{
		double q[] = {0.1, 1.5 - M_PI_2, -0.3, 45.0};
		/*double q[] = {0.0, M_PI_2, 0.0, 0.0}; */
		printf("Fwd for q: {%f, %f, %f, %f}\n", q[0], q[1], q[2], q[3]);

		double out_tcp[4][4];
		double orient_angle;
		if (drive_to_cart(&kin_model, q, out_tcp, &orient_angle) < 0)
		{
			printf("fwd_nom() failed\n");
			return -1;
		}

		/*
		printf("out_tcp:");
		print_matrix((double*)out_tcp, 4); */
		printf("pos: {%.8f, %.8f, %.8f}\n", out_tcp[0][3], out_tcp[1][3], out_tcp[2][3]);
		printf("ori: %.16f\n", orient_angle);

#ifdef EXTRA_CALC
		double joints[4];
		res_q_extra_t res_q_extra;
		ax4_fwd_ret_t ax4_fwd_ret;
		drive_to_joint_full(&kin_model, q, joints, &res_q_extra, &ax4_fwd_ret);
		printf("res_q_extra:\n");
		printf("q23: %f\n", res_q_extra.q23);
		printf("rod_twist: %f\n", res_q_extra.rod_twist);
		printf("qy: %f\n", res_q_extra.qy);
		printf("qx: %f\n", res_q_extra.qx);
		printf("rod_rot: %f\n", res_q_extra.rod_rot);
		printf("rod_tilt: %f\n", res_q_extra.rod_tilt);
		printf("rod_rot_z: %f\n", res_q_extra.rod_rot_z);
		printf("alpha1: %f\n", res_q_extra.alpha1);
		printf("alpha2: %f\n", res_q_extra.alpha2);
		printf("q4_angle: %f\n", res_q_extra.q4_angle);

		printf("ax4_fwd_ret:\n");
		printf("bs_ang: %f\n", ax4_fwd_ret.bs_ang);
		printf("a_extra: %f\n", ax4_fwd_ret.a_extra);
		printf("c_ang: %f\n", ax4_fwd_ret.c_ang);
		printf("theta_b: %f\n", ax4_fwd_ret.theta_b);
		printf("r1: %f\n", ax4_fwd_ret.r1);
		printf("r2: %f\n", ax4_fwd_ret.r2);
		printf("alpha_c: %f\n", ax4_fwd_ret.alpha_c);
		printf("alpha_d: %f\n", ax4_fwd_ret.alpha_d);
		printf("alpha_ec_diff: %f\n", ax4_fwd_ret.alpha_ec_diff);
		printf("alpha_f: %f\n", ax4_fwd_ret.alpha_f);
		printf("q4: %f\n", ax4_fwd_ret.q4);
#endif

		double inv_in[] = {out_tcp[0][3], out_tcp[1][3], out_tcp[2][3]};
		double q_out[4];

		if (cart_to_drive(&kin_model, inv_in, orient_angle, q_out) < 0)
		{
			printf("inv_nom() failed\n");
			return -1;
		}

		printf("Inv:\n");
		printf("q_out[0]: {%f, %f, %f, %f}\n", q_out[0], q_out[1], q_out[2], q_out[3]);
		/*
		printf("q_out[1]: {%f, %f, %f, %f}\n", q_out[1][0], q_out[1][1], q_out[1][2], q_out[0][3]);
		*/
	}

	return 0;
}
