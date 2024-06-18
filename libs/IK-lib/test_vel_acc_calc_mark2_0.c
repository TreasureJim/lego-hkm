#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <time.h>
#include "kinematics.h"
#include "mark2_0.h"

int main(void)
{
	int ret;
	double q[4] = {1.0, 0.4, 0.2, 2.0};
	double out_tcp[4][4];
	double orient_angle;
	ret = drive_to_cart(&mark2_0, q, out_tcp, &orient_angle);
	assert(ret == 0);

	double pos[3] = {out_tcp[0][3], out_tcp[1][3], out_tcp[2][3]};
	double vel[4] = {0.5, 0.5, 0.2, 23.0};
	double acc[4] = {-1.0, 1.0, -1.0, 1.0};
	double q_inv[4];
	double qd_inv[4];
	double qdd_inv[4];
	double qd_inv_py[4] = {-0.46477974938774946, -0.3549797593184458, 0.21452373169386776, 23.81155444100656};
	double qdd_inv_py[4] = {1.8327610489602044, -1.780064510296686, -1.446457543677809, 1.2718796044705813};

	double joint_pos[4];
	double joint_vel[4];
	double joint_acc[4];
	double q_inv_py2[4] = {1.0, 0.4282388182096672, 0.1823564146560869, 2.0};
	double qd_inv_py2[4] = {-0.4647797493877503, -0.3467746916188115, 0.22472054542766015, 23.81155444100656};
	double qdd_inv_py2[4] = {1.8327610489602064, -2.1046406534307875, -1.1142903983574057, 1.271879604470581};

	printf("\nTest inv_with_vel_acc:\n");
	ret = inv_with_vel_acc(&mark2_0, pos, orient_angle, vel, acc, joint_pos, joint_vel, joint_acc);
	printf("ret: %d\n", ret);
	printf("q_inv: %f %f %f %f\n", joint_pos[0], joint_pos[1], joint_pos[2], joint_pos[3]);
	printf("qd_inv: %f %f %f %f\n", joint_vel[0], joint_vel[1], joint_vel[2], joint_vel[3]);
	printf("qdd_inv: %f %f %f %f\n", joint_acc[0], joint_acc[1], joint_acc[2], joint_acc[3]);
	for (int i = 0; i < 4; i++)
		assert(fabs(joint_pos[i] - q_inv_py2[i]) < 1e-10);
	for (int i = 0; i < 4; i++)
		assert(fabs(joint_vel[i] - qd_inv_py2[i]) < 1e-10);
	for (int i = 0; i < 4; i++)
		assert(fabs(joint_acc[i] - qdd_inv_py2[i]) < 1e-10);

	double tcp_fwd[4][4];
	double orient_angle_fwd;
	double vel_fwd[4];
	double acc_fwd[4];
	printf("\nTest fwd_with_vel_acc:\n");
	ret = fwd_with_vel_acc(&mark2_0, joint_pos, joint_vel, joint_acc, tcp_fwd, &orient_angle_fwd, vel_fwd, acc_fwd);
	printf("ret: %d\n", ret);
	printf("pos: %f %f %f %f\n", tcp_fwd[0][3], tcp_fwd[1][3], tcp_fwd[2][3], orient_angle_fwd);
	printf("vel: %f %f %f %f\n", vel_fwd[0], vel_fwd[1], vel_fwd[2], vel_fwd[3]);
	printf("acc: %f %f %f %f\n", acc_fwd[0], acc_fwd[1], acc_fwd[2], acc_fwd[3]);
	for (int i = 0; i < 3; i++)
		assert(fabs(tcp_fwd[i][3] - pos[i]) < 1e-10);
	assert(fabs(orient_angle_fwd - orient_angle) < 1e-10);
	for (int i = 0; i < 4; i++)
		assert(fabs(vel_fwd[i] - vel[i]) < 1e-10);
	for (int i = 0; i < 4; i++)
		assert(fabs(acc_fwd[i] - acc[i]) < 1e-10);

	printf("\nTest joint_to_drive_with_vel_acc:\n");
	ret = joint_to_drive_with_vel_acc(&mark2_0, joint_pos, joint_vel, joint_acc, q_inv, qd_inv, qdd_inv);
	printf("ret: %d\n", ret);
	printf("q_inv: %f %f %f %f\n", q_inv[0], q_inv[1], q_inv[2], q_inv[3]);
	printf("qd_inv: %f %f %f %f\n", qd_inv[0], qd_inv[1], qd_inv[2], qd_inv[3]);
	printf("qdd_inv: %f %f %f %f\n", qdd_inv[0], qdd_inv[1], qdd_inv[2], qdd_inv[3]);
	for (int i = 0; i < 4; i++)
		assert(fabs(q_inv[i] - q[i]) < 1e-10);
	for (int i = 0; i < 4; i++)
		assert(fabs(qd_inv[i] - qd_inv_py[i]) < 1e-10);
	for (int i = 0; i < 4; i++)
		assert(fabs(qdd_inv[i] - qdd_inv_py[i]) < 1e-10);

	printf("\nTest drive_to_joint_with_vel_acc:\n");
	ret = drive_to_joint_with_vel_acc(&mark2_0, q_inv, qd_inv, qdd_inv, joint_pos, joint_vel, joint_acc);
	printf("ret: %d\n", ret);
	printf("q_inv: %f %f %f %f\n", joint_pos[0], joint_pos[1], joint_pos[2], joint_pos[3]);
	printf("qd_inv: %f %f %f %f\n", joint_vel[0], joint_vel[1], joint_vel[2], joint_vel[3]);
	printf("qdd_inv: %f %f %f %f\n", joint_acc[0], joint_acc[1], joint_acc[2], joint_acc[3]);
	for (int i = 0; i < 4; i++)
		assert(fabs(joint_pos[i] - q_inv_py2[i]) < 1e-10);
	for (int i = 0; i < 4; i++)
		assert(fabs(joint_vel[i] - qd_inv_py2[i]) < 1e-10);
	for (int i = 0; i < 4; i++)
		assert(fabs(joint_acc[i] - qdd_inv_py2[i]) < 1e-10);

	printf("\nTest drive_to_cart_with_vel_acc:\n");
	ret = drive_to_cart_with_vel_acc(&mark2_0, q_inv, qd_inv, qdd_inv, tcp_fwd, &orient_angle_fwd, vel_fwd, acc_fwd);
	printf("ret: %d\n", ret);
	printf("pos: %f %f %f %f\n", tcp_fwd[0][3], tcp_fwd[1][3], tcp_fwd[2][3], orient_angle_fwd);
	printf("vel: %f %f %f %f\n", vel_fwd[0], vel_fwd[1], vel_fwd[2], vel_fwd[3]);
	printf("acc: %f %f %f %f\n", acc_fwd[0], acc_fwd[1], acc_fwd[2], acc_fwd[3]);
	for (int i = 0; i < 3; i++)
		assert(fabs(tcp_fwd[i][3] - pos[i]) < 1e-10);
	assert(fabs(orient_angle_fwd - orient_angle) < 1e-10);
	for (int i = 0; i < 4; i++)
		assert(fabs(vel_fwd[i] - vel[i]) < 1e-10);
	for (int i = 0; i < 4; i++)
		assert(fabs(acc_fwd[i] - acc[i]) < 1e-10);

	printf("\nTest cart_to_drive_with_vel_acc:\n");
	ret = cart_to_drive_with_vel_acc(&mark2_0, pos, orient_angle, vel, acc, q_inv, qd_inv, qdd_inv);
	printf("ret: %d\n", ret);
	printf("q_inv: %f %f %f %f\n", q_inv[0], q_inv[1], q_inv[2], q_inv[3]);
	printf("qd_inv: %f %f %f %f\n", qd_inv[0], qd_inv[1], qd_inv[2], qd_inv[3]);
	printf("qdd_inv: %f %f %f %f\n", qdd_inv[0], qdd_inv[1], qdd_inv[2], qdd_inv[3]);
	for (int i = 0; i < 4; i++)
		assert(fabs(q_inv[i] - q[i]) < 1e-10);
	for (int i = 0; i < 4; i++)
		assert(fabs(qd_inv[i] - qd_inv_py[i]) < 1e-10);
	for (int i = 0; i < 4; i++)
		assert(fabs(qdd_inv[i] - qdd_inv_py[i]) < 1e-10);

	double tool[3] = {0.1, -0.2, 0.3};
	double q_inv_jl3[4] = {0.7664454554087272, 0.6733546385027749, -0.15071422040421031, 2.0};
	double qd_inv_jl3[4] = {-4.140489917477738, -2.3433065854811947, 0.22352837175077808, 23.811554441006557};
	double qdd_inv_jl3[4] = {115.28986157799874, -98.51909446007096, -1.1252298248498798, 1.2718796044705807};
	printf("\nTest inv_posonly_with_vel_acc:\n");
	ret = inv_posonly_with_vel_acc(&mark2_0, pos, vel, acc, q_inv_jl3[3], qd_inv_jl3[3], qdd_inv_jl3[3], tool, q_inv, qd_inv, qdd_inv);
	printf("ret: %d\n", ret);
	printf("q_inv: %f %f %f %f\n", q_inv[0], q_inv[1], q_inv[2], q_inv[3]);
	printf("qd_inv: %f %f %f %f\n", qd_inv[0], qd_inv[1], qd_inv[2], qd_inv[3]);
	printf("qdd_inv: %f %f %f %f\n", qdd_inv[0], qdd_inv[1], qdd_inv[2], qdd_inv[3]);
	for (int i = 0; i < 4; i++)
		assert(fabs(q_inv[i] - q_inv_jl3[i]) < 1e-10);
	for (int i = 0; i < 4; i++)
		assert(fabs(qd_inv[i] - qd_inv_jl3[i]) < 1e-10);
	for (int i = 0; i < 4; i++)
		assert(fabs(qdd_inv[i] - qdd_inv_jl3[i]) < 1e-10);

	double pos_elbow[3] = {0.5652812930579054, 0.7600704307620331, 0.64275};
	double vel_elbow[4] = {0.35326534432661716, -0.2627312977210367, 0.0, -0.4647797493877503};
	double acc_elbow[4] = {-1.5151396666781616, 0.8718349574188862, 0.0, 1.8327610489602064};
	printf("\nTest fwd_elbow_with_vel_acc:\n");
	ret = fwd_elbow_with_vel_acc(&mark2_0, joint_pos[0], joint_vel[0], joint_acc[0], tcp_fwd, vel_fwd, acc_fwd);
	printf("ret: %d\n", ret);
	printf("pos: %f %f %f\n", tcp_fwd[0][3], tcp_fwd[1][3], tcp_fwd[2][3]);
	printf("vel: %f %f %f %f\n", vel_fwd[0], vel_fwd[1], vel_fwd[2], vel_fwd[3]);
	printf("acc: %f %f %f %f\n", acc_fwd[0], acc_fwd[1], acc_fwd[2], acc_fwd[3]);
	for (int i = 0; i < 3; i++)
		assert(fabs(tcp_fwd[i][3] - pos_elbow[i]) < 1e-10);
	for (int i = 0; i < 4; i++)
		assert(fabs(vel_fwd[i] - vel_elbow[i]) < 1e-10);
	for (int i = 0; i < 4; i++)
		assert(fabs(acc_fwd[i] - acc_elbow[i]) < 1e-10);



}
