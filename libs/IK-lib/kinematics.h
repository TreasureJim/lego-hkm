#ifndef C_KINEMATICS_H
#define C_KINEMATICS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "c_lpoe/lpoe.h"
#include "ax4_kinematics.h"
#define DEFAULT_TOL 1e-6
#define DEFAULT_MAX_ITER 20
#define DELTA1 1e-5
#define DELTA2 1e-4

struct agile_pkm_model {
	struct agile_pkm_backhoe *bh;
	double L0;
	double L1;
	double L2;
	double L3;
	double L3b;
	double Lpar;
	double a2;
	double d2;
	double ax2x;
	double ax2y;
	double ax2z;
	double ax3y;
	double q2_offs;
	double joint_lims[4][2];
};

typedef struct {
	double q23;
	double alpha1;
	double alpha2;
	double qy;
	double qx;
#ifdef EXTRA_CALC
	double beta1;
	double beta2;
	double rod_twist;
	double rod_rot;
	double rod_tilt;
	double rod_rot_z;
	double q4_angle;
	// tcp_pos_ and out2 are currently not calculated, in order to
	// efficiently implement all calculations in only drive_to_joint.
	// If needed, can be added to fwd().
	// double tcp_pos_[3];
	// double out2[3];
	double sign;
#endif
} res_q_extra_t;

int inv(const struct agile_pkm_model *rob, const double pos[3], const double orient, double q[4]);

int inv_posonly(const struct agile_pkm_model *rob, const double pos[3], const double q4, const double tool[3], double q[4]);

int fwd(const struct agile_pkm_model *rob, const double q[4], double tcp_out[4][4], double *orient_angle);

int fwd_elbow(const struct agile_pkm_model *rob, const double q1, double elbow_out[4][4]);

int joint_to_drive(const struct agile_pkm_model *rob, const double joints[4], double drives[4]);

int drive_to_joint(const struct agile_pkm_model *rob, const double drives[4], double joints[4]);

#ifdef EXTRA_CALC
int drive_to_joint_full(const struct agile_pkm_model *rob, const double drives[4], double joints[4], res_q_extra_t *res_q_extra, ax4_fwd_ret_t *q4_angle);
#else
int drive_to_joint_full(const struct agile_pkm_model *rob, const double drives[4], double joints[4], res_q_extra_t *res_q_extra);
#endif

int cart_to_drive(const struct agile_pkm_model *rob, const double pos[3], const double orient_angle, double q[4]);

int cart_to_drive_posonly(const struct agile_pkm_model *rob, const double pos[3], const double q4, const double tool[3], double q[4]);

int drive_to_cart(const struct agile_pkm_model *rob, const double q[4], double tcp_out[4][4], double *orient_angle);

int inv_with_vel_acc(const struct agile_pkm_model *rob, const double pos[3], const double orient_angle, const double vel[4], const double acc[4], double q[4], double qd[4], double qdd[4]);

int inv_posonly_with_vel_acc(const struct agile_pkm_model *rob, const double pos[3], const double vel[3], const double acc[3], const double q4, const double q4d, const double q4dd, const double tool[3], double q[4], double qd[4], double qdd[4]);

int fwd_with_vel_acc(const struct agile_pkm_model *rob, const double q[4], const double qd[4], const double qdd[4], double tcp_out[4][4], double *orient_angle, double vel[4], double acc[4]);

int fwd_elbow_with_vel_acc(const struct agile_pkm_model *rob, const double q1, const double qd1, const double qdd1, double elbow_out[4][4], double elbow_vel[4], double elbow_acc[4]);

int joint_to_drive_with_vel_acc(const struct agile_pkm_model *rob, const double joint_pos[4], const double joint_vel[4], const double joint_acc[4], double drive_pos[4], double drive_vel[4], double drive_acc[4]);

int drive_to_joint_with_vel_acc(const struct agile_pkm_model *rob, const double drive_pos[4], const double drive_vel[4], const double drive_acc[4], double joint_pos[4], double joint_vel[4], double joint_acc[4]);

int cart_to_drive_with_vel_acc(const struct agile_pkm_model *rob, const double pos[3], const double orient_angle, const double vel[4], const double acc[4], double q[4], double qd[4], double qdd[4]);

int drive_to_cart_with_vel_acc(const struct agile_pkm_model *rob, const double q[4], const double qd[4], const double qdd[4], double tcp_out[4][4], double *orient_angle, double vel[4], double acc[4]);

#ifdef EXTRA_CALC
int inv_33_special(const double a[4][4], double a_inv[4][4]);
int cart_to_drive_with_vel_acc_num(const struct agile_pkm_model *rob, const double pos[3], const double orient_angle, const double vel[4], const double acc[4], double q[4], double qd[4], double qdd[4]);
#endif

#ifdef __cplusplus
}
#endif

#endif /* C_KINEMATICS_H */
