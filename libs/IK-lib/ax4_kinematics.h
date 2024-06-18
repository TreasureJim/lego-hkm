#ifndef AX4_KINEMATICS_H
#define AX4_KINEMATICS_H

struct agile_pkm_elbow_backhoe {
	double ax;
	double ay;
	double ah;
	double a0;
	double bs0;
	double La1;
	double La2;
	double a_ang;
	double d_ang;
	double Ld;
	double dx;
	double dy;
	double dxb;
	double Lc;
	double Lb1;
	double Lb2;
	double z_offs;
};

struct agile_pkm_wrist_backhoe {
	double La1;
	double La2;
	double Lc1;
	double Lc2;
	double Lc3;
	double c_ang;
	double Ld;
	double Le1;
	double Le2;
	double e_ang;
	double Lf;
	double Lg;
	double Lh;
	double q4_offs;
	double alpha_c_pol[2];
	double alpha_e_pol[2];
	double alpha_c_max[2];
	double alpha_c_min;
	double alpha_e_min[2];
	double alpha_e_max;
	double q4_angle_lims[2];
};

struct agile_pkm_backhoe {
	double L_rod;
	struct agile_pkm_elbow_backhoe *ebh;
	struct agile_pkm_wrist_backhoe *wbh;
};

typedef struct {
#ifdef EXTRA_CALC
	double r1;
	double r2;
	double alpha_c;
	double alpha_d;
	double alpha_ec_diff;
	double alpha_f;
#endif
	double q4;
} bh_wrist_fwd_ret_t;

typedef struct {
#ifdef EXTRA_CALC
	double bs_ang;
	double a_extra;
	double c_ang;
	double theta_b;

	double r1;
	double r2;
	double alpha_c;
	double alpha_d;
	double alpha_ec_diff;
	double alpha_f;
#endif
	double q4;
} ax4_fwd_ret_t;

int ax4_fwd(const struct agile_pkm_backhoe *bh, double delta_ballscrew, double tf_rot_center[4][4], ax4_fwd_ret_t *out);

int ax4_inv(const struct agile_pkm_backhoe *bh, double q4, double tf_rot_center[4][4], double *out);

#endif