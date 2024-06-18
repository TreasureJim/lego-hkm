#ifndef CALIBRATED_KINEMATICS_H
#define CALIBRATED_KINEMATICS_H

#include "c_lpoe/lpoe.h"
#include "ax4_kinematics.h"

struct model_hkm {
    struct model_lpoe frame4_0;
    struct model_lpoe frame4_6;
    struct model_lpoe frame9_7;
    struct model_lpoe frame9_8;
    struct agile_pkm_model* nominal_model;
	double ax3_stiffness;
	double ax3_offs;
};

int inv_calib_full(const struct model_hkm* rob, const struct model_lpoe_agile_pkm* rob_lpoe,
	const struct dyn_params_link dpar[], const struct dyn_params_link* tool,
 	double pos[3], double orient, double q_out[4], double tol, int max_iter);

int inv_calib(const struct model_hkm* rob, double pos[3], double orient,
	double q_out[4], double tol, int max_iter);

int fwd_calib_full(const struct model_hkm* rob, const struct model_lpoe_agile_pkm* rob_lpoe,
	const struct dyn_params_link dpar[], const struct dyn_params_link* tool,
	const double q_in[4], double out[4][4], double* orient, double elbow[4][4], double tol, int max_iter);

int fwd_calib(const struct model_hkm* rob, const double q[4], double out[4][4],
	double* orient, double elbow[4][4], double tol, int max_iter);

#endif
