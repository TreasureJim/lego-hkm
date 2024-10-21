#include <stddef.h>
#include "kinematics.h"

struct agile_pkm_model lego_model = {
	.bh = NULL,
	.L0 = 0.10234, // height from floor to bottom part of elbow 
	// .L1 = 0.18, // length from servo 1 to end of elbow 
	.L1 = 0.20,
	.L2 = 0.136 + 0.015, // lenght of elbow + D2
	.L3 = 0.03, //
	.L3b = 0.033, //
	.Lpar = 0.143, //
	.a2 = 0.01, // tool length
	.d2 = 0.015, //
	.ax2x = 0.018, 
	.ax2y = 0.020,
	.ax2z = 0.0315 - 0.0335, // 
	.ax3y = -0.065, // length from servo 1 to elbow rotation axis on x axis
	.q2_offs = -0.07398341092526937,
	.joint_lims = {{-1.56, 1.56}, {-0.61, 0.872}, {-0.785, 0.959}, {-3.141592653589793, 3.141592653589793}},
};
