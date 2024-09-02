#include <stddef.h>
#include "kinematics.h"

struct agile_pkm_model lego_model = {
	.bh = NULL,
	.L0 = 0.64275, // height from floor to bottom part of elbow // UNFINISHED
	.L1 = 0.945, // length from servo 1 to end of elbow // UNFINISHED
	.L2 = 0.136 + 0.015, // lenght of elbow + D2
	.L3 = 0.03, //
	.L3b = 0.033, //
	.Lpar = 0.143, //
	.a2 = 0.01, // tool length
	.d2 = 0.015, //
	.ax2x = 0.1, // UNFINISHED
	.ax2y = 0.1, // UNFINISHED
	.ax2z = 0.0315 - 0.0335, // 
	.ax3y = -0.065, // length from servo 1 to elbow rotation axis on x axis
	.q2_offs = -0.07398341092526937,
	.joint_lims = {{-3.6651914291880923, 2.6179938779914944}, {-1.4178869349609031, 0.885947677671612}, {-1.2217304763960306, 1.2217304763960306}, {-3.141592653589793, 3.141592653589793}},
};
