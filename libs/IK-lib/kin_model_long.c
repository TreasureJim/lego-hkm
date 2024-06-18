/* Generated for ConceptPrototype_long */
#include "kinematics.h"

struct agile_pkm_elbow_backhoe elbow_bh = {
        .ax = 717.84,
        .ay = 8.0,
        .ah = 717.8845767949051,
        .a0 = 0.011144083392314397,
        .bs0 = 740.96,
        .La1 = 100.0,
        .La2 = 140.0,
        .a_ang = 0.17453292519943295,
        .d_ang = 0.2857103985514718,
        .Ld = 70.0,
        .dx = 67.1623173671606,
        .dy = 19.728738577841185,
        .dxb = 5.0,
        .Lc = 131.55696691002717,
        .Lb1 = 70.0,
        .Lb2 = 120.0,
        .z_offs = 12.0,
};

struct agile_pkm_wrist_backhoe wrist_bh = {
        .La1 = 120.0,
        .La2 = 18.0,
        .Lc1 = 120.0,
        .Lc2 = 18.0,
        .Lc3 = 72.0,
        .c_ang = 0.29670597283903605,
        .Ld = 68.85,
        .Le1 = 36.0,
        .Le2 = 36.0,
        .e_ang = 0.29670597283903605,
        .Lf = 34.43,
        .Lg = 18.0,
        .Lh = 36.0,
        .q4_offs = 0.0,
        .alpha_c_pol = {0.3620537994403131, -0.4180692628587669},
        .alpha_e_pol = {0.6378304839063976, 0.41862898345499616},
        .alpha_c_max = {2.0, 1.43},
        .alpha_c_min = -1.353,
        .alpha_e_min = {-1.6, -1.212},
        .alpha_e_max = 4.345,
        .q4_angle_lims = {-2.5656340004316642, 5.759586531581287},
};

struct agile_pkm_backhoe backhoe = {
        .L_rod = 960.0,
        .ebh = &elbow_bh,
        .wbh = &wrist_bh,
};

struct agile_pkm_model kin_model_long = {
        .bh = &backhoe,
        .L1 = 0,
        .L1 = 0.96,
        .L2 = 0.96,
        .L3 = 0.15,
        .L3b = 0.15,
        .Lpar = 0.8595,
        .a2 = 0.065,
        .d2 = 0.055,
        .ax2x = 0.122352,
        .ax2y = 0.057053,
        .ax2z = -0.03358,
        .ax3y = -0.095,
        .q2_offs = 0.05111432543995331,
        .joint_lims = {{-3.141592653589793, 3.141592653589793}, {-1.5196820013549432, 0.7492460262376852}, {-0.5235987755982988, 0.5235987755982988}, {-1.53, 4.74}},
};

