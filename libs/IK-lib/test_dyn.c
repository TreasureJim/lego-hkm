/*
 * Copyright (C) 2020 Cognibotics
 *
 * Use or copying of this file or any part of it, without explicit
 * permission from Cognibotics is not allowed in any way.
 */
#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <time.h>

#include "dyn.h"
#include "concept_prototype.h"
#include "kinematics.h"
#include "kin_model.h"


static void test_kin_lpoe(void)
{
        double tfs[9][4][4];
        double q[9] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7 - M_PI_2, 0.8, 0.9};

        double tfs_true[9][4][4] = {
                {{0.9950041652780258, -0.09983341664682815, 0.0, 0.0},
                 {0.09983341664682815, 0.9950041652780258, 0.0, 0.0},
                 {0.0, 0.0, 1.0, 0.0},
                 {0.0, 0.0, 0.0, 1.0}},
                {{1.0, 0.0, 0.0, 0.64},
                 {0.0, 0.9800665778412416, -0.19866933079506122, -0.095},
                 {0.0, 0.19866933079506122, 0.9800665778412416, 0.0},
                 {0.0, 0.0, 0.0, 1.0}},
                {{0.955336489125606, -0.29552020666133955, 0.0, 0.0},
                 {0.29552020666133955, 0.955336489125606, 0.0, 0.0},
                 {0.0, 0.0, 1.0, 0.0},
                 {0.0, 0.0, 0.0, 1.0}},
                {{0.9210609940028851, 0.0, 0.3894183423086505, 0.0},
                 {0.0, 1.0, 0.0, 0.585},
                 {-0.3894183423086505, 0.0, 0.9210609940028851, 0.0},
                 {0.0, 0.0, 0.0, 1.0}},
                {{1.0, 0.0, 0.0, 0.0},
                 {0.0, 0.8775825618903728, -0.479425538604203, 0.0},
                 {0.0, 0.479425538604203, 0.8775825618903728, 0.0},
                 {0.0, 0.0, 0.0, 1.0}},
                {{0.8253356149096783, -0.5646424733950354, 0.0, 0.0},
                 {0.5646424733950354, 0.8253356149096783, 0.0, 0.055},
                 {0.0, 0.0, 1.0, 0.0},
                 {0.0, 0.0, 0.0, 1.0}},
                {{0.7648421872844885, -0.644217687237691, 0.0, 0.122352},
                 {0.644217687237691, 0.7648421872844885, 0.0, 0.057053},
                 {0.0, 0.0, 1.0, -0.03358},
                 {0.0, 0.0, 0.0, 1.0}},
                {{0.6967067093471654, -0.7173560908995228, 0.0, 0.15},
                 {0.7173560908995228, 0.6967067093471654, 0.0, 0.0},
                 {0.0, 0.0, 1.0, 0.0},
                 {0.0, 0.0, 0.0, 1.0}},
                {{0.6216099682706644, 0.0, 0.7833269096274834, 0.0},
                 {0.0, 1.0, 0.0, 0.0},
                 {-0.7833269096274834, 0.0, 0.6216099682706644, 0.0},
                 {0.0, 0.0, 0.0, 1.0}}};

        kin(&concept_prototype, q, tfs);

        printf("\nTest of fwd kin for dyn:\n");
        for(int i=0; i<9; i++){
                printf("\ni: %d\n", i);
                for(int i1=0; i1<4; i1++){
                        for(int i2=0; i2<4; i2++){
                                printf("%f ", tfs[i][i1][i2]);
                                assert(fabs(tfs[i][i1][i2] - tfs_true[i][i1][i2]) < 1e-10);
                        }
                        printf("\n");
                }
        }
}


static void test_transform_a2b(void)
{
        double tf[4][4] = {{0.0, -1.0, 0.0, 0.0},
                           {0.0, 0.0, -1.0, 0.0},
                           {1.0, 0.0, 0.0, 0.0},
                           {0.0, 0.0, 0.0, 1.0}};
        double c_a[3] = {1.0, 2.0, 3.0};
        double c_b[3];
        double c_b_true[3] = {3.0, -1.0, -2.0};

        transform_a2b(tf, c_a, c_b);

        printf("Test of transform_a2b:\n");
        printf("c_b: %f %f %f\n", c_b[0], c_b[1], c_b[2]);
        for (int i1 = 0; i1 < 3; i1++) {
                assert(fabs(c_b[i1]-c_b_true[i1]) < 1e-10);
        }
}


static void test_transform_b2a(void)
{
        double tf[4][4] = {{0.0, -1.0, 0.0, 0.0},
                           {0.0, 0.0, -1.0, 0.0},
                           {1.0, 0.0, 0.0, 0.0},
                           {0.0, 0.0, 0.0, 1.0}};
        double c_b[3] = {3.0, -1.0, -2.0};
        double c_a[3];
        double c_a_true[3] = {1.0, 2.0, 3.0};

        transform_b2a(tf, c_b, c_a);

        printf("Test of transform_b2a:\n");
        printf("c_a: %f %f %f\n", c_a[0], c_a[1], c_a[2]);
        for (int i1 = 0; i1 < 3; i1++) {
                assert(fabs(c_a[i1]-c_a_true[i1]) < 1e-10);
        }
}


static void test_dyn_inv_agile_extended(void)
{
        struct dyn_params_link tool = {
                .m = 4,
                .r = {0, 0, 0},
                .ival = {{0, 0, 0},
                         {0, 0, 0},
                         {0, 0, 0}},
        };

        double q[9] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7 - M_PI_2, 0.8, 0.9};
        double qd[9] = {0.1, -0.1, 0.1, -0.1, 0.1, -0.1, 0.1, -0.1, 0.1};
        double qdd[9] = {-0.1, 0.1, -0.1, 0.1, -0.1, 0.1, -0.1, 0.1, -0.1};
        double g[3] = {0.0, 0.0, -9.82};

        int ret;
        double trq[9];

        double trq_true[9] = {-0.9557381868317143, 40.16937182245898,
                              -2.9079122058349904, 0.2640834467183138,
                              3.728959565024346, -0.00019450770920200673,
                              -0.14767964459069263, -0.001546336876867941,
                              -0.3528266174643957};

        ret = dyn_inv_agile_extended(&concept_prototype, concept_prototype_dynpars, &tool, q, qd, qdd, g, trq);

        printf("\nTest of dyn_inv_agile_extended\n");
        printf("ret: %d\n", ret);
        for(int k=0; k<9; k++){
                printf("trq[%d]: %f\n", k, trq[k]);
                assert(fabs(trq[k]-trq_true[k]) < 1e-10);
        }
}


static void test_kin_extra_angles(void)
{
        double q[4] = {0.1, 1.5 - M_PI_2, -0.3, 45};

        double out_tcp[4][4];
        double out_elbow[4][4];
        double orient_angle;
        int ret;

        double out_tcp_true[4][4] = {
                        {-0.7431733344159279, 0.6690989426184379,
                         2.215600010143668e-19, 0.6260594139439498},
                        {-0.6690989426184378, -0.7431733344159277,
                         1.7857932934111646e-17, 0.5830161313013404},
                        {1.2273617469597505e-17, 1.743768420189692e-17,
                         1.0, -0.23752468721765566},
                        {0.0, 0.0, 0.0, 1.0}};
        double ori_angle_true = -2.408596974700364 + 2 * M_PI;
        struct {
	        double alpha1;
	        double alpha2;
	        double q23;
	        double qy;
	        double qx;
	        double q4_angle;
        } q_extra_true = {-1.7979763208697221, 0.019716131465607502,
                			-0.06406306766540304, 0.019800888552092503,
					        0.29936550945132806, 3.841637682188686};;

        res_q_extra_t res_q_extra;
        ax4_fwd_ret_t ax4_fwd_ret;
        // ret = fwd_nom(&kin_model, q, out_tcp, &orient_angle, out_elbow, &res_q_extra, &ax4_fwd_ret);
        double th[4];
        ret = drive_to_joint_full(&kin_model, q, th, &res_q_extra, &ax4_fwd_ret);
        assert(ret == 0);
        ret = fwd(&kin_model, th, out_tcp, &orient_angle);
        assert(ret == 0);

        printf("\n\nTest of fwd:\n");
        printf("ret: %d\n", ret);
        printf("out_tcp:\n");
        for (int i1 = 0; i1 < 4; i1++) {
                for (int i2 = 0; i2 < 4; i2++) {
                        printf("%f ", out_tcp[i1][i2]);
                        // assert(fabs(out_tcp[i1][i2] - out_tcp_true[i1][i2]) < 1e-10);
                }
                printf("\n");
        }
        printf("ori_angle: %f\n", orient_angle);
        // assert(fabs(orient_angle - ori_angle_true) < 1e-10);

        assert(fabs(res_q_extra.alpha1 - q_extra_true.alpha1) < 1e-10);
        assert(fabs(res_q_extra.alpha2 - q_extra_true.alpha2) < 1e-10);
        assert(fabs(res_q_extra.q23 - q_extra_true.q23) < 1e-10);
        assert(fabs(res_q_extra.qy - q_extra_true.qy) < 1e-10);
        assert(fabs(res_q_extra.qx - q_extra_true.qx) < 1e-10);
        // TODO: fix ax4_fwd?
        // assert(fabs(res_q_extra.q4_angle - q_extra_true.q4_angle) < 1e-10); 
}


static void test_calc_gmat(void)
{
        int ret;
        double q[4] = {0.1, 1.5 - M_PI_2, -0.3, 45};
        double qd[4] = {0.5, -0.5, 0.5, 30};
        double gmat[9][4];
        double gvec[9];
        double qe[9];

        double gmat1_py[9][4] = {{1.000000000001, 0.0, 0.0, 0.0},
                                 {0.0, 0.0, 0.9999999999732445, 0.0},
                                 {0.0, 1.0028316701493623,
                                  -0.10846719789014969, 0.0},
                                 {0.0, -0.30945448892608085,
                                  -0.03664690684246352, 0.0},
                                 {0.0, 0.01985550546201864,
                                  -0.9999006357341855, 0.0},
                                 {1.199040866595169e-08, -2.727197191187969,
                                  -1.1920234750384395, 0.11213970552148567},
                                 {0.0, 0.9999999999177334, 0.0, 0.0},
                                 {0.0, -1.0027622405761605,
                                  0.08530424167041417, 0.0},
                                 {0.0, 0.005265658464814749,
                                  -0.2651726027612378, 0.0}};

        double gmat2_py[9][4] = {{1.0000000000000286, 0.0, 0.0, 0.0},
                                 {0.0, 0.0, 0.9999999999998899, 0.0},
                                 {0.0, 1.0028316774146617,
                                  -0.10846748325299416, 0.0},
                                 {0.0, -0.30945447869929615,
                                  -0.03664671856442453, 0.0},
                                 {0.0, 0.0198556607106104,
                                  -0.9999006465902238, 0.0},
                                 {0.0, -2.727195942529015,
                                  -1.1920246604280038, 0.11213970688039865},
                                 {0.0, 0.9999999999998899, 0.0, 0.0},
                                 {0.0, -1.0027622464892083,
                                  0.08530438916021232, 0.0},
                                 {0.0, 0.00526569965636145,
                                  -0.26517256499236985, 0.0}};
        double gvec2_py[9] = {0.0, 0.0, 0.1591071718820558,
                              -0.6524151678968804, -0.05351624282612377,
                              -9.511565823849821, 0.0,
                              -0.08871304246760303, -0.03511036964637393};

        printf("\nTest calc_gmat 1:\n");

        ret = calc_gmat(&kin_model, q, 1e-6, 0, 0, qd, qe, gmat, gvec);
        printf("ret: %d\n", ret);
        assert(ret == 0);
        printf("gmat:\n");
        for(int i=0; i<9; i++){
                for(int j=0; j<4; j++){
                        printf("%f ", gmat[i][j]);
                        assert(fabs(gmat[i][j] - gmat1_py[i][j]) < 1e-7);
                }
                printf("\n");
        }

        printf("\nTest calc_gmat 2:\n");

        ret = calc_gmat(&kin_model, q, 1e-4, 0, 1, qd, qe, gmat, gvec);
        printf("ret: %d\n", ret);
        assert(ret == 0);
        printf("gmat:\n");
        for(int i=0; i<9; i++){
                for(int j=0; j<4; j++){
                        printf("%f ", gmat[i][j]);
                        assert(fabs(gmat[i][j] - gmat2_py[i][j]) < 1e-7);
                }
                printf("\n");
        }
        printf("gvec:\n");
        for(int i=0; i<9; i++){
                printf("%f (%f)\n", gvec[i], gvec2_py[i]);
                if( i == 5 ){
                        assert(fabs(gvec[i]-gvec2_py[i]) < 1e-3);
                } else {
                        assert(fabs(gvec[i]-gvec2_py[i]) < 1e-7);
                }
        }
}


static void test_dyn_inv_agile(void)
{
        struct dyn_params_link tool = {
                .m = 4,
                .r = {0, 0, 0},
                .ival = {{0, 0, 0},
                         {0, 0, 0},
                         {0, 0, 0}},
        };

        double q[4] = {0.1, 1.5 - M_PI_2, -0.3, 45};
        double qd[4] = {0.5, -0.5, 0.5, 30};
        double qdd[4] = {2.3, 4.5, 6.23, -301.4};
        double g[3] = {0.0, 0.0, -9.82};

        int ret;
        double trq[4];

        double trq_py1[4] = {35.6401948090838, 20.91424772419974,
                             55.860807368034614, -0.025566575139799288};
        double trq_py2[4] = {35.888133751012475, 21.341901388795453,
                             55.86761531165337, -0.030545858829055153};

        printf("\nTest of dyn_inv_agile:\n");

        printf("\nWithout all coriolis/centrifugal forces terms:\n");
        ret = dyn_inv_agile(&kin_model, &concept_prototype, concept_prototype_dynpars,
                        &tool, q, qd, qdd, g, 0, trq);


        printf("ret: %d\n", ret);
        printf("trq: %f %f %f %f\n", trq[0], trq[1], trq[2], trq[3]);

        for(int i=0; i<4; i++){
                assert(fabs(trq[i]-trq_py1[i]) < 1e-7);
        }

        printf("\nIncluding all coriolis/centrifugal forces terms:\n");
        ret = dyn_inv_agile(&kin_model, &concept_prototype, concept_prototype_dynpars,
                        &tool, q, qd, qdd, g, 1, trq);

        printf("ret: %d\n", ret);
        printf("trq: %f %f %f %f\n", trq[0], trq[1], trq[2], trq[3]);
        for(int i=0; i<4; i++){
                assert(fabs(trq[i]-trq_py2[i]) < 1e-5);
        }
}


static void test_fric(void)
{
        double qd1[4] = {0.0, 0.0, 0.0, 0.0};
        double qd2[4] = {1.0, 1.0, 1.0, 1.0};

        double trq[4];

        printf("\n\nTest of fric trq calculation:\n");

        printf("\nZero velocity:\n");
        fric_trq_agile(concept_prototype_fricpars, qd1, trq);
        printf("trq: %f %f %f %f\n", trq[0], trq[1], trq[2], trq[3]);

        printf("\nPositive velocity for all joints:\n");
        fric_trq_agile(concept_prototype_fricpars, qd2, trq);
        printf("trq: %f %f %f %f\n", trq[0], trq[1], trq[2], trq[3]);
}


static void test_inertia_matrix(void)
{
        struct dyn_params_link tool = {
                .m = 4,
                .r = {0, 0, 0},
                .ival = {{0, 0, 0},
                         {0, 0, 0},
                         {0, 0, 0}},
        };

        double q1[4] = {0.1, 1.5 - M_PI_2, -0.3, 45};
        double q12[4] = {0.5, 1.5 - M_PI_2, -0.3, 45}; /* inertia should be equal for q1 and q12 */
        double q2[4] = {0.5, 0.3 - M_PI_2, 0.1, 30}; /* inertia for j1 should be larger here than for q1 */

        int ret;
        double imat1[4][4];
        double imat12[4][4];
        double imat2[4][4];

        printf("\nTest of inertia_matrix:\n");

        printf("\nTest pos 1:\n");
        ret = inertia_matrix(&kin_model, &concept_prototype, concept_prototype_dynpars,
                        &tool, q1, imat1);

        printf("ret: %d\n", ret);
        printf("imat:\n");

        for(int i=0; i<4; i++){
                for(int j=0; j<4; j++){
                        printf("%f ", imat1[i][j]);
                }
                printf("\n");
        }

        printf("\nTest pos 2:\n");
        ret = inertia_matrix(&kin_model, &concept_prototype, concept_prototype_dynpars,
                        &tool, q12, imat12);

        printf("ret: %d\n", ret);
        printf("imat:\n");

        for(int i=0; i<4; i++){
                for(int j=0; j<4; j++){
                        printf("%f ", imat12[i][j]);
                        assert(fabs(imat1[i][j]-imat12[i][j]) < 1e-5);
                }
                printf("\n");
        }

        printf("\nTest pos 3:\n");
        ret = inertia_matrix(&kin_model, &concept_prototype, concept_prototype_dynpars,
                        &tool, q2, imat2);

        printf("ret: %d\n", ret);
        printf("imat:\n");

        for(int i=0; i<4; i++){
                for(int j=0; j<4; j++){
                        printf("%f ", imat2[i][j]);
                }
                printf("\n");
        }
        assert(imat2[0][0] > imat1[0][0]);
}


int main(void)
{
        test_kin_lpoe();
        test_transform_a2b();
        test_transform_b2a();
        test_dyn_inv_agile_extended();
        test_kin_extra_angles();
        // test_calc_gmat();
        // test_dyn_inv_agile();
        test_fric();
        test_inertia_matrix();
}
