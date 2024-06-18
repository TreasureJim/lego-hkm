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
#include "kinematics.h"
#include "mark2_0.h"
#include "mark2_0_lpoe.h"


static void test_dyn_inv_mark2_0(void)
{
        struct dyn_params_link tool = {
                .m = 4,
                .r = {0, 0, 0},
                .ival = {{0, 0, 0},
                         {0, 0, 0},
                         {0, 0, 0}},
        };

        double q[4] = {0.1, 1.5 - M_PI_2, -0.3, 2.0};
        double qd[4] = {0.5, -0.5, 0.5, 30};
        double qdd[4] = {2.3, 4.5, 6.23, -301.4};
        double g[3] = {0.0, 0.0, -9.82};

        int ret;
        double trq[4];
        double trq_py1[4] = {120.88057708272515, 55.87141025438487,
                             123.77804035654573, -0.0072957026921831557};
        double trq_py2[4] = {120.55971616630421, 56.66828545813046,
                             123.29826845200742, -0.007717514394622961};

        printf("\nTest of dyn_inv for mark2_0:\n");

        printf("\nWithout all coriolis/centrifugal forces terms:\n");
        ret = dyn_inv_agile(&mark2_0, &mark2_0_lpoe, mark2_0_dynpars,
                        &tool, q, qd, qdd, g, 0, trq);

        if (ret)
        {
            printf("dyn_inv_agile() fail: %d\n", ret);
        }
        else
        {
            printf("trq: %f %f %f %f\n", trq[0], trq[1], trq[2], trq[3]);
            for(int i=0; i<4; i++){
                    assert(fabs(trq[i]-trq_py1[i]) < 1e-7);
            }
        }

        printf("\nIncluding all coriolis/centrifugal forces terms:\n");
        ret = dyn_inv_agile(&mark2_0, &mark2_0_lpoe, mark2_0_dynpars,
                        &tool, q, qd, qdd, g, 1, trq);

        if (ret)
        {
            printf("dyn_inv_agile() fail: %d\n", ret);
        }
        else
        {
            printf("trq: %f %f %f %f\n", trq[0], trq[1], trq[2], trq[3]);
            for(int i=0; i<4; i++){
                    assert(fabs(trq[i]-trq_py2[i]) < 1e-5);
            }
        }
}


static void test_dyn_inv_grav_mark2_0(void)
{
        struct dyn_params_link tool = {
                .m = 4,
                .r = {0, 0, 0},
                .ival = {{0, 0, 0},
                         {0, 0, 0},
                         {0, 0, 0}},
        };

        double q[4] = {0.1, 0.2, 0.3, 0.4};
        double qd[4] = {0, 0, 0, 0};
        double qdd[4] = {0, 0, 0, 0};
        double g[3] = {0.0, 0.0, -9.82};

        int ret;
        double trq[4];
        double trq_ref[4];

        dyn_inv_agile(&mark2_0, &mark2_0_lpoe, mark2_0_dynpars,
                &tool, q, qd, qdd, g, 0, trq_ref);

        printf("\nTest of dyn_inv_grav for mark2_0:\n");

        ret = dyn_inv_agile_grav(&mark2_0, &mark2_0_lpoe, mark2_0_dynpars,
                        &tool, q, g, trq);

        if (ret)
        {
            printf("dyn_inv_agile_grav() fail: %d\n", ret);
        }
        else
        {
            printf("trq: %f %f %f %f\n", trq[0], trq[1], trq[2], trq[3]);
            for(int i=0; i<4; i++){
                    assert(fabs(trq[i]-trq_ref[i]) < 1e-7);
            }
        }
}


static void test_dyn_inv_granular_mark2_0(void)
{
        struct dyn_params_link tool = {
                .m = 4,
                .r = {0, 0, 0},
                .ival = {{0, 0, 0},
                         {0, 0, 0},
                         {0, 0, 0}},
        };

        double q[4] = {0.1, 0.2, 0.3, 0.4};
        double qd[4] = {1.0, -1.0, 1.0, -1.0};
        double qdd[4] = {-10.0, 10.0, -10.0, 10.0};
        double g[3] = {0.0, 0.0, -9.82};

        int ret;
        double trq_grav[4];
        double trq_vel[4];
        double trq_acc[4];
        double trq_ref[4];

        dyn_inv_agile(&mark2_0, &mark2_0_lpoe, mark2_0_dynpars,
                &tool, q, qd, qdd, g, 1, trq_ref);

        printf("\nTest of dyn_inv_granular for mark2_0:\n");

        ret = dyn_inv_agile_granular(&mark2_0, &mark2_0_lpoe, mark2_0_dynpars,
                        &tool, q, qd, qdd, g, 1, trq_grav, trq_vel, trq_acc);

        if (ret)
        {
                printf("dyn_inv_agile_granular() fail: %d\n", ret);
        }
        else
        {
                printf("trq_ref: %f %f %f %f\n", trq_ref[0], trq_ref[1], trq_ref[2], trq_ref[3]);
                printf("trq_grav: %f %f %f %f\n", trq_grav[0], trq_grav[1], trq_grav[2], trq_grav[3]);
                printf("trq_vel: %f %f %f %f\n", trq_vel[0], trq_vel[1], trq_vel[2], trq_vel[3]);
                printf("trq_acc: %f %f %f %f\n", trq_acc[0], trq_acc[1], trq_acc[2], trq_acc[3]);
                printf("trq_sum: %f %f %f %f\n", trq_grav[0] + trq_vel[0] + trq_acc[0], trq_grav[1] + trq_vel[1] + trq_acc[1], trq_grav[2] + trq_vel[2] + trq_acc[2], trq_grav[3] + trq_vel[3] + trq_acc[3]);
                for(int i=0; i<4; i++){
                        assert(fabs(trq_grav[i]+trq_vel[i]+trq_acc[i]-trq_ref[i]) < 1e-12);
                }
        }
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

        double q1[4] = {0.1, 1.5 - M_PI_2, -0.3, 2.0};
        double q12[4] = {0.5, 1.5 - M_PI_2, -0.3, 2.0}; /* inertia should be equal for q1 and q12 */
        double q2[4] = {0.5, 0.3 - M_PI_2, 0.1, 2.0}; /* inertia for j1 should be larger here than for q1 */

        int ret;
        double imat1[4][4];
        double imat12[4][4];
        double imat2[4][4];

        printf("\nTest of inertia_matrix for mark2_0:\n");

        printf("\nTest pos 1:\n");
        ret = inertia_matrix(&mark2_0, &mark2_0_lpoe, mark2_0_dynpars,
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
        ret = inertia_matrix(&mark2_0, &mark2_0_lpoe, mark2_0_dynpars,
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
        ret = inertia_matrix(&mark2_0, &mark2_0_lpoe, mark2_0_dynpars,
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


static void test_inertia_matrix_new_joint_space(void)
{
        struct dyn_params_link tool = {
                .m = 4,
                .r = {0, 0, 0},
                .ival = {{0, 0, 0},
                         {0, 0, 0},
                         {0, 0, 0}},
        };

        double q1[4] = {0.1, -0.1, -0.3, 2.0};
        double q12[4] = {0.5, -0.1, -0.3, 2.0}; /* inertia should be equal for q1 and q12 */
        double q2[4] = {0.5, -1.0, 0.1, 2.0}; /* inertia for j1 should be larger here than for q1 */

        int ret;
        double imat1[4][4];
        double imat12[4][4];
        double imat2[4][4];

        printf("\nTest of inertia_matrix in joint space for mark2_0:\n");

        printf("\nTest pos 1:\n");
        ret = inertia_matrix_new_joint_space(&mark2_0, &mark2_0_lpoe, mark2_0_dynpars,
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
        ret = inertia_matrix_new_joint_space(&mark2_0, &mark2_0_lpoe, mark2_0_dynpars,
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
        ret = inertia_matrix_new_joint_space(&mark2_0, &mark2_0_lpoe, mark2_0_dynpars,
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

        double trq_true[9] = {-3.1558358315229622, 74.612462346399226, 
                                -7.8410753739357286, 6.3348289462071241, 
                                9.9781945414249211, 0.0086928795874459709, 
                                -1.3840767253002846, -1.1742324581442176, 
                                -19.571948539292006};

        ret = dyn_inv_agile_extended(&mark2_0_lpoe, mark2_0_dynpars, &tool, q, qd, qdd, g, trq);

        printf("\nTest of dyn_inv_agile_extended for HKM1800\n");
        printf("ret: %d\n", ret);
        for(int k=0; k<9; k++){
                printf("trq[%d]: %f\n", k, trq[k]);
                assert(fabs(trq[k]-trq_true[k]) < 1e-10);
        }
}


static void test_fric(void)
{
        double qd1[4] = {0.0, 0.0, 0.0, 0.0};
        double qd2[4] = {1.0, 1.0, 1.0, 1.0};

        double trq[4];

        printf("\n\nTest of fric trq calculation for mark2_0:\n");

        printf("\nZero velocity:\n");
        fric_trq_agile(mark2_0_fricpars, qd1, trq);
        printf("trq: %f %f %f %f\n", trq[0], trq[1], trq[2], trq[3]);

        printf("\nPositive velocity for all joints:\n");
        fric_trq_agile(mark2_0_fricpars, qd2, trq);
        printf("trq: %f %f %f %f\n", trq[0], trq[1], trq[2], trq[3]);
}


int main(void)
{
        test_dyn_inv_agile_extended();
        test_dyn_inv_mark2_0();
        test_dyn_inv_grav_mark2_0();
        test_dyn_inv_granular_mark2_0();
        test_inertia_matrix();
        test_inertia_matrix_new_joint_space();
        test_fric();
}
