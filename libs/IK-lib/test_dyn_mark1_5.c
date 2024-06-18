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
#include "mark1_5.h"
#include "mark1_5_lpoe.h"


static void test_dyn_inv_mark1_5(void)
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

        double trq_py1[4] = {85.70861523422056, 43.74973088704732,
                             104.13075850016985, -1.4273637308259468};
        double trq_py2[4] = {86.21469763940492, 44.34879829880508,
                             104.0718392105158, -1.425983279839376};

        printf("\nTest of dyn_inv for mark1_5:\n");

        printf("\nWithout all coriolis/centrifugal forces terms:\n");
        ret = dyn_inv_agile(&mark1_5, &mark1_5_lpoe, mark1_5_dynpars,
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
        ret = dyn_inv_agile(&mark1_5, &mark1_5_lpoe, mark1_5_dynpars,
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

        printf("\nTest of inertia_matrix for mark1_5:\n");

        printf("\nTest pos 1:\n");
        ret = inertia_matrix(&mark1_5, &mark1_5_lpoe, mark1_5_dynpars,
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
        ret = inertia_matrix(&mark1_5, &mark1_5_lpoe, mark1_5_dynpars,
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
        ret = inertia_matrix(&mark1_5, &mark1_5_lpoe, mark1_5_dynpars,
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

        double trq_true[9] = {-2.281224328742794, 65.46053082200622,
                              -4.954261886810023, 0.1612388799512403,
                              4.232086239462257, -0.00019450770920200673,
                              -0.18740305912006688, -0.032905205332857665,
                              -4.8905785829597};

        ret = dyn_inv_agile_extended(&mark1_5_lpoe, mark1_5_dynpars, &tool, q, qd, qdd, g, trq);

        printf("\nTest of dyn_inv_agile_extended for mark1_5_lpoe\n");
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

        printf("\n\nTest of fric trq calculation for mark1_5:\n");

        printf("\nZero velocity:\n");
        fric_trq_agile(mark1_5_fricpars, qd1, trq);
        printf("trq: %f %f %f %f\n", trq[0], trq[1], trq[2], trq[3]);

        printf("\nPositive velocity for all joints:\n");
        fric_trq_agile(mark1_5_fricpars, qd2, trq);
        printf("trq: %f %f %f %f\n", trq[0], trq[1], trq[2], trq[3]);
}


int main(void)
{
        test_dyn_inv_agile_extended();
        test_dyn_inv_mark1_5();
        test_inertia_matrix();
        test_fric();
}
