/*
 * Copyright (C) 2021 Cognibotics
 *
 * Use or copying of this file or any part of it, without explicit
 * permission from Cognibotics is not allowed in any way.
 */
#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

#include "kinematics.h"
#include "kin_model_long.h"
#include "mark1_5_nom.h"
#include "mark2_0_nom.h"

static void test_fwd_new_joint_space(void)
{
        double q1[4] = {0.1, 1.7 - M_PI_2, 0.3, 10.0};
        double q2[4] = {0.3, 1.7349885084000443 - M_PI_2, 0.305731843866098, 10.792737179952496};

        double th1[4];
        double th2[4];

        double th1_py[4] = {0.1, 0.16282714906828305, 0.2962614138422491, 0.24200338553347223};
        double th2_py[4] = {0.3, 0.2, 0.3, 0.2};

        int ret;

        printf("\nTest of fwd_new_joint_space for HKM1800:\n");
        printf("Position 1:\n");
        ret = fwd_new_joint_space(&kin_model_long, q1, th1);
        printf("ret: %d\n", ret);
        assert(ret == 0);

        printf("th1: %f %f %f %f\n", th1[0], th1[1], th1[2], th1[3]);
        for (int i = 0; i < 4; i++)
                assert(fabs(th1[i]-th1_py[i]) < 1e-12);

        printf("Position 2:\n");
        ret = fwd_new_joint_space(&kin_model_long, q2, th2);
        printf("ret: %d\n", ret);
        assert(ret == 0);

        printf("th2: %f %f %f %f\n", th2[0], th2[1], th2[2], th2[3]);
        for (int i = 0; i < 4; i++)
                assert(fabs(th2[i]-th2_py[i]) < 1e-12);
}


static void test_fwd_new_joint_space_mark1_5(void)
{
        double q1[4] = {0.1, 1.7 - M_PI_2, 0.3, 1.0};
        double q2[4] = {0.3, 1.7349885084000443 - M_PI_2, 0.305731843866098, 1.792737179952496};

        double th1[4];
        double th2[4];

        double th1_py[4] = {0.1, 0.16690765630079074, 0.29607189781616783, 1.0};
        double th2_py[4] = {0.3, 0.20542890825255436, 0.2996851175663846, 1.792737179952496};

        int ret;

        printf("\nTest of fwd_new_joint_space for Mark1_5:\n");
        printf("Position 1:\n");
        ret = fwd_new_joint_space(&mark1_5_nom, q1, th1);
        printf("ret: %d\n", ret);
        assert(ret == 0);

        printf("th1: %f %f %f %f\n", th1[0], th1[1], th1[2], th1[3]);
        for (int i = 0; i < 4; i++)
                assert(fabs(th1[i]-th1_py[i]) < 1e-12);

        printf("Position 2:\n");
        ret = fwd_new_joint_space(&mark1_5_nom, q2, th2);
        printf("ret: %d\n", ret);
        assert(ret == 0);

        printf("th2: %f %f %f %f\n", th2[0], th2[1], th2[2], th2[3]);
        for (int i = 0; i < 4; i++)
                assert(fabs(th2[i]-th2_py[i]) < 1e-12);
}

static void test_fwd_new_joint_space_mark2_0(void)
{
        double q1[4] = {0.1, 1.7 - M_PI_2, 0.3, 1.0};
        double q2[4] = {0.3, 1.7349885084000443 - M_PI_2, 0.305731843866098, 1.792737179952496};

        double th1[4];
        double th2[4];

        // double th1_py[4] = {0.1, 0.16690765630079074, 0.29607189781616783, 1.0};
        // double th2_py[4] = {0.3, 0.20542890825255436, 0.2996851175663846, 1.792737179952496};

        int ret;

        printf("\nTest of fwd_new_joint_space for Mark2_0:\n");
        printf("Position 1:\n");
        ret = fwd_new_joint_space(&mark2_0_nom, q1, th1);
        printf("ret: %d\n", ret);
        assert(ret == 0);

        printf("th1: %f %f %f %f\n", th1[0], th1[1], th1[2], th1[3]);
        // for (int i = 0; i < 4; i++)
        //         assert(fabs(th1[i] - th1_py[i]) < 1e-12);

        printf("Position 2:\n");
        ret = fwd_new_joint_space(&mark2_0_nom, q2, th2);
        printf("ret: %d\n", ret);
        assert(ret == 0);

        printf("th2: %f %f %f %f\n", th2[0], th2[1], th2[2], th2[3]);
        // for (int i = 0; i < 4; i++)
        //         assert(fabs(th2[i] - th2_py[i]) < 1e-12);
}

static void test_inv_new_joint_space(void)
{
        double th1[4] = {0.3, 0.2, 0.3, 0.2};
        double th2[4] = {0.1, 0.16282714906828305, 0.2962614138422491, 0.24200338553347223};

        double q1[4];
        double q2[4];

        double q1_py[4] = {0.3, 1.7349885084000443 - M_PI_2, 0.305731843866098, 10.792737179952496};
        double q2_py[4] = {0.1, 1.7 - M_PI_2, 0.3, 10.0};

        int ret;

        printf("\nTest of inv_new_joint_space for HKM1800:\n");
        printf("Position 1:\n");
        ret = inv_new_joint_space(&kin_model_long, th1, q1);
        printf("ret: %d\n", ret);
        assert(ret == 0);

        printf("q1: %f %f %f %f\n", q1[0], q1[1], q1[2], q1[3]);
        for (int i = 0; i < 4; i++)
                assert(fabs(q1[i]-q1_py[i]) < 1e-12);

        printf("Position 2:\n");
        ret = inv_new_joint_space(&kin_model_long, th2, q2);
        printf("ret: %d\n", ret);
        assert(ret == 0);

        printf("q2: %f %f %f %f\n", q2[0], q2[1], q2[2], q2[3]);
        for (int i = 0; i < 4; i++)
                assert(fabs(q2[i]-q2_py[i]) < 1e-12);
}


static void test_inv_new_joint_space_mark1_5(void)
{
        double th1[4] = {0.3, 0.2, 0.3, 0.2};
        double th2[4] = {0.1, 0.16282714906828305, 0.2962614138422491, 0.24200338553347223};
        double th3[4] = {0.1, 0.16690765630079074, 0.29607189781616783, 1.0};

        double q1[4];
        double q2[4];
        double q3[4];

        double q1_py[4] = {0.3, 1.729900156090201 - M_PI_2, 0.305731843866098, 0.2};
        double q2_py[4] = {0.1, 1.6961659140659773 - M_PI_2, 0.3, 0.24200338553347223};
        double q3_py[4] = {0.1, 1.7 - M_PI_2, 0.3, 1.0};

        int ret;

        printf("\nTest of inv_new_joint_space for Mark1_5:\n");
        printf("Position 1:\n");
        ret = inv_new_joint_space(&mark1_5_nom, th1, q1);
        printf("ret: %d\n", ret);
        assert(ret == 0);

        printf("q1: %f %f %f %f\n", q1[0], q1[1], q1[2], q1[3]);
        for (int i = 0; i < 4; i++)
                assert(fabs(q1[i]-q1_py[i]) < 1e-12);

        printf("Position 2:\n");
        ret = inv_new_joint_space(&mark1_5_nom, th2, q2);
        printf("ret: %d\n", ret);
        assert(ret == 0);

        printf("q2: %f %f %f %f\n", q2[0], q2[1], q2[2], q2[3]);
        for (int i = 0; i < 4; i++)
                assert(fabs(q2[i]-q2_py[i]) < 1e-12);

        printf("Position 3:\n");
        ret = inv_new_joint_space(&mark1_5_nom, th3, q3);
        printf("ret: %d\n", ret);
        assert(ret == 0);

        printf("q3: %f %f %f %f\n", q3[0], q3[1], q3[2], q3[3]);
        for (int i = 0; i < 4; i++)
                assert(fabs(q3[i]-q3_py[i]) < 1e-12);
}

static void test_inv_new_joint_space_mark2_0(void)
{
        double th1[4] = {0.3, 0.2, 0.3, 0.2};
        double th2[4] = {0.1, 0.16282714906828305, 0.2962614138422491, 0.24200338553347223};
        double th3[4] = {0.1, 0.16690765630079074, 0.29607189781616783, 1.0};

        double q1[4];
        double q2[4];
        double q3[4];

        // double q1_py[4] = {0.3, 1.729900156090201 - M_PI_2, 0.305731843866098, 0.2};
        // double q2_py[4] = {0.1, 1.6961659140659773 - M_PI_2, 0.3, 0.24200338553347223};
        // double q3_py[4] = {0.1, 1.7 - M_PI_2, 0.3, 1.0};

        int ret;

        printf("\nTest of inv_new_joint_space for Mark2_0:\n");
        printf("Position 1:\n");
        ret = inv_new_joint_space(&mark2_0_nom, th1, q1);
        printf("ret: %d\n", ret);
        assert(ret == 0);

        printf("q1: %f %f %f %f\n", q1[0], q1[1], q1[2], q1[3]);
        // for (int i = 0; i < 4; i++)
        //         assert(fabs(q1[i] - q1_py[i]) < 1e-12);

        printf("Position 2:\n");
        ret = inv_new_joint_space(&mark2_0_nom, th2, q2);
        printf("ret: %d\n", ret);
        assert(ret == 0);

        printf("q2: %f %f %f %f\n", q2[0], q2[1], q2[2], q2[3]);
        // for (int i = 0; i < 4; i++)
        //         assert(fabs(q2[i] - q2_py[i]) < 1e-12);

        printf("Position 3:\n");
        ret = inv_new_joint_space(&mark2_0_nom, th3, q3);
        printf("ret: %d\n", ret);
        assert(ret == 0);

        printf("q3: %f %f %f %f\n", q3[0], q3[1], q3[2], q3[3]);
        // for (int i = 0; i < 4; i++)
        //         assert(fabs(q3[i] - q3_py[i]) < 1e-12);
}

static void test_calc_q23_dot(void)
{
        double q1[4] = {0.1, 1.5 - M_PI_2, 0.2, 10.0};
        double q2[4] = {1.1, 1.7 - M_PI_2, -0.1, 42.0};
        double q23_1 = -0.055313274485151;
        double q23_2 = 0.12513077234680736;

        double dq23dq2;
        double dq23dq3;

        double dq23dq2_py1 = 0.995838924652051;
        double dq23dq3_py1 = 0.10921801771371435;
        double dq23dq2_py2 = 0.9814352080726642;
        double dq23dq3_py2 = 0.004234134689326903;

        int ret;

        printf("\nTest of calc_q23_dot for HKM1800:\n");
        printf("Position 1:\n");
        ret = calc_q23_dot(&kin_model_long, q1, q23_1, 0, &dq23dq2, &dq23dq3);
        printf("ret: %d\n", ret);
        assert(ret == 0);

        printf("dq23dq2: %f\n", dq23dq2);
        printf("dq23dq3: %f\n", dq23dq3);
        assert(fabs(dq23dq2-dq23dq2_py1) < 1e-12);
        assert(fabs(dq23dq3-dq23dq3_py1) < 1e-12);

        printf("Position 2:\n");
        ret = calc_q23_dot(&kin_model_long, q2, q23_2, 0, &dq23dq2, &dq23dq3);
        printf("ret: %d\n", ret);
        assert(ret == 0);

        printf("dq23dq2: %f\n", dq23dq2);
        printf("dq23dq3: %f\n", dq23dq3);
        assert(fabs(dq23dq2-dq23dq2_py2) < 1e-12);
        assert(fabs(dq23dq3-dq23dq3_py2) < 1e-12);
}


static void test_calc_q23_dot_mark1_5(void)
{
        double q1[4] = {0.1, 1.5 - M_PI_2, 0.2, 1.0};
        double q2[4] = {1.1, 1.7 - M_PI_2, -0.1, 0.42};
        double q23_1 = -0.055313274485151;
        double q23_2 = 0.12513077234680736;

        double dq23dq2;
        double dq23dq3;

        double dq23dq2_py1 = 1.00710791032868;
        double dq23dq3_py1 = 0.11087675768419135;
        double dq23dq2_py2 = 1.0140192846071128;
        double dq23dq3_py2 = 0.004079435656634;

        int ret;

        printf("\nTest of calc_q23_dot for Mark1_5:\n");
        printf("Position 1:\n");
        ret = calc_q23_dot(&mark1_5_nom, q1, q23_1, 0, &dq23dq2, &dq23dq3);
        printf("ret: %d\n", ret);
        assert(ret == 0);

        printf("dq23dq2: %f\n", dq23dq2);
        printf("dq23dq3: %f\n", dq23dq3);
        assert(fabs(dq23dq2-dq23dq2_py1) < 1e-12);
        assert(fabs(dq23dq3-dq23dq3_py1) < 1e-12);

        printf("Position 2:\n");
        ret = calc_q23_dot(&mark1_5_nom, q2, q23_2, 0, &dq23dq2, &dq23dq3);
        printf("ret: %d\n", ret);
        assert(ret == 0);

        printf("dq23dq2: %f\n", dq23dq2);
        printf("dq23dq3: %f\n", dq23dq3);
        assert(fabs(dq23dq2-dq23dq2_py2) < 1e-12);
        assert(fabs(dq23dq3-dq23dq3_py2) < 1e-12);
}


static void test_inv44_special(void)
{
        double jac[4][4] = {{1.0, 0.0, 0.0, 0.0},
                            {0.0, 1.0, 2.0, 0.0},
                            {0.0, 3.0, 4.0, 0.0},
                            {0.0, 5.0, 6.0, 8.0}};
        double jac_inv[4][4];
        int ret;
        double jac_inv_true[4][4] = {{1.0, 0.0, 0.0, 0.0},
                                     {0.0, -2.0, 1.0, 0.0},
                                     {0.0, 1.5, -0.5, 0.0},
                                     {0.0, 0.125, -0.25, 0.125}};

        printf("\nTest inv44_special:\n");
        ret = inv_44_special(jac, jac_inv);
        printf("ret: %d\n", ret);
        assert(ret == 0);
        for (int i1 = 0; i1 < 4; i1++)
                for (int i2 = 0; i2 < 4; i2++)
                        assert(fabs(jac_inv[i1][i2] - jac_inv_true[i1][i2]) < 1e-12);
}


static void test_calc_derivatives_new_joint_space(void)
{
        double th[4] = {0.1, 0.2, 0.3, 0.4};
        double q[4];
        double jac[4][4];
        double gacc[4][4][4];

        double q_py[4] = {0.1, 1.7349885084000443 - M_PI_2, 0.305731843866098, 13.843173197885335};
        double jac_py[4][4] = {{1.0, 0.0, 0.0, 0.0},
                               {0.0, 1.0269629333816312, 0.2174258753263463, 0.0},
                               {0.0, -0.05877241901113421, 0.9711405021608944, 0.0},
                               {0.0, -2.571906460277495, -0.4475435824630169, 0.06667902579193807}};
        double gacc1_py[4][4] = {{0.0, 0.0, 0.0, 0.0},
                                 {0.0, -0.11359333992004395, 0.304903507232666, 0.0},
                                 {0.0, -0.29338836669921875, -0.24994182586669922, 0.0},
                                 {0.0, 0.7069716453552246, -0.7573337554931641, 8.83936882019043e-05}};
        double gacc2_py[4][4] = {{0.0, 0.0, 0.0, 0.0},
                                 {0.0, 0.304903507232666, 0.6768217086791992, 0.0},
                                 {0.0, -0.24994182586669922, -0.10136890411376953, 0.0},
                                 {0.0, -0.7573337554931641, -0.4745912551879883, -0.00020712614059448242}};
        double gacc3_py[4][4] = {{0.0, 0.0, 0.0, 0.0},
                                 {0.0, 0.0, 0.0, 0.0},
                                 {0.0, 0.0, 0.0, 0.0},
                                 {0.0, 8.83936882019043e-05, -0.00020712614059448242, 0.0007738247513771057}};
        int ret;

        printf("\nTest of calc_derivatives_new_joint_space:\n");
        double q23;
        inv_new_joint_space_full(&kin_model_long, th, q, &q23);
        ret = calc_derivatives_new_joint_space(&kin_model_long, th, q, q23, jac, gacc);
        printf("ret: %d\n", ret);
        assert(ret == 0);

        printf("\nq: %f %f %f %f\n", q[0], q[1], q[2], q[3]);
        for (int i1 = 0; i1 < 4; i1++)
                assert(fabs(q[i1] - q_py[i1]) < 1e-12);

        printf("\njac:\n");
        for (int i1 = 0; i1 < 4; i1++) {
                for (int i2 = 0; i2 < 4; i2++)
                        printf("%f ", jac[i1][i2]);
                printf("\n");
        }

        for (int i1 = 0; i1 < 4; i1++) {
                double tol = i1 == 3 ? 1e-9 : 1e-14;
                for (int i2 = 0; i2 < 4; i2++)
                        assert(fabs(jac[i1][i2] - jac_py[i1][i2]) < tol);
        }

        printf("\ngacc:\n");
        for (int i1 = 0; i1 < 4; i1++) {
                printf("gacc[%d]:\n", i1);
                for (int i2 = 0; i2 < 4; i2++) {
                        for (int i3 = 0; i3 < 4; i3++)
                                printf("%f ", gacc[i1][i2][i3]);
                        printf("\n");
                }
        }

        double tol1 = 1e10;
        double tol2 = 1e4;
        for (int i1 = 0; i1 < 4; i1++) {
                double tol = i1 == 3 ? tol2 : tol1;
                for (int i2 = 0; i2 < 4; i2++)
                        assert(fabs(gacc[0][i1][i2]) < tol);
        }
        for (int i1 = 0; i1 < 4; i1++) {
                double tol = i1 == 3 ? tol2 : tol1;
                for (int i2 = 0; i2 < 4; i2++)
                        assert(fabs(gacc[1][i1][i2]-gacc1_py[i1][i2]) < tol);
        }
        for (int i1 = 0; i1 < 4; i1++) {
                double tol = i1 == 3 ? tol2 : tol1;
                for (int i2 = 0; i2 < 4; i2++)
                        assert(fabs(gacc[2][i1][i2]-gacc2_py[i1][i2]) < tol);
        }
        for (int i1 = 0; i1 < 4; i1++) {
                double tol = i1 == 3 ? tol2 : tol1;
                for (int i2 = 0; i2 < 4; i2++)
                        assert(fabs(gacc[3][i1][i2]-gacc3_py[i1][i2]) < tol);
        }
}


static void test_calc_derivatives_new_joint_space_mark1_5(void)
{
        double th[4] = {0.1, 0.2, 0.3, 0.4};
        double q[4];
        double jac[4][4];
        double gacc[4][4][4];

        double q_py[4] = {0.1, 1.729900156090201 - M_PI_2, 0.305731843866098, 0.4};
        double jac_py[4][4] = {{1.0, 0.0, 0.0, 0.0},
                               {0.0, 1.0668767467081592, 0.221490165228808, 0.0},
                               {0.0, -0.06105666052064462, 0.9709079055000358, 0.0},
                               {0.0, 0.0, 0.0, 1.0}};
        double gacc1_py[4][4] = {{0.0, 0.0, 0.0, 0.0},
                                 {0.0, 0.02050042152404785, 0.32576632499694824, 0.0},
                                 {0.0, -0.32482481002807617, -0.26140594482421875, 0.0},
                                 {0.0, 0.0, 0.0, 0.0}};
        double gacc2_py[4][4] = {{0.0, 0.0, 0.0, 0.0},
                                 {0.0, 0.32576632499694824, 0.6896450519561768, 0.0},
                                 {0.0, -0.26140594482421875, -0.10394763946533203, 0.0},
                                 {0.0, 0.0, 0.0, 0.0}};
        double gacc3_py[4][4] = {{0.0, 0.0, 0.0, 0.0},
                                 {0.0, 0.0, 0.0, 0.0},
                                 {0.0, 0.0, 0.0, 0.0},
                                 {0.0, 0.0, 0.0, 0.0}};
        int ret;

        printf("\nTest of calc_derivatives_new_joint_space for mark1_5:\n");
        double q23;
        inv_new_joint_space_full(&mark1_5_nom, th, q, &q23);
        ret = calc_derivatives_new_joint_space(&mark1_5_nom, th, q, q23, jac, gacc);
        printf("ret: %d\n", ret);
        assert(ret == 0);

        printf("\nq: %f %f %f %f\n", q[0], q[1], q[2], q[3]);
        for (int i1 = 0; i1 < 4; i1++)
                assert(fabs(q[i1] - q_py[i1]) < 1e-12);

        printf("\njac:\n");
        for (int i1 = 0; i1 < 4; i1++) {
                for (int i2 = 0; i2 < 4; i2++)
                        printf("%f ", jac[i1][i2]);
                printf("\n");
        }

        for (int i1 = 0; i1 < 4; i1++) {
                double tol = i1 == 3 ? 1e-9 : 1e-14;
                for (int i2 = 0; i2 < 4; i2++)
                        assert(fabs(jac[i1][i2] - jac_py[i1][i2]) < tol);
        }

        printf("\ngacc:\n");
        for (int i1 = 0; i1 < 4; i1++) {
                printf("gacc[%d]:\n", i1);
                for (int i2 = 0; i2 < 4; i2++) {
                        for (int i3 = 0; i3 < 4; i3++)
                                printf("%f ", gacc[i1][i2][i3]);
                        printf("\n");
                }
        }

        double tol1 = 1e10;
        double tol2 = 1e4;
        for (int i1 = 0; i1 < 4; i1++) {
                double tol = i1 == 3 ? tol2 : tol1;
                for (int i2 = 0; i2 < 4; i2++)
                        assert(fabs(gacc[0][i1][i2]) < tol);
        }
        for (int i1 = 0; i1 < 4; i1++) {
                double tol = i1 == 3 ? tol2 : tol1;
                for (int i2 = 0; i2 < 4; i2++)
                        assert(fabs(gacc[1][i1][i2]-gacc1_py[i1][i2]) < tol);
        }
        for (int i1 = 0; i1 < 4; i1++) {
                double tol = i1 == 3 ? tol2 : tol1;
                for (int i2 = 0; i2 < 4; i2++)
                        assert(fabs(gacc[2][i1][i2]-gacc2_py[i1][i2]) < tol);
        }
        for (int i1 = 0; i1 < 4; i1++) {
                double tol = i1 == 3 ? tol2 : tol1;
                for (int i2 = 0; i2 < 4; i2++)
                        assert(fabs(gacc[3][i1][i2]-gacc3_py[i1][i2]) < tol);
        }
}


static void test_kin_vel_acc_new_joint_space(void)
{
        double th[4] = {0.3, 0.2, 0.1, 0.0};
        double th_dot[4] = {1.0, -1.0, 1.0, -1.0};
        double th_dotdot[4] = {-10.0, 10.0, -10.0, 10.0};

        int ret;
        double q[4];
        double qd[4];
        double qdd[4];

        double q_py[4] = {0.3, 1.766872704713431 - M_PI_2, 0.10201994014414235, 7.571349261071873};
        double qd_py[4] = {1.0, -1.1172239692847978, 0.9993838723276787, -53.77016503937007};
        double qdd_py[4] = {-10.0, 10.94476320380705, -10.297540004359133, 483.39490775613217};

        printf("\nTest inv_vel_acc_new_joint_space:\n");
        ret = inv_vel_acc_new_joint_space(&kin_model_long, th,
                        th_dot, th_dotdot, q, qd, qdd);
        printf("ret: %d\n", ret);
        assert(ret == 0);

        printf("\nq: %f %f %f %f\n", q[0], q[1], q[2], q[3]);
        for (int i = 0; i < 4; i++)
                assert(fabs(q[i] - q_py[i]) < 1e-14);

        printf("\nqd: %f %f %f %f\n", qd[0], qd[1], qd[2], qd[3]);
        for (int i = 0; i < 4; i++)
                assert(fabs(qd[i] - qd_py[i]) < 1e-10);

        printf("\nqdd: %f %f %f %f\n", qdd[0], qdd[1], qdd[2], qdd[3]);
        for (int i = 0; i < 4; i++)
                assert(fabs(qdd[i] - qdd_py[i]) < 1e-10);

        double th_out[4];
        double th_dot_out[4];
        double th_dotdot_out[4];

        printf("\nTest fwd_vel_acc_new_joint_space:\n");
        ret = fwd_vel_acc_new_joint_space(&kin_model_long, q,
                        qd, qdd, th_out, th_dot_out, th_dotdot_out);
        printf("ret: %d\n", ret);
        assert(ret == 0);

        printf("\nth: %f %f %f %f\n", th[0], th[1], th[2], th[3]);
        for (int i = 0; i < 4; i++)
                assert(fabs(th_out[i] - th[i]) < 1e-14);

        printf("\nth_dot: %f %f %f %f\n", th_dot_out[0], th_dot_out[1], th_dot_out[2], th_dot_out[3]);
        for (int i = 0; i < 4; i++)
                assert(fabs(th_dot_out[i] - th_dot[i]) < 1e-10);

        printf("\nth_dotdot: %f %f %f %f\n", th_dotdot_out[0], th_dotdot_out[1], th_dotdot_out[2], th_dotdot_out[3]);
        assert(fabs(th_dotdot_out[0] - th_dotdot[0]) < 1e-10);
        for (int i = 1; i < 4; i++)
                assert(fabs(th_dotdot_out[i] - th_dotdot[i]) < 1e-3);
}


static void test_kin_vel_acc_new_joint_space_mark1_5(void)
{
        double th[4] = {0.3, 0.2, 0.1, 0.0};
        double th_dot[4] = {1.0, -1.0, 1.0, -1.0};
        double th_dotdot[4] = {-10.0, 10.0, -10.0, 10.0};

        int ret;
        double q[4];
        double qd[4];
        double qdd[4];

        double q_py[4] = {0.3, 1.7611372993848793 - M_PI_2, 0.10201994014414235, 0.0};
        double qd_py[4] = {1.0, -1.074143784453132, 0.9993838723276786, -1.0};
        double qdd_py[4] = {-10.0, 10.360032288357578, -10.29753977414356, 10.0};

        printf("\nTest inv_vel_acc_new_joint_space Mark1_5:\n");
        ret = inv_vel_acc_new_joint_space(&mark1_5_nom, th,
                        th_dot, th_dotdot, q, qd, qdd);
        printf("ret: %d\n", ret);
        assert(ret == 0);

        printf("\nq: %f %f %f %f\n", q[0], q[1], q[2], q[3]);
        for (int i = 0; i < 4; i++)
                assert(fabs(q[i] - q_py[i]) < 1e-14);

        printf("\nqd: %f %f %f %f\n", qd[0], qd[1], qd[2], qd[3]);
        for (int i = 0; i < 4; i++)
                assert(fabs(qd[i] - qd_py[i]) < 1e-10);

        printf("\nqdd: %f %f %f %f\n", qdd[0], qdd[1], qdd[2], qdd[3]);
        for (int i = 0; i < 4; i++)
                assert(fabs(qdd[i] - qdd_py[i]) < 1e-10);

        double th_out[4];
        double th_dot_out[4];
        double th_dotdot_out[4];

        printf("\nTest fwd_vel_acc_new_joint_space Mark1_5:\n");
        ret = fwd_vel_acc_new_joint_space(&mark1_5_nom, q,
                        qd, qdd, th_out, th_dot_out, th_dotdot_out);
        printf("ret: %d\n", ret);
        assert(ret == 0);

        printf("\nth: %f %f %f %f\n", th[0], th[1], th[2], th[3]);
        for (int i = 0; i < 4; i++)
                assert(fabs(th_out[i] - th[i]) < 1e-14);

        printf("\nth_dot: %f %f %f %f\n", th_dot_out[0], th_dot_out[1], th_dot_out[2], th_dot_out[3]);
        for (int i = 0; i < 4; i++)
                assert(fabs(th_dot_out[i] - th_dot[i]) < 1e-10);

        printf("\nth_dotdot: %f %f %f %f\n", th_dotdot_out[0], th_dotdot_out[1], th_dotdot_out[2], th_dotdot_out[3]);
        assert(fabs(th_dotdot_out[0] - th_dotdot[0]) < 1e-10);
        for (int i = 1; i < 3; i++)
                assert(fabs(th_dotdot_out[i] - th_dotdot[i]) < 1e-3);
        assert(fabs(th_dotdot_out[3] - th_dotdot[3]) < 1e-10);
}

static void test_kin_vel_acc_new_joint_space_mark2_0(void)
{
        double th[4] = {0.0, 0.0, 0.0, 74.16*M_PI/180.0};
        double th_dot[4] = {0.0, 12.0*M_PI/180.0, 0.0, 0.0};
        double th_dotdot[4] = {0.0, 1200.0*M_PI/180.0, 0.0, 0.0};

        int ret;
        double q[4];
        double qd[4];
        double qdd[4];

        // double q_py[4] = {0.3, 1.7611372993848793 - M_PI_2, 0.10201994014414235, 0.0};
        // double qd_py[4] = {1.0, -1.074143784453132, 0.9993838723276786, -1.0};
        // double qdd_py[4] = {-10.0, 10.360032288357578, -10.29753977414356, 10.0};

        printf("\nTest inv_vel_acc_new_joint_space Mark2_0:\n");
        ret = inv_vel_acc_new_joint_space(&mark2_0_nom, th,
                                          th_dot, th_dotdot, q, qd, qdd);
        printf("ret: %d\n", ret);
        assert(ret == 0);

        printf("\nq: %f %f %f %f\n", q[0], q[1], q[2], q[3]);
        // for (int i = 0; i < 4; i++)
        //         assert(fabs(q[i] - q_py[i]) < 1e-14);

        printf("\nqd: %f %f %f %f\n", qd[0], qd[1]*180.0/M_PI, qd[2], qd[3]);
        // for (int i = 0; i < 4; i++)
        //         assert(fabs(qd[i] - qd_py[i]) < 1e-10);

        printf("\nqdd: %f %f %f %f\n", qdd[0], qdd[1]*180.0/M_PI, qdd[2], qdd[3]);
        // for (int i = 0; i < 4; i++)
        //         assert(fabs(qdd[i] - qdd_py[i]) < 1e-10);

        double th_out[4];
        double th_dot_out[4];
        double th_dotdot_out[4];

        printf("\nTest fwd_vel_acc_new_joint_space Mark2_0:\n");
        ret = fwd_vel_acc_new_joint_space(&mark2_0_nom, q,
                                          qd, qdd, th_out, th_dot_out, th_dotdot_out);
        printf("ret: %d\n", ret);
        assert(ret == 0);

        printf("\nth: %f %f %f %f\n", th[0], th[1], th[2], th[3]);
        for (int i = 0; i < 4; i++)
                assert(fabs(th_out[i] - th[i]) < 1e-14);

        printf("\nth_dot: %f %f %f %f\n", th_dot_out[0], th_dot_out[1]*180.0/M_PI, th_dot_out[2], th_dot_out[3]);
        for (int i = 0; i < 4; i++)
                assert(fabs(th_dot_out[i] - th_dot[i]) < 1e-10);

        printf("\nth_dotdot: %f %f %f %f\n", th_dotdot_out[0], th_dotdot_out[1]*180.0/M_PI, th_dotdot_out[2], th_dotdot_out[3]);
        assert(fabs(th_dotdot_out[0] - th_dotdot[0]) < 1e-10);
        for (int i = 1; i < 3; i++)
                assert(fabs(th_dotdot_out[i] - th_dotdot[i]) < 1e-3);
        assert(fabs(th_dotdot_out[3] - th_dotdot[3]) < 1e-10);
}

int main(void)
{
        test_fwd_new_joint_space();
        test_inv_new_joint_space();
        test_calc_q23_dot();
        test_inv44_special();
        test_calc_derivatives_new_joint_space();
        test_kin_vel_acc_new_joint_space();

        test_fwd_new_joint_space_mark1_5();
        test_inv_new_joint_space_mark1_5();
        test_calc_q23_dot_mark1_5();
        test_calc_derivatives_new_joint_space_mark1_5();
        test_kin_vel_acc_new_joint_space_mark1_5();

        test_fwd_new_joint_space_mark2_0();
        test_inv_new_joint_space_mark2_0();
        test_kin_vel_acc_new_joint_space_mark2_0();
}
