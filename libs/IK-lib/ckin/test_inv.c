/*
 * Copyright (C) 2016, 2017, 2018, 2019 Cognibotics
 *
 * Use or copying of this file or any part of it, without explicit
 * permission from Cognibotics is not allowed in any way.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdbool.h>

#include "lpoe.h"
#include "kin_skel.h"
#include "debug.h"

#define MAX_JDIFF (1e-5)

extern struct model_lpoe_link lpoe_IRB140[];

int main(int argc, char **argv)
{
        double target_thetas[][6] = {
                {0.1, 0, 0, 0, 0, 0},
                {0, 0, 0, 0.1, 0.1, 0.1},
                {0.1, 0.1, 0.1, 0.1, 0.1, 0.1},
                {-0.1, -0.1, -0.1, -0.1, -0.1, -0.1},
                {D2R(10), 0, 0, D2R(30), D2R(10), D2R(-30)},
                {D2R(-10), 0, 0, D2R(-30), D2R(-10), D2R(30)},
                {D2R(5), D2R(5), D2R(5), D2R(5), D2R(5), D2R(5)},
                /* Problematic positions (encountered in practice): */
                {0.082538, 0.526414, 0.030095, 0.062925, -1.531567, -2.150944},
                {0.503769, -0.283855, 0.229515, -3.045179, -1.655626, -6.555971},
                {0.500917, -0.349711, 0.238076, -3.094336, -1.473717, -12.152558},
                {0.571091, -0.350559, 0.245613, -2.771367, -1.528575, -12.874756},
                {0.499429, -0.354947, 0.239953, -3.095857, -1.458156, -18.431167},
        };
        unsigned int ambig_targets_start = 7;
        int res = EXIT_SUCCESS;
        bool test = false;
        struct model_lpoe model;
        lpoe_init(&model);
        model.lrob = lpoe_IRB140;
        model.n_screw = 7;
        model.n_joints = 6;

        if (argc > 1) {
                if (!strncmp("test", argv[1], 5))
                        test = true;
        }
        for (unsigned int i = 0; i < sizeof(target_thetas)/(sizeof(double)*6); ++i) {
                unsigned long n = test ? 10000 : 1;
                struct timespec t_start;
                struct timespec t_end;
                double dt;

                int ret;
                double target_pose[4][4];
                double q_start[6] = {0};
                double q_res[6];
                struct kin_skel_stats stats;
                double pose_all_tmp[7][4][4];

                fwd_lpoe(&model, target_thetas[i], 7, pose_all_tmp);
                memcpy(target_pose, pose_all_tmp[6], 4*4*sizeof(double));

                clock_gettime(CLOCK_MONOTONIC_RAW, &t_start);
                for (unsigned long i_n = 0; i_n < n; i_n++) {
                        ret = kin_skel_inv(6, &model, target_pose, q_start,
                                        0, 0, 0, q_res, &stats);
                        if (ret != KIN_SUCCESS)
                                break;
                }
                clock_gettime(CLOCK_MONOTONIC_RAW, &t_end);
                dt = ((t_end.tv_sec - t_start.tv_sec) + 1e-9 * (t_end.tv_nsec - t_start.tv_nsec)) / n;
                if (test)
                        printf("Time to solve for pose %d: %e (%lu iter)\n",
                                        i, dt, stats.n_iter);

                if (ret == KIN_SUCCESS) {

                        if (i < ambig_targets_start) {
                                double jdiff_max = 0.0;

                                /*
                                 * Closest solution from zero position are the
                                 * targets_thetas. That solution should be
                                 * found.
                                 */
                                for (int j = 0; j < 6; j++) {
                                        double jdiff;

                                        jdiff = fabs(target_thetas[i][j] - q_res[j]);
                                        if (jdiff > jdiff_max)
                                                jdiff_max = jdiff;
                                }
                                if (jdiff_max > MAX_JDIFF) {
                                        printf("jdiff: %f\n", jdiff_max);
                                        res = EXIT_FAILURE;
                                        break;
                                }
                        } else {
                                double res_pose[4][4];

                                /*
                                 * Rechable positions with multiple
                                 * revolutions on some wrist
                                 * axes. Accept any equivalent
                                 * solution.
                                 */

                                fwd_lpoe(&model, q_res, 7, pose_all_tmp);
                                memcpy(res_pose, pose_all_tmp[6], 4*4*sizeof(double));

                                for (int j = 0; j < 4; j++) {
                                        for (int k = 0; k < 4; k++) {
                                                double diff;

                                                diff = fabs(target_pose[j][k] - res_pose[j][k]);
                                                if (diff >  0.0001) {
                                                        printf("diff pose\n");
                                                        res = EXIT_FAILURE;
                                                }
                                        }
                                }
                        }
                } else {
                        printf("Pose %d failed with ret %d.\n", i, ret);
                        res = EXIT_FAILURE;
                        break;
                }
        }

        if (test) {
                if (res == EXIT_SUCCESS) {
                        printf("Generated C kinematics test passed.\n");
                } else {
                        printf("Generated C kinematics test FAILED.\n");
                }
        }

        return res;
}
