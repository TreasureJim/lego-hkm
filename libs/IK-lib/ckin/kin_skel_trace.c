/*
 * Copyright (C) 2016, 2018 Cognibotics
 *
 * Use or copying of this file or any part of it, without explicit
 * permission from Cognibotics is not allowed in any way.
 */
#include <stdlib.h>
#include <stdio.h>

static FILE *log;
static unsigned long cnt;

void kin_skel_inv_trace(unsigned long i_step, double lambda,
                double q_step_norm, double err_pos, double err_rot,
                double q_curr[6])
{
        if (i_step == 0) {
                char name_buf[128];

                sprintf(name_buf, "kin_skel_trace%03lu.csv", cnt);
                log = fopen(name_buf, "w");
                fprintf(log,
                                "i_step,"
                                "lambda,"
                                "q_step_norm,"
                                "err_pos,"
                                "err_rot,"
                                "q_curr[0],"
                                "q_curr[1],"
                                "q_curr[2],"
                                "q_curr[3],"
                                "q_curr[4],"
                                "q_curr[5]\n"
                        );
                cnt += 1;
        }
        fprintf(log, "%lu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                        i_step,
                        lambda,
                        q_step_norm,
                        err_pos,
                        err_rot,
                        q_curr[0],
                        q_curr[1],
                        q_curr[2],
                        q_curr[3],
                        q_curr[4],
                        q_curr[5]);
}
