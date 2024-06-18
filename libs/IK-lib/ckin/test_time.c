/*
 * time.c
 * Copyright (C) 2016 Cognibotics
 */

#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#include "lpoe.h"
#include "lin_alg.h"
#include "kin_skel.h"

extern struct model_lpoe_link lpoe_IRB140[];

#define N_DOF 6
#define NBR_ITE 1000000
void rand_theta(double *thetas)
{
        for (int i = 0; i < N_DOF; ++i)
                thetas[i] = ((double) rand() - 0.5)/RAND_MAX;
}

int main(int argc, char **argv)
{
        double theta[N_DOF];
        double jac[N_DOF][N_DOF];
        double pose[4][4];
        double err[N_DOF];
        double q_step[N_DOF];
        double lambda = 1.0;
        double start, stop;
        srand(666);

        struct model_lpoe model;
        lpoe_init(&model);
        model.lrob = lpoe_IRB140;
        model.n_screw = 7;
        model.n_joints = 6;

        for (int i = 0; i < N_DOF; i++)
                err[i] = 0.1*(i + 1);

        start = clock();
        for (int i = 0; i < NBR_ITE; ++i) {
                rand_theta(theta);
                spatial_jacobian(&model, theta, pose, jac);
        }
        stop = clock();
        printf("%d jacobian calculations execution time: %f\n",
                        NBR_ITE, (double)(stop-start)/CLOCKS_PER_SEC);

        start = clock();
        for (int i = 0; i < NBR_ITE; ++i) {
                rand_theta(theta);
                spatial_jacobian(&model, theta, pose, jac);
                solve_66_gauss_elim(jac, err, q_step);
        }
        stop = clock();
        printf("%d jac+solve_66_gauss_elim calculations execution time: %f\n",
                        NBR_ITE, (double)(stop-start)/CLOCKS_PER_SEC);

        start = clock();
        for (int i = 0; i < NBR_ITE; ++i) {
                rand_theta(theta);
                spatial_jacobian(&model, theta, pose, jac);
                solve_66_6_damped_ls(jac, err, 0.001, q_step);
        }
        stop = clock();
        printf("%d jac+solve_66_6_damped_ls calculations execution time: %f\n",
                        NBR_ITE, (double)(stop-start)/CLOCKS_PER_SEC);
}
