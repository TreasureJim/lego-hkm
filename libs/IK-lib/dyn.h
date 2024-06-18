/*
 * Copyright (C) 2020 Cognibotics
 *
 * Use or copying of this file or any part of it, without explicit
 * permission from Cognibotics is not allowed in any way.
 */
/*!
 * @file
 * @brief LPOE-based pkm kinematics.
 */
#ifndef DYN_H
#define DYN_H

#include "c_lpoe/lpoe.h"

struct model_lpoe_agile_pkm {
        struct model_lpoe_link *lrob;  /*!< LPOE parameters for robot. */
        int n_screws;
        double last_trans[2][4][4];
        int lam[9];
        int mu[9];
        int mu_inds[10][2];
        int tool_body;
        int chain_ends[2];
};

/*!
 * Dynamic parameters for a single link.
 */
struct dyn_params_link {
        double m;  /*!< Mass [kg]. */
        double r[3];  /*!< Position of center of mass [m]. */
        double ival[3][3];  /*!< Inertia matrix. */
};

/*!
 * Friction parameters for a single joint
 */
struct fricpars_joint {
        double visc;  /*!< Viscous friction parameter. */
        double coul;  /*!< Coulomb friction parameter. */
};
#include "kinematics.h"

void test_func(const double theta[6], double out[4][4]);
void test_func2(const struct model_lpoe_agile_pkm *rob, double out[4][4]);
void kin(const struct model_lpoe_agile_pkm *rob, const double q[9], double tfs[9][4][4]);

void transform_a2b(const double tf[4][4], const double c_a[3], double c_b[3]);
void transform_b2a(const double tf[4][4], const double c_b[3], double c_a[3]);

int dyn_inv_agile_extended(const struct model_lpoe_agile_pkm *rob,
                const struct dyn_params_link dpar[], const struct dyn_params_link *tool,
                const double q[9], const double qd[9], const double qdd[9],
                const double g[3], double trq[9]);

int calc_gmat(const struct agile_pkm_model *rob, const double q[4],
                double delta, int extra_accurate, int calc_gvec,
                const double qd[4], double qe_nom[9], double gmat[9][4], double gvec[9]);

int dyn_inv_agile(const struct agile_pkm_model *rob,
                const struct model_lpoe_agile_pkm *rob_lpoe,
                const struct dyn_params_link dpar[], const struct dyn_params_link *tool,
                const double q[4], const double qd[4], const double qdd[4],
                const double g[3], int full_model, double trq[4]);

void fric_trq_agile(const struct fricpars_joint fricpars[], const double qd[4], double trq[4]);

int inertia_matrix(const struct agile_pkm_model *rob,
                const struct model_lpoe_agile_pkm *rob_lpoe,
                const struct dyn_params_link dpar[], const struct dyn_params_link *tool,
                const double q[4], double imat[4][4]);

int inertia_matrix_new_joint_space(const struct agile_pkm_model *rob,
                const struct model_lpoe_agile_pkm *rob_lpoe,
                const struct dyn_params_link dpar[], const struct dyn_params_link *tool,
                const double th[4], double imat[4][4]);

int dyn_inv_agile_extended_grav(const struct model_lpoe_agile_pkm *rob,
                const struct dyn_params_link dpar[], const struct dyn_params_link *tool,
                const double q[9], const double g[3], double trq[9]);

int calc_gmat_grav(const struct agile_pkm_model *rob, const double q[4],
                double delta, double qe_nom[9], double gmat[9][4]);

int dyn_inv_agile_grav(const struct agile_pkm_model *rob,
                const struct model_lpoe_agile_pkm *rob_lpoe,
                const struct dyn_params_link dpar[], const struct dyn_params_link *tool,
                const double q[4], const double g[3], double trq[4]);

int dyn_inv_agile_granular(const struct agile_pkm_model *rob,
                const struct model_lpoe_agile_pkm *rob_lpoe,
                const struct dyn_params_link dpar[], const struct dyn_params_link *tool,
                const double q[4], const double qd[4], const double qdd[4],
                const double g[3], int full_model, double trq_grav[4], double trq_vel[4], double trq_acc[4]);

#endif
