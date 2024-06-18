#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "kinematics.h"
#include "mark2_0.h"
#include "mark2_0_perturbed.h"

int main(void)
{

        double q[4] = {1.0, 0.5, 0.2, 2.0};
        int ret;
        double orient_angle;
        double out_tcp[4][4];
        double out_elbow[4][4];

        ret = fwd(&mark2_0, q, out_tcp, &orient_angle, out_elbow, 0.0, 0);
        printf("ret: %d\n", ret);
        assert(ret == 0);

        double pos[3] = {out_tcp[0][3], out_tcp[1][3], out_tcp[2][3]};
        double v[4] = {0.5, 0.5, 0.2, 23.0};
        double acc[4] = {-1.0, 1.0, -1.0, 1.0};
        double q_inv[4];
        double qd_inv[4];
        double qdd_inv[4];
        double qd_inv_jl[4] = {-0.57449014825408562, -0.30023965743322489, 0.22487289912021163, 23.863067035674352};
        double qdd_inv_jl[4] = {1.8499759541376464, -1.7655013024020318, -1.5911170726866655, 1.298046173019892};
        printf("Test inv_with_vel_acc:\n");
        ret = inv_with_vel_acc(&mark2_0, pos, orient_angle, v, acc, q_inv, qd_inv, qdd_inv);
        printf("ret: %d\n", ret);
        printf("q_inv: %f %f %f %f\n", q_inv[0], q_inv[1], q_inv[2], q_inv[3]);
        printf("qd_inv: %f %f %f %f\n", qd_inv[0], qd_inv[1], qd_inv[2], qd_inv[3]);
        printf("qdd_inv: %f %f %f %f\n", qdd_inv[0], qdd_inv[1], qdd_inv[2], qdd_inv[3]);
        for (int i=0; i < 4; i++)
                assert(fabs(q_inv[i] - q[i]) < 1e-10);
        for (int i=0; i < 3; i++)
                assert(fabs(qd_inv[i] - qd_inv_jl[i]) < 1e-4);
        assert(fabs(qd_inv[3] - qd_inv_jl[3]) < 1e-3);
        for (int i=0; i < 3; i++)
                assert(fabs(qdd_inv[i] - qdd_inv_jl[i]) < 1e-4);
        assert(fabs(qdd_inv[3] - qdd_inv_jl[3]) < 10.0);

        double fwdres[4][4];
        double orient_out;
        double v_out[4];
        double a_out[4];

        printf("Test fwd_with_vel_acc:\n");
        ret = fwd_with_vel_acc(&mark2_0, q_inv, qd_inv, qdd_inv, fwdres, &orient_out, v_out, a_out, out_elbow);
        printf("ret: %d\n", ret);
        printf("pos_inv: %f %f %f %f\n", fwdres[0][3], fwdres[1][3], fwdres[2][3], orient_out);
        printf("v_inv: %f %f %f %f\n", v_out[0], v_out[1], v_out[2], v_out[3]);
        printf("a_inv: %f %f %f %f\n", a_out[0], a_out[1], a_out[2], a_out[3]);
        for (int i=0; i < 3; i++)
                assert(fabs(fwdres[i][3] - pos[i]) < 1e-10);
        assert(fabs(orient_out - orient_angle) < 1e-10);
        for (int i=0; i < 3; i++)
                assert(fabs(v_out[i] - v[i]) < 1e-4);
        assert(fabs(v_out[3] - v[3]) < 1e-3);
        for (int i=0; i < 3; i++)
                assert(fabs(a_out[i] - acc[i]) < 1e-4);
        assert(fabs(a_out[3] - acc[3]) < 10.0);

        printf("\n--------Perturbed--------\n");

        ret = fwd(&mark2_0_perturbed, q, out_tcp, &orient_angle, out_elbow, 0.0, 0);
        printf("ret: %d\n", ret);
        assert(ret == 0);

        double pos_per[3] = {out_tcp[0][3], out_tcp[1][3], out_tcp[2][3]};
        double qd_inv_jl_per[4] = {-0.57448946443782256, -0.3002396464879713, 0.22487282002466202, 23.863066300081044};
        double qdd_inv_jl_per[4] = {1.8499781227923007, -1.7655245653373348, -1.5911154869934463, 1.298236828107401};
        printf("Test inv_with_vel_acc:\n");
        ret = inv_with_vel_acc(&mark2_0_perturbed, pos_per, orient_angle, v, acc, q_inv, qd_inv, qdd_inv);
        printf("ret: %d\n", ret);
        printf("q_inv: %f %f %f %f\n", q_inv[0], q_inv[1], q_inv[2], q_inv[3]);
        printf("qd_inv: %f %f %f %f\n", qd_inv[0], qd_inv[1], qd_inv[2], qd_inv[3]);
        printf("qdd_inv: %f %f %f %f\n", qdd_inv[0], qdd_inv[1], qdd_inv[2], qdd_inv[3]);
        for (int i=0; i < 4; i++)
                assert(fabs(q_inv[i] - q[i]) < 1e-5);
        for (int i=0; i < 3; i++)
                assert(fabs(qd_inv[i] - qd_inv_jl_per[i]) < 1e-5);
        assert(fabs(qd_inv[3] - qd_inv_jl_per[3]) < 1e-4);
        for (int i=0; i < 3; i++)
                assert(fabs(qdd_inv[i] - qdd_inv_jl_per[i]) < 1e-1);
        assert(fabs(qdd_inv[3] - qdd_inv_jl_per[3]) < 10.0);

        printf("Test fwd_with_vel_acc:\n");
        ret = fwd_with_vel_acc(&mark2_0_perturbed, q_inv, qd_inv, qdd_inv, fwdres, &orient_out, v_out, a_out, out_elbow);
        printf("ret: %d\n", ret);
        printf("pos_inv: %f %f %f %f\n", fwdres[0][3], fwdres[1][3], fwdres[2][3], orient_out);
        printf("v_inv: %f %f %f %f\n", v_out[0], v_out[1], v_out[2], v_out[3]);
        printf("a_inv: %f %f %f %f\n", a_out[0], a_out[1], a_out[2], a_out[3]);
        for (int i=0; i < 3; i++)
                assert(fabs(fwdres[i][3] - pos_per[i]) < 1e-6);
        assert(fabs(orient_out - orient_angle) < 1e-6);
        for (int i=0; i < 3; i++)
                assert(fabs(v_out[i] - v[i]) < 1e-4);
        assert(fabs(v_out[3] - v[3]) < 1e-3);
        for (int i=0; i < 3; i++)
                assert(fabs(a_out[i] - acc[i]) < 1e-4);
        assert(fabs(a_out[3] - acc[3]) < 10.0);

        return 0;
}
