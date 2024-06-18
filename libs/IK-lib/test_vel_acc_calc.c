#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "kinematics.h"
#include "kin_model.h"

int main(void)
{
        double q[4] = {1.0, 2.0 - M_PI_2, 0.2, 20.0};
        int ret;
        res_q_extra_t res_q_extra;
        ax4_fwd_ret_t ax4_fwd_ret;
        double orient_angle;
        double out_tcp[4][4];
        double out_elbow[4][4];
        double j23[4][2];
        double j23_true[4][2] = {
                        {-0.07726131633785965, 0.07173331144757278},
                        {-0.6392141263487274, -0.18948075082036173},
                        {-0.05172634121592132, 0.5040782822616794},
                        {0, 0}};

        ret = drive_to_cart(&kin_model, q, out_tcp, &orient_angle);
        // ret = fwd_nom(&kin_model, q, out_tcp, &orient_angle, out_elbow, &res_q_extra, &ax4_fwd_ret);
        // printf("ret: %d\n", ret);
        // assert(ret == 0);

        // calc_q23_dot_full(&kin_model, q, &res_q_extra, j23);

        // printf("\nj23:\n");
        // for (int i1 = 0; i1 < 4; i1++) {
        //         for (int i2 = 0; i2 < 2; i2++) {
        //                 printf("%f ", j23[i1][i2]);
        //                 assert(fabs(j23[i1][i2] - j23_true[i1][i2]) < 1e-10);
        //         }
        //         printf("\n");
        // }

        double a[4][4] = {{1.0, 2.0, 3.0, 0.0}, {4.0, 5.0, 6.0, 0.0},
                          {0.0, 7.0, 8.0, 0.0}, {0.0, 0.0, 0.0, 0.0}};
        double a_inv[4][4];
        double a_inv_true[4][4] = {{-1.0/9, 1.0/3.6,-1.0/6, 0},
                                   {-16.0/9, 4.0/9, 1.0/3, 0},
                                   {14.0/9, -7.0/18, -1.0/6, 0},
                                   {0, 0, 0, 0}};

        printf("\nTest inv_33_special:\n");
        ret = inv_33_special(a, a_inv);
        printf("ret: %d\n", ret);
        assert(ret == 0);
        printf("inv:\n");
        for (int i1 = 0; i1 < 3; i1++) {
                for (int i2 = 0; i2 < 3; i2++) {
                        printf("%f ", a_inv[i1][i2]);
                        assert(fabs(a_inv[i1][i2] - a_inv_true[i1][i2]) < 1e-10);
                }
                printf("\n");
        }

        double pos[3] = {out_tcp[0][3], out_tcp[1][3], out_tcp[2][3]};
        double v[4] = {0.5, 0.5, 0.2, 23.0};
        double acc[4] = {-1.0, 1.0, -1.0, 1.0};
        double q_inv[4];
        double qd_inv[4];
        double qdd_inv[4];
        double qd_inv_py[4] = {-0.7710413904636735, -0.6375263165699097, 0.3313435672443226, 318.01039494932843};
        double qdd_inv_py[4] = {3.5504288721380934, -2.820888339674231, -2.3682490265219487, -1266.0131249531364};
        printf("\nTest cart_to_drive_with_vel_acc:\n");
        ret = cart_to_drive_with_vel_acc(&kin_model, pos, orient_angle, v, acc, q_inv, qd_inv, qdd_inv);
        printf("ret: %d\n", ret);
        printf("q_inv: %f %f %f %f\n", q_inv[0], q_inv[1], q_inv[2], q_inv[3]);
        printf("qd_inv: %f %f %f %f\n", qd_inv[0], qd_inv[1], qd_inv[2], qd_inv[3]);
        printf("qdd_inv: %f %f %f %f\n", qdd_inv[0], qdd_inv[1], qdd_inv[2], qdd_inv[3]);
        // for (int i=0; i < 4; i++)
        //         assert(fabs(q_inv[i] - q[i]) < 1e-10);
        // for (int i=0; i < 3; i++)
        //         assert(fabs(qd_inv[i] - qd_inv_py[i]) < 1e-10);
        // assert(fabs(qd_inv[3] - qd_inv_py[3]) < 1e-6);
        // for (int i=0; i < 3; i++)
        //         assert(fabs(qdd_inv[i] - qdd_inv_py[i]) < 1e-4);
        // assert(fabs(qdd_inv[3] - qdd_inv_py[3]) < 10.0);

        return 0;
}
