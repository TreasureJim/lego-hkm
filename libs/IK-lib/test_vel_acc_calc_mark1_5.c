#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "kinematics.h"
#include "mark1_5.h"

int main(void)
{

        double q[4] = {1.0, 2.0 - M_PI_2, 0.2, 2.0};
        int ret;
        res_q_extra_t res_q_extra;
        ax4_fwd_ret_t ax4_fwd_ret;
        double orient_angle;
        double out_tcp[4][4];
        double out_elbow[4][4];
        double j23[4][2];
        double j23_true[4][2] = {
                        {-0.12224891169678298, 0.11962519215693682},
                        {-0.9898194485759298, -0.20542008773580364},
                        {-0.08125957328003991, 0.7778036217666159},
                        {1.0545513314822628, 0.2023107526367022}};

        ret = drive_to_cart(&mark1_5, q, out_tcp, &orient_angle);
        printf("ret: %d\n", ret);
        assert(ret == 0);

        double pos[3] = {out_tcp[0][3], out_tcp[1][3], out_tcp[2][3]};
        double v[4] = {0.5, 0.5, 0.2, 23.0};
        double acc[4] = {-1.0, 1.0, -1.0, 1.0};
        double q_inv[4];
        double qd_inv[4];
        double qdd_inv[4];
        double qd_inv_py[4] = {-0.4981968123675202, -0.3784202279504486, 0.2175995701483059, 23.853237634802362};
        double qdd_inv_py[4] = {1.8718015730466673, -1.7036627572262935, -1.502219390738326, 1.20060641220924324};
        printf("\nTest inv_nom_with_vel_acc:\n");
        ret = cart_to_drive_with_vel_acc(&mark1_5, pos, orient_angle, v, acc, q_inv, qd_inv, qdd_inv);
        printf("ret: %d\n", ret);
        printf("q_inv: %f %f %f %f\n", q_inv[0], q_inv[1], q_inv[2], q_inv[3]);
        printf("qd_inv: %f %f %f %f\n", qd_inv[0], qd_inv[1], qd_inv[2], qd_inv[3]);
        printf("qdd_inv: %f %f %f %f\n", qdd_inv[0], qdd_inv[1], qdd_inv[2], qdd_inv[3]);
        for (int i = 0; i < 4; i++)
                assert(fabs(q_inv[i] - q[i]) < 1e-10);
        for (int i = 0; i < 3; i++)
                assert(fabs(qd_inv[i] - qd_inv_py[i]) < 1e-10);
        assert(fabs(qd_inv[3] - qd_inv_py[3]) < 1e-6);
        for (int i = 0; i < 3; i++)
                assert(fabs(qdd_inv[i] - qdd_inv_py[i]) < 1e-4);
        assert(fabs(qdd_inv[3] - qdd_inv_py[3]) < 10.0);

        return 0;
}
