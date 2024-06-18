#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "kinematics.h"
#include "kin_model_long.h"

int main(void)
{
        double pos[3] = {-0.944527339, 0.8684955767, -0.125531043};
        double orient_angle = -0.7853981633974483;
        double v[4] = {-0.0, 4.1955727185, -0.4267976723, 0.0};
        double acc[4] = {0.0, 0.0, 0.0, 0.0};
        double q_inv[4];
        double qd_inv[4];
        double qdd_inv[4];
        int ret;
        double q_inv_py[4] = {1.6605101595584344, 1.5836010069720858 - M_PI_2, -0.06693901389668527, 46.94775697477837};
        double qd_inv_py[4] = {-0.4093309387854297, -4.0236195902689005, -0.46979947170761704, -42.42136130248197};
        double qdd_inv_py[4] = {19.556313270825427, -16.92371082470416, -1.0260075669064892, -393.5943120602553};
        printf("\nTest inv_nom_with_vel_acc:\n");
        ret = cart_to_drive_with_vel_acc_num(&kin_model_long, pos, orient_angle, v, acc, q_inv, qd_inv, qdd_inv);
        printf("ret: %d\n", ret);
        printf("q_inv: %f %f %f %f\n", q_inv[0], q_inv[1], q_inv[2], q_inv[3]);
        printf("qd_inv: %f %f %f %f\n", qd_inv[0], qd_inv[1], qd_inv[2], qd_inv[3]);
        printf("qdd_inv: %f %f %f %f\n", qdd_inv[0], qdd_inv[1], qdd_inv[2], qdd_inv[3]);
        // for (int i=0; i < 4; i++)
        //         assert(fabs(q_inv[i] - q_inv_py[i]) < 1e-10);
        // for (int i=0; i < 3; i++)
        //         assert(fabs(qd_inv[i] - qd_inv_py[i]) < 1e-10);
        // assert(fabs(qd_inv[3] - qd_inv_py[3]) < 1e-6);
        // for (int i=0; i < 3; i++)
        //         assert(fabs(qdd_inv[i] - qdd_inv_py[i]) < 1e-4);
        // assert(fabs(qdd_inv[3] - qdd_inv_py[3]) < 10.0);

        return 0;
}
