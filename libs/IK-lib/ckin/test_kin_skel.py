#!/usr/bin/python3.6

import unittest
import copy
import numpy as np
from collections import defaultdict

import cb.kin.model_lpoe
import cb.test

import lpoe_wrapper
import kin_wrapper


def D2R(val):
    return np.pi*val/180.0


class TestKinSkel(cb.test.CBTestCase):
    JVAL = np.array([
        [0.0, 0.0, 0.0, 0.0, 0.2, 0.0],
        [0.1, 0.0, 0.0, 0.0, 0.2, 0.0],
        [0.0, 0.1, 0.0, 0.0, 0.2, 0.0],
        [0.0, 0.0, 0.1, 0.0, 0.2, 0.0],
        [0.0, 0.0, 0.0, 0.1, 0.2, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.2, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.2, 0.1],
        [0.1, 0.1, 0.1, 0.1, 0.2, 0.1],
        [-1.0, -0.8, -0.8, -1.0, -1.0, -1.0],
        [1.0, -0.8, 0.8, -1.0, 1.0, -1.0],
        [0.3, 0.6, 0.8, 1.2, 1.5, 1.8],
        [-0.4, 0.8, -0.4, 0.4, -0.8, 1.3],
        [2.4, 3.2, 0.34, -0.51, 0.43, 1.5],
        [2.5, -0.32, 3.44, 2.3, 0.0, 4.5],
    ])

    BP_ERR = np.array([[0.001, 0.0005, -0.0007, 0.0002, 0.003, -0.001],
                       [0.002, 0.001, 0.0007, 0.0003, 0.002, 0.001],
                       [0.0, -0.0005, -0.001, 0.0002, 0.001, -0.001],
                       [-0.001, -0.0005, -0.0007, 0.0001, 0.0, 0.001],
                       [-0.002, 0.0, 0.0005, 0.0002, -0.003, -0.001],
                       [-0.001, 0.002, -0.002, -0.0002, -0.002, 0.001],
                       [0.001, 0.001, -0.0007, -0.001, -0.001, 0.0005]])

    def compare(self, robot, qs):
        robot_id = copy.deepcopy(robot)
        for k in range(len(robot_id.trans)):
            diff_t44 = cb.tfm.basepar_to_t44(self.BP_ERR[k, :])
            new_t = np.array(robot_id.trans[k], dtype=float).dot(diff_t44)
            robot_id._trans[k] = new_t.tolist()

        for k in range(qs.shape[0]):
            q = qs[k, :]
            if q[4] == 0:
                continue

            # Fwd
            py_pose = robot_id.fwd(q)
            pose = lpoe_wrapper.fwd(robot_id, q)
            np.testing.assert_allclose(
                pose,
                py_pose,
                atol=1e-15,
                err_msg="diff[%d]: %s" % (k, pose-py_pose))

            # Inv
            q_inv = kin_wrapper.inv(robot_id, pose, q_start=q+.2)
            np.testing.assert_allclose(
                q_inv,
                q,
                atol=1e-6,
                err_msg="q_diff[%d]: %s" % (k, q_inv-q),
            )

    def test_er6(self):
        self.compare(cb.kin.model_lpoe.ER6(), self.JVAL)

    def test_140(self):
        self.compare(cb.kin.model_lpoe.IRB140_B(), self.JVAL)

    def test_kr(self):
        self.compare(
            cb.kin.model_lpoe.KR20R1810(),
            self.JVAL + [0.0, -np.pi/2, np.pi/2, 0.0, 0.0, 0.0],
        )

    def test_irb140_from_c(self):
        target_thetas = np.array(
            [[0.1, 0, 0, 0, 0, 0],
             [0, 0, 0, 0.1, 0.1, 0.1],
             [0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
             [-0.1, -0.1, -0.1, -0.1, -0.1, -0.1],
             [D2R(10), 0, 0, D2R(30), D2R(10), D2R(-30)],
             [D2R(-10), 0, 0, D2R(-30), D2R(-10), D2R(30)],
             [D2R(5), D2R(5), D2R(5), D2R(5), D2R(5), D2R(5)],
             [0.082538, 0.526414, 0.030095, 0.062925, -1.531567, -2.150944],
             [0.503769, -0.283855, 0.229515, -3.045179, -1.655626, -6.555971],
             [0.500917, -0.349711, 0.238076, -3.094336, -1.473717, -12.152558],
             [0.571091, -0.350559, 0.245613, -2.771367, -1.528575, -12.874756],
             [0.499429, -0.354947, 0.239953, -3.095857, -1.458156,
              -18.431167]])
        rob = cb.kin.model_lpoe.IRB140_B()
        lpoe_s = np.array(rob.s, dtype=float)
        lpoe_trans = np.array(rob.trans, dtype=float)
        for k in range(target_thetas.shape[0]):
            th = target_thetas[k, :]
            target_pose = cb.num.kin.np_lpoe_fwd(lpoe_s, lpoe_trans, th)
            th_ret = kin_wrapper.inv(rob, target_pose)
            th_ret = np.array(th_ret)
            pose_ret = cb.num.kin.np_lpoe_fwd(lpoe_s, lpoe_trans, th_ret)
            np.testing.assert_allclose(pose_ret, pose_ret, atol=1e-7)

    def compare2(self, robot, bp_err=None, use_nom_for_init=False):
        lpoe_s = np.array(robot.s, dtype=float)
        lpoe_trans_nom = np.array(robot.trans, dtype=float)
        lpoe_trans = np.copy(lpoe_trans_nom)
        if bp_err is not None:
            for k in range(bp_err.shape[0]):
                diff_t44 = cb.tfm.basepar_to_t44(bp_err[k, :])
                lpoe_trans[k, :, :] = np.dot(diff_t44, lpoe_trans_nom[k, :, :])
        rob = copy.deepcopy(robot)
        rob._trans = lpoe_trans.tolist()
        rob._S = lpoe_s.tolist()
        for k in range(self.JVAL.shape[0]):
            th = self.JVAL[k, :]
            print(th)
            target_pose = cb.num.kin.np_lpoe_fwd(lpoe_s, lpoe_trans, th)
            th_init = None
            if use_nom_for_init:
                try:
                    th_init = robot.nom_inv(target_pose, lim=False).tolist()
                except:
                    pass
            th_ret = kin_wrapper.inv(rob, target_pose, q_start=th_init)
            th_ret = np.array(th_ret)
            # self.assertTrue(th_ret is not None)
            # self.assertTrue(stats['iter'] < 15)
            # print(stats)
            pose_ret = cb.num.kin.np_lpoe_fwd(lpoe_s, lpoe_trans, th_ret)
            np.testing.assert_allclose(pose_ret, target_pose, atol=1e-6)
            print(np.linalg.norm(pose_ret - target_pose))

    def test_er6_nom(self):
        self.compare2(cb.kin.model_lpoe.ER6())

    def test_er6_skew(self):
        self.compare2(cb.kin.model_lpoe.ER6(), bp_err=self.BP_ERR)

    def test_140_nom(self):
        self.compare2(cb.kin.model_lpoe.IRB140_B())

    def test_140_skew(self):
        self.compare2(cb.kin.model_lpoe.IRB140_B(), bp_err=self.BP_ERR)

    def test_kr20r1810_nom(self):
        self.compare2(cb.kin.model_lpoe.KR20R1810())

    def test_kr20r1810_skew(self):
        self.compare2(cb.kin.model_lpoe.KR20R1810(), bp_err=self.BP_ERR)

    def test_racer7_nom(self):
        self.compare2(cb.kin.model_lpoe.Racer71_4())

    def test_racer7_skew(self):
        self.compare2(cb.kin.model_lpoe.Racer71_4(), bp_err=self.BP_ERR)


if __name__ == '__main__':
    unittest.main()
