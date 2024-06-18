#!/usr/bin/env python
#
# Copyright (C) 2017, 2018, 2019 Cognibotics
#
# Use or copying of this file or any part of it, without explicit
# permission from Cognibotics is not allowed in any way.

from __future__ import absolute_import, print_function

import argparse
import collections
import copy
import ctypes
from enum import Enum
import os.path
import sys

try:
    from functools import lru_cache
except ImportError:
    from backports.functools_lru_cache import lru_cache

try:
    import cb.kin.model_lpoe
    cblib_available = True
except ImportError:
    cblib_available = False

_MOD_DIR = os.path.dirname(sys.modules[__name__].__file__)
_LIB = None


class RobType(Enum):
    STANDARD_ROBOT = 0
    HOLLOW_WRIST = 1
    UR = 2
    GOFA = 3


def _load():
    global _LIB
    if _LIB is None:
        lib_name = os.path.join(_MOD_DIR, 'liblpoe.so')
        if os.path.isfile(lib_name):
            _LIB = ctypes.CDLL(lib_name)
        else:
            raise RuntimeError('Failed to find shared library')


class CLpoeParamsLink(ctypes.Structure):
    """Single link attached to (optional) joint."""
    _fields_ = [
        ('s', (ctypes.c_double * 6)),
        ('trans', (ctypes.c_double * 4 * 4)),
    ]


class CLpoeModel(ctypes.Structure):
    _fields_ = [
        ('n_screw', ctypes.c_int),
        ('lrob', ctypes.POINTER(CLpoeParamsLink)),
        ('has_pbar', ctypes.c_int),
        ('par_fact', ctypes.c_double),
        ('n_joints', ctypes.c_int),
        ('coupling_matrix', ctypes.POINTER(ctypes.c_double)),
        ('inverse_coupling_matrix', ctypes.POINTER(ctypes.c_double)),
        ('q_a_offs', ctypes.POINTER(ctypes.c_double)),
        ('lrob_pbar', ctypes.POINTER(CLpoeParamsLink)),
        ('handle_as_pbar', ctypes.c_int),
        ('robtype', ctypes.c_int)
    ]


def convert_robdef(rob):
    if isinstance(rob, collections.Hashable):
        return _convert_robdef(rob)
    return _convert_robdef.__wrapped__(rob)


@lru_cache()
def _convert_robdef(rob):
    return convert_robdef2(rob.s, rob.trans)


def convert_robdef2(s, trans):
    c_rob = (CLpoeParamsLink * len(s))()
    for i in range(len(s)):
        for j in range(6):
            c_rob[i].s[j] = s[i][j]
    for i in range(len(s)):
        for j in range(4):
            for k in range(4):
                c_rob[i].trans[j][k] = trans[i][j][k]
    return c_rob


def convert_robdef_pbar(rob):
    n = int(rob._PARALLEL_BAR[3])
    return convert_robdef_pbar2(n, rob._S_PARALLEL, rob._TRANS_PARALLEL)


def convert_robdef_pbar2(n_par, s_par, trans_par):
    c_rob_pbar = (CLpoeParamsLink * n_par)()
    for i in range(n_par):
        for j in range(6):
            c_rob_pbar[i].s[j] = float(s_par[i][j])
        for j in range(4):
            for k in range(4):
                c_rob_pbar[i].trans[j][k] = float(trans_par[i][j][k])
    return c_rob_pbar


def convert_robdef_extended(rob, dq=None):
    n_pbar = 0 if not rob.has_parallel_bar else int(rob._PARALLEL_BAR[3])
    s_pbar = None if n_pbar == 0 else rob._S_PARALLEL
    tr_pbar = None if n_pbar == 0 else rob._TRANS_PARALLEL
    dq_ = dq if dq is not None else rob.moffs
    robtype = RobType.STANDARD_ROBOT
    if cblib_available:
        if isinstance(rob, cb.kin.model_lpoe.ComauHollowWrist):
            robtype = RobType.HOLLOW_WRIST
        elif isinstance(rob, cb.kin.model_lpoe.UR):
            robtype = RobType.UR
        elif isinstance(rob, cb.kin.model_lpoe.GoFa):
            robtype = RobType.GOFA
    return convert_robdef_extended2(
        rob.s, rob.trans, len(rob), dq_, rob.has_parallel_bar,
        rob.handle_as_par_bar_robot, rob.par_fact, rob.coupling_matrix,
        rob.inverse_coupling_matrix, n_pbar, s_pbar, tr_pbar, robtype)


def convert_robdef_extended2(
        s, trans, n_joints, dq=None, has_parallel_bar=False,
        handle_as_par_bar_robot=False, par_fact=1.0, coupling_mat=None,
        inverse_coupling_mat=None, n_par=0, s_par=None, trans_par=None,
        robtype=RobType.STANDARD_ROBOT):
    c_model = CLpoeModel()
    c_model.lrob = convert_robdef2(s, trans)
    c_model.n_screw = len(s)
    c_model.has_pbar = has_parallel_bar
    c_model.handle_as_pbar = handle_as_par_bar_robot
    c_model.par_fact = par_fact
    c_model.n_joints = n_joints
    if coupling_mat is None or inverse_coupling_mat is None:
        c_model.coupling_matrix = None
        c_model.inverse_coupling_matrix = None
    else:
        coup_mat_vec = coupling_mat.reshape((-1,))
        n_coup_mat = coup_mat_vec.shape[0]
        c_coup_mat = (ctypes.c_double * n_coup_mat)(*coup_mat_vec)
        inv_coup_mat_vec = inverse_coupling_mat.reshape((-1,))
        c_inv_coup_mat = (ctypes.c_double * n_coup_mat)(*inv_coup_mat_vec)
        c_double_p = ctypes.POINTER(ctypes.c_double)
        c_model.coupling_matrix = ctypes.cast(c_coup_mat, c_double_p)
        c_model.inverse_coupling_matrix = ctypes.cast(c_inv_coup_mat, c_double_p)
    if dq is not None:
        c_dq = (ctypes.c_double * n_joints)(*dq)
        c_model.q_a_offs = ctypes.cast(c_dq, ctypes.POINTER(ctypes.c_double))
    else:
        c_model.q_a_offs = None
    if has_parallel_bar and n_par > 0:
        c_model.lrob_pbar = convert_robdef_pbar2(n_par, s_par, trans_par)
    else:
        c_model.lrob_pbar = None
    c_model.robtype = robtype.value
    return c_model


def ctypes_matrix(vals):
    res = (ctypes.POINTER(ctypes.c_double) * len(vals))()
    for i, val in enumerate(vals):
        res[i] = (ctypes.c_double * len(val))()
        for j, v in enumerate(val):
            res[i][j] = v
    return res


def fwd(robot_lpoe, q=None, end_link=None, dq=None):
    if q is None:
        q = [0] * len(robot_lpoe)
    if end_link is None:
        end_link = len(robot_lpoe) + 1
    c_q = (ctypes.c_double * len(robot_lpoe))(*q)
    c_lpoe = convert_robdef_extended(robot_lpoe, dq=dq)

    c_el = (ctypes.c_int)(end_link)
    c_tf = (ctypes.c_double * 4 * 4 * end_link)()
    _load()
    _LIB.fwd_lpoe(ctypes.byref(c_lpoe), c_q, c_el, ctypes.byref(c_tf))
    res = []
    for i in range(4):
        tmp = []
        for j in range(4):
            tmp.append(c_tf[-1][i][j])
        res.append(tmp)
    return res


def fwd_list(robot_lpoe, q_list=None, end_link=None, dq=None):
    if q_list is None:
        q_list = [0] * len(robot_lpoe)
    if end_link is None:
        end_link = len(robot_lpoe) + 1

    c_q_list = ctypes_matrix(q_list)
    n_samp = len(q_list)
    c_lpoe = convert_robdef_extended(robot_lpoe, dq=dq)
    c_el = (ctypes.c_int)(end_link)
    c_pose = (ctypes.c_double * 4 * 4 * n_samp)()

    _load()
    _LIB.fwd_lpoe_list(ctypes.byref(c_lpoe), c_q_list, end_link, n_samp, c_pose)

    res_samp = []
    for i_samp in range(n_samp):
        res = []
        for i in range(4):
            tmp = []
            for j in range(4):
                tmp.append(c_pose[i_samp][i][j])
            res.append(tmp)
        res_samp.append(res)

    return res_samp


def jacobian_jdot(robot_lpoe, q, qd):
    n = len(robot_lpoe)
    n_screws = n+1
    c_q = (ctypes.c_double * n)(*q)
    c_qd = (ctypes.c_double * n)(*qd)
    c_lpoe_link = convert_robdef(robot_lpoe)

    c_tf = (ctypes.c_double * 4 * 4)()
    c_jac = (ctypes.c_double * n * 6)()
    c_jdot = (ctypes.c_double * n * 6)()
    _load()
    _LIB.jacobian_jdot(ctypes.byref(c_lpoe_link), n_screws, n, c_q, c_qd,
                       ctypes.byref(c_tf), ctypes.byref(c_jac), ctypes.byref(c_jdot))
    res_jac = []
    res_jdot = []
    for i in range(6):
        tmp_jac = []
        tmp_jdot = []
        for j in range(n):
            tmp_jac.append(c_jac[i][j])
            tmp_jdot.append(c_jdot[i][j])
        res_jac.append(tmp_jac)
        res_jdot.append(tmp_jdot)
    return res_jac, res_jdot


def jdot_spec(robot_lpoe, q, qd):
    n_joints = len(robot_lpoe)
    n_screws = len(robot_lpoe.s)
    c_q = (ctypes.c_double * n_joints)(*q)
    c_qd = (ctypes.c_double * n_joints)(*qd)
    c_lpoe_link = convert_robdef(robot_lpoe)

    c_jdot = (ctypes.c_double * n_joints * 6)()
    c_jdot2 = (ctypes.c_double * n_joints * 6)()
    c_jddvec = (ctypes.c_double * 6)()
    _load()
    _LIB.jdot_spec(ctypes.byref(c_lpoe_link), n_screws, n_joints, c_q, c_qd,
                       ctypes.byref(c_jdot), ctypes.byref(c_jdot2), ctypes.byref(c_jddvec))
    res_jdot = []
    res_jdot2 = []
    res_jddvec = []
    for i in range(6):
        tmp_jdot = []
        tmp_jdot2 = []
        for j in range(n_joints):
            tmp_jdot.append(c_jdot[i][j])
            tmp_jdot2.append(c_jdot2[i][j])
        res_jdot.append(tmp_jdot)
        res_jdot2.append(tmp_jdot2)
        res_jddvec.append(c_jddvec[i])
    return res_jdot, res_jdot2, res_jddvec


def main(argv):
    import numpy as np
    import numpy.testing
    import cb.kin.model_lpoe
    import cb.tfm
    from cb.pprint import TimedPrinter

    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default='IRB140_B', help="Name of robot model.")
    args = ap.parse_args(argv)

    rob = getattr(cb.kin.model_lpoe, args.model)()
    n_samp = 10000
    q = [
        [0] * 6,
        [10] * 6,
        [20] * 6,
        [30] * 6,
        [40] * 6,
    ]
    q2 = [copy.deepcopy(q) for _ in range(n_samp // len(q))]
    q = sum(q2, [])

    # Loop through jpos in Python.
    with TimedPrinter('Calling fwd()      on %d points' % len(q)):
        s_res = []
        for i_samp in range(len(q)):
            s_res.append(np.array(fwd(rob, q[i_samp])))

    # Loop through jpos in C.
    with TimedPrinter('Calling fwd_list() on %d points' % len(q)):
        m_res = fwd_list(rob, q)

    # Compare results
    assert len(s_res) == len(m_res)
    for i in range(len(s_res)):
        numpy.testing.assert_allclose(
            s_res[i], m_res[i], err_msg="First failing pos: q[%d]" % i)
    print('Results are equal')

    # Test parallel bar robot
    irb2400 = cb.kin.model_lpoe.IRB2400_16()
    cm = irb2400.coupling_matrix
    print("coupling_mat1:\n", cm)
    q = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    fk_py = irb2400.fwd(q)
    fk_c1 = np.array(fwd(irb2400, q))
    cm_ = np.eye(6)
    cm_[2, 1] = -1.0
    irb2400._COUPLING_MATRIX = cm_.tolist()
    cm2 = irb2400.coupling_matrix
    print("coupling_mat2:\n", cm2)
    fk_c2 = np.array(fwd(irb2400, q))
    print("python:\n", fk_py)
    print("C1 (no coupling matrix):\n", fk_c1)
    print("C2 (with coupling matrix):\n", fk_c2)
    print("diff py-C1:", np.linalg.norm(cb.tfm.tr2diff(fk_py, fk_c1)))
    print("diff py-C2:", np.linalg.norm(cb.tfm.tr2diff(fk_py, fk_c2)))

    # Test offsets
    er6 = cb.kin.model_lpoe.ER6()
    q = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    dq = np.ones((6,))*0.1
    fk1 = np.array(fwd(rob, q))
    fk2 = np.array(fwd(rob, q-dq, dq=dq))
    print("difference:", np.linalg.norm(fk1-fk2))

    robot = cb.kin.model_lpoe.UR5e()
    q = [.1, .2, .3, .4, .5, .6]
    qd = [1., 1., 1., 1., 1., 1.]
    jac, jdot = jacobian_jdot(robot, q, qd)
    print('jac:\n', np.array(jac), '\njdot:\n', np.array(jdot))
    jdot_v2, jdot2, jddvec = jdot_spec(robot, q, qd)
    print('jdot_v2:\n', np.array(jdot_v2), '\njdot2:\n', np.array(jdot2))
    print('jddvec:', np.array(jddvec))

    import cb.num.kin
    np_s = np.array(robot.s, dtype=float)
    np_trans = np.array(robot.trans, dtype=float)
    jac2 = cb.num.kin.np_lpoe_body_jacobian(np_s, np_trans, np.array(q), in_base_frame=True)
    diff = np.array(jac) - jac2
    assert np.max(np.abs(diff) < 1e-14)

    jdot_py, jdot2_py, jddvec_py = cb.num.kin.jdot_spec(np_s, np_trans, np.array(q), np.array(qd))
    assert np.linalg.norm(jdot - jdot_py) < 1e-12
    assert np.linalg.norm(jdot_v2 - jdot_py) < 1e-12
    assert np.linalg.norm(jdot2 - jdot2_py) < 1e-12
    assert np.linalg.norm(jddvec - jddvec_py) < 1e-12


if __name__ == '__main__':
    main(sys.argv[1:])
