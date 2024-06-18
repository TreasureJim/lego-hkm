#!/usr/bin/python2.7
#
# Copyright (C) 2018, 2019 Cognibotics
#
# Use or copying of this file or any part of it, without explicit
# permission from Cognibotics is not allowed in any way.

import ctypes
import os.path
import sys

import lpoe_wrapper

MOD_DIR = os.path.dirname(sys.modules[__name__].__file__)

lib_name = os.path.join(MOD_DIR, 'libkin.so')
if os.path.exists(lib_name):
    LIB = ctypes.CDLL(lib_name)
else:
    raise RuntimeError('Failed to find shared library')


class UnreachableException(Exception):
    pass


class CLpoeNom(ctypes.Structure):
    """Structure with nominal lpoe parameters.
    """
    _fields_ = [
        ('model', lpoe_wrapper.CLpoeModel),
        ('rot_dirs', ctypes.c_double * 6),
        ('wcp0', ctypes.c_double * 4 * 4),
        ('l1', ctypes.c_double),
        ('l2', ctypes.c_double),
        ('th1_0', ctypes.c_double),
        ('th2_0', ctypes.c_double),
        ('hollow_wrist', ctypes.c_int),
    ]


def convert_nom_lpoe(lrob_nom):
    import cb.codegen.ne
    rob_name = lrob_nom.__class__.__name__
    pars = cb.codegen.ne.gen_nominal_robdef('', False, rob_name, ret_pars=True)
    if len(pars) == 8:
        # Workaround for old cblib.
        pars = list(pars) + [False]
    s, trans, rot_dirs, wcp0, l1, l2, th1_0, th2_0, hw = pars

    n = len(lrob_nom)
    r = CLpoeNom()
    r.model = lpoe_wrapper.convert_robdef_extended(lrob_nom)
    r.rot_dirs = (ctypes.c_double * n)(*rot_dirs)
    r.l1 = l1
    r.l2 = l2
    r.th1_0 = th1_0
    r.th2_0 = th2_0
    r.hollow_wrist = hw

    return r


def nom_inv(lrob, pose, overhead_pos=False, elbow_down=False, neg_a5=False):
    n = len(lrob)
    c_lrob_nom = convert_nom_lpoe(lrob)

    c_pose = (ctypes.c_double * 4 * 4)()
    for i in range(4):
        for j in range(4):
            c_pose[i][j] = pose[i][j]
    c_q = (ctypes.c_double * n)()

    ret = LIB.nom_inv(ctypes.byref(c_lrob_nom), c_pose, overhead_pos, elbow_down, neg_a5, c_q)
    if ret:
        raise UnreachableException("Unreachable")
    return [c_q[i] for i in range(n)]


def nom_inv_hw(lrob, pose, overhead_pos=False, elbow_down=False, neg_a5=False,
               tol=1e-6, initial_tol=0.01, q0=None):
    n = len(lrob)
    c_lrob_nom = convert_nom_lpoe(lrob)

    c_pose = (ctypes.c_double * 4 * 4)()
    for i in range(4):
        for j in range(4):
            c_pose[i][j] = pose[i][j]
    c_q = (ctypes.c_double * n)()
    c_tol = ctypes.c_double(tol)
    c_initial_tol = ctypes.c_double(initial_tol)
    c_q0 = (ctypes.c_double * n)()
    if q0 is None:
        for k in range(6):
            c_q0[k] = 0.0
    else:
        for k in range(6):
            c_q0[k] = q0[k]

    ret = LIB.nom_inv_hw(ctypes.byref(c_lrob_nom), c_pose, overhead_pos,
                         elbow_down, neg_a5, c_q0, c_tol, c_initial_tol, c_q)
    if ret:
        raise UnreachableException("Unreachable")
    return [c_q[i] for i in range(n)]


def inv(lrob, pose, q_start=None):
    n = len(lrob)

    if q_start is None:
        q_start = [0] * n

    c_lrob = lpoe_wrapper.convert_robdef_extended(lrob)

    c_pose = (ctypes.c_double * 4 * 4)()
    for i in range(4):
        for j in range(4):
            c_pose[i][j] = pose[i][j]

    c_q = (ctypes.c_double * n)()
    c_q_start = (ctypes.c_double * n)(*q_start)

    ret = LIB.kin_skel_inv(
        ctypes.c_ulong(n),
        ctypes.byref(c_lrob),
        c_pose,
        c_q_start,
        ctypes.c_ulong(0),
        ctypes.c_double(0.0),
        ctypes.c_double(0.0),
        c_q,
        None,
    )
    if ret:
        raise Exception("Failed to converge")

    return [c_q[i] for i in range(n)]


def main():
    import cb.kin.model_lpoe
    rob = cb.kin.model_lpoe.ER6()
    pose = rob.fwd([.1] * 6)
    print(pose)

    print("Nom")
    print(nom_inv(rob, pose))

    print("Iter")
    print(inv(rob, pose))


if __name__ == '__main__':
    main()
