#!/usr/bin/env python
#
# Copyright (C) 2020 Cognibotics
#
# Use or copying of this file or any part of it, without explicit
# permission from Cognibotics is not allowed in any way.

from __future__ import absolute_import, print_function

import ctypes
import os.path
import sys

import numpy as np

_MOD_DIR = os.path.dirname(sys.modules[__name__].__file__)
_LIB = None


def _load():
    global _LIB
    if _LIB is None:
        lib_name = os.path.join(_MOD_DIR, 'libagile_all.so')
        if os.path.isfile(lib_name):
            _LIB = ctypes.CDLL(lib_name)
        else:
            raise RuntimeError('Failed to find shared library')


class CAgilePkmElbowBackhoe(ctypes.Structure):
    """Elbow backhoe structure."""
    _fields_ = [
        ('ax', (ctypes.c_double)),
        ('ay', (ctypes.c_double)),
        ('ah', (ctypes.c_double)),
        ('a0', (ctypes.c_double)),
        ('bs0', (ctypes.c_double)),
        ('La1', (ctypes.c_double)),
        ('La2', (ctypes.c_double)),
        ('a_ang', (ctypes.c_double)),
        ('d_ang', (ctypes.c_double)),
        ('Ld', (ctypes.c_double)),
        ('dx', (ctypes.c_double)),
        ('dy', (ctypes.c_double)),
        ('dxb', (ctypes.c_double)),
        ('Lc', (ctypes.c_double)),
        ('Lb1', (ctypes.c_double)),
        ('Lb2', (ctypes.c_double)),
        ('z_offs', (ctypes.c_double)),
    ]


class CAgilePkmWristBackhoe(ctypes.Structure):
    """Wrist backhoe structure."""
    _fields_ = [
        ('La1', (ctypes.c_double)),
        ('La2', (ctypes.c_double)),
        ('Lc1', (ctypes.c_double)),
        ('Lc2', (ctypes.c_double)),
        ('Lc3', (ctypes.c_double)),
        ('c_ang', (ctypes.c_double)),
        ('Ld', (ctypes.c_double)),
        ('Le1', (ctypes.c_double)),
        ('Le2', (ctypes.c_double)),
        ('e_ang', (ctypes.c_double)),
        ('Lf', (ctypes.c_double)),
        ('Lg', (ctypes.c_double)),
        ('Lh', (ctypes.c_double)),
        ('q4_offs', (ctypes.c_double)),
        ('alpha_c_pol', (ctypes.c_double * 2)),
        ('alpha_e_pol', (ctypes.c_double * 2)),
    ]


class CAgilePkmBackhoe(ctypes.Structure):
    """Backhoe structure."""
    _fields_ = [
        ('L_rod', (ctypes.c_double)),
        ('ebh', ctypes.POINTER(CAgilePkmElbowBackhoe)),
        ('wbh', ctypes.POINTER(CAgilePkmWristBackhoe)),
    ]


class CAgilePkmModel(ctypes.Structure):
    """Agile Pkm model structure."""
    _fields_ = [
        ('bh', ctypes.POINTER(CAgilePkmBackhoe)),
        ('L1', (ctypes.c_double)),
        ('L2', (ctypes.c_double)),
        ('L3', (ctypes.c_double)),
        ('L3b', (ctypes.c_double)),
        ('Lpar', (ctypes.c_double)),
        ('a2', (ctypes.c_double)),
        ('d2', (ctypes.c_double)),
        ('ax2x', (ctypes.c_double)),
        ('ax2y', (ctypes.c_double)),
        ('ax2z', (ctypes.c_double)),
        ('ax3y', (ctypes.c_double)),
        ('q2_offs', (ctypes.c_double)),
        ('joint_lims', (ctypes.c_double * 2 * 4)),
    ]


class CLpoeParamsLink(ctypes.Structure):
    """Single link attached to (optional) joint."""
    _fields_ = [
        ('s', (ctypes.c_double * 6)),
        ('trans', (ctypes.c_double * 4 * 4)),
    ]


class CAgilePkmLpoeModel(ctypes.Structure):
    """Agile PKM LPOE model."""
    _fields_ = [
        ('lrob', ctypes.POINTER(CLpoeParamsLink)),
        ('n_screws', (ctypes.c_int)),
        ('last_trans', (ctypes.c_double * 4 * 4 * 2)),
        ('lam', (ctypes.c_int * 9)),
        ('mu', (ctypes.c_int * 9)),
        ('mu_inds', (ctypes.c_int * 2 * 10)),
        ('tool_body', (ctypes.c_int)),
        ('chain_ends', (ctypes.c_int * 2)),
    ]


class CDynParamsLink(ctypes.Structure):
    """Dynamic parameters for a single link."""
    _fields_ = [
        ('m', (ctypes.c_double)),
        ('r', (ctypes.c_double * 3)),
        ('ival', (ctypes.c_double * 3 * 3)),
    ]


def _convert_dyn_params(n, d):
    c_dyn = (CDynParamsLink * (n+1))()
    for i in range(n+1):
        c_dyn[i].m = d.get('m%d' % i, 0)

        c_dyn[i].r[0] = d.get('r%d0' % i, 0)
        c_dyn[i].r[1] = d.get('r%d1' % i, 0)
        c_dyn[i].r[2] = d.get('r%d2' % i, 0)

        c_dyn[i].ival[0][0] = d.get('I%d0' % i, 0)
        c_dyn[i].ival[1][1] = d.get('I%d1' % i, 0)
        c_dyn[i].ival[2][2] = d.get('I%d2' % i, 0)

        c_dyn[i].ival[0][1] = d.get('I%d3' % i, 0)
        c_dyn[i].ival[0][2] = d.get('I%d4' % i, 0)
        c_dyn[i].ival[1][2] = d.get('I%d5' % i, 0)

        c_dyn[i].ival[1][0] = d.get('I%d3' % i, 0)
        c_dyn[i].ival[2][0] = d.get('I%d4' % i, 0)
        c_dyn[i].ival[2][1] = d.get('I%d5' % i, 0)

    return c_dyn


def convert_robdef(agile_pkm_rob):
    if agile_pkm_rob.bh is None:
        c_bh = None
    else:
        c_ebh = CAgilePkmElbowBackhoe()
        c_ebh.ax = agile_pkm_rob.bh.ebh.ax
        c_ebh.ay = agile_pkm_rob.bh.ebh.ay
        c_ebh.ah = agile_pkm_rob.bh.ebh.ah
        c_ebh.a0 = agile_pkm_rob.bh.ebh.a0
        c_ebh.bs0 = agile_pkm_rob.bh.ebh.bs0
        c_ebh.La1 = agile_pkm_rob.bh.ebh.La1
        c_ebh.La2 = agile_pkm_rob.bh.ebh.La2
        c_ebh.a_ang = agile_pkm_rob.bh.ebh.a_ang
        c_ebh.d_ang = agile_pkm_rob.bh.ebh.d_ang
        c_ebh.Ld = agile_pkm_rob.bh.ebh.Ld
        c_ebh.dx = agile_pkm_rob.bh.ebh.dx
        c_ebh.dy = agile_pkm_rob.bh.ebh.dy
        c_ebh.dxb = agile_pkm_rob.bh.ebh.dxb
        c_ebh.Lc = agile_pkm_rob.bh.ebh.Lc
        c_ebh.Lb1 = agile_pkm_rob.bh.ebh.Lb1
        c_ebh.Lb2 = agile_pkm_rob.bh.ebh.Lb2
        c_ebh.z_offs = agile_pkm_rob.bh.ebh.z_offs

        c_wbh = CAgilePkmWristBackhoe()
        c_wbh.La1 = agile_pkm_rob.bh.wbh.La1
        c_wbh.La2 = agile_pkm_rob.bh.wbh.La2
        c_wbh.Lc1 = agile_pkm_rob.bh.wbh.Lc1
        c_wbh.Lc2 = agile_pkm_rob.bh.wbh.Lc2
        c_wbh.Lc3 = agile_pkm_rob.bh.wbh.Lc3
        c_wbh.c_ang = agile_pkm_rob.bh.wbh.c_ang
        c_wbh.Ld = agile_pkm_rob.bh.wbh.Ld
        c_wbh.Le1 = agile_pkm_rob.bh.wbh.Le1
        c_wbh.Le2 = agile_pkm_rob.bh.wbh.Le2
        c_wbh.e_ang = agile_pkm_rob.bh.wbh.e_ang
        c_wbh.Lf = agile_pkm_rob.bh.wbh.Lf
        c_wbh.Lg = agile_pkm_rob.bh.wbh.Lg
        c_wbh.Lh = agile_pkm_rob.bh.wbh.Lh
        c_wbh.q4_offs = agile_pkm_rob.bh.wbh.q4_offs
        c_wbh.alpha_c_pol[0] = agile_pkm_rob.bh.wbh.alpha_c_pol[0]
        c_wbh.alpha_c_pol[1] = agile_pkm_rob.bh.wbh.alpha_c_pol[1]
        c_wbh.alpha_e_pol[0] = agile_pkm_rob.bh.wbh.alpha_e_pol[0]
        c_wbh.alpha_e_pol[1] = agile_pkm_rob.bh.wbh.alpha_e_pol[1]

        c_bh = CAgilePkmBackhoe()
        c_bh.L_rod = agile_pkm_rob.bh.L_rod
        c_bh.ebh = ctypes.pointer(c_ebh)
        c_bh.wbh = ctypes.pointer(c_wbh)

    c_rob = CAgilePkmModel()
    if c_bh is None:
        c_rob.bh = None
    else:
        c_rob.bh = ctypes.pointer(c_bh)
    c_rob.L1 = agile_pkm_rob.L1
    c_rob.L2 = agile_pkm_rob.L2
    c_rob.L3 = agile_pkm_rob.L3
    c_rob.L3b = agile_pkm_rob.L3b
    c_rob.Lpar = agile_pkm_rob.Lpar
    c_rob.a2 = agile_pkm_rob.a2
    c_rob.d2 = agile_pkm_rob.d2
    c_rob.ax2x = agile_pkm_rob.ax2x
    c_rob.ax2y = agile_pkm_rob.ax2y
    c_rob.ax2z = agile_pkm_rob.ax2z
    c_rob.ax3y = agile_pkm_rob.ax3y
    c_rob.q2_offs = agile_pkm_rob.q2_offs

    for k1 in range(4):
        for k2 in range(2):
            c_rob.joint_lims[k1][k2] = agile_pkm_rob.joint_lims[k1][k2]

    return c_rob


def convert_lpoe_def(lpoe_model):
    c_model = CAgilePkmLpoeModel()

    n_screws = lpoe_model.s.shape[0]
    c_lrob = (CLpoeParamsLink * n_screws)()
    for i in range(n_screws):
        for j in range(6):
            c_lrob[i].s[j] = lpoe_model.s[i, j]
        for j1 in range(4):
            for j2 in range(4):
                c_lrob[i].trans[j1][j2] = lpoe_model.trans[i, j1, j2]
                if i < 2:
                    c_model.last_trans[i][j1][j2] = lpoe_model.last_trans[i, j1, j2]
    c_model.lrob = c_lrob#ctypes.pointer(c_lrob)
    c_model.n_screws = n_screws

    for k in range(9):
        c_model.lam[k] = lpoe_model.lam[k]

    count = 0
    for i, _mu in enumerate(lpoe_model.mu):
        c_model.mu_inds[i][0] = count
        for mu_val in _mu:
            c_model.mu[count] = mu_val
            count += 1
        c_model.mu_inds[i][1] = count

    c_model.tool_body = lpoe_model.tool_body
    c_model.chain_ends[0] = lpoe_model.chain_ends[0]
    c_model.chain_ends[1] = lpoe_model.chain_ends[1]

    return c_model


def convert_dyn_params(n, d):
    c_dyn = (CDynParamsLink * (n+1))()
    for i in range(n+1):
        c_dyn[i].m = d.get('m%d' % i, 0)

        c_dyn[i].r[0] = d.get('r%d0' % i, 0)
        c_dyn[i].r[1] = d.get('r%d1' % i, 0)
        c_dyn[i].r[2] = d.get('r%d2' % i, 0)

        c_dyn[i].ival[0][0] = d.get('I%d0' % i, 0)
        c_dyn[i].ival[1][1] = d.get('I%d1' % i, 0)
        c_dyn[i].ival[2][2] = d.get('I%d2' % i, 0)

        c_dyn[i].ival[0][1] = d.get('I%d3' % i, 0)
        c_dyn[i].ival[0][2] = d.get('I%d4' % i, 0)
        c_dyn[i].ival[1][2] = d.get('I%d5' % i, 0)

        c_dyn[i].ival[1][0] = d.get('I%d3' % i, 0)
        c_dyn[i].ival[2][0] = d.get('I%d4' % i, 0)
        c_dyn[i].ival[2][1] = d.get('I%d5' % i, 0)

    return c_dyn


def convert_dyn_params2(ms, rcgs, ivals):
    n = ms.shape[0]
    c_dyn = (CDynParamsLink * n)()
    for i in range(n):
        c_dyn[i].m = ms[i]

        for j in range(3):
            c_dyn[i].r[j] = rcgs[i, j]
            for k in range(3):
                c_dyn[i].ival[j][k] = ivals[i, j, k]

    return c_dyn


def convert_tool_params(d):
    c_tool = CDynParamsLink()

    c_tool.m = d.get('m_tool', 0.0)
    c_tool.r[0] = d.get('r_tool0', 0.0)
    c_tool.r[1] = d.get('r_tool1', 0.0)
    c_tool.r[2] = d.get('r_tool2', 0.0)

    c_tool.ival[0][0] = d.get('I_tool0', 0.0)
    c_tool.ival[1][1] = d.get('I_tool1', 0.0)
    c_tool.ival[2][2] = d.get('I_tool2', 0.0)

    c_tool.ival[0][1] = d.get('I_tool3', 0.0)
    c_tool.ival[0][2] = d.get('I_tool4', 0.0)
    c_tool.ival[1][2] = d.get('I_tool5', 0.0)

    c_tool.ival[1][0] = d.get('I_tool3', 0.0)
    c_tool.ival[2][0] = d.get('I_tool4', 0.0)
    c_tool.ival[2][1] = d.get('I_tool5', 0.0)

    return c_tool


def convert_tool_params2(m, r, ival):
    c_tool = CDynParamsLink()
    c_tool.m = m
    for j in range(3):
        c_tool.r[j] = r[j]
        for k in range(3):
            c_tool.ival[j][k] = ival[j, k]
    return c_tool


def dyn_inv_agile(c_kin_model, c_lpoe_model, c_dyn_pars, c_tool_dyn_pars,
                  q, qd, qdd, g, full_model):

    c_q = (ctypes.c_double * 4)(*q)
    c_qd = (ctypes.c_double * 4)(*qd)
    c_qdd = (ctypes.c_double * 4)(*qdd)
    c_g = (ctypes.c_double * 3)(*g)
    c_full_model = (ctypes.c_int)(full_model)

    c_trq = (ctypes.c_double * 4)()

    _load()
    ret = _LIB.dyn_inv_agile(
        ctypes.byref(c_kin_model),
        ctypes.byref(c_lpoe_model),
        ctypes.byref(c_dyn_pars),
        ctypes.byref(c_tool_dyn_pars),
        c_q,
        c_qd,
        c_qdd,
        c_g,
        c_full_model,
        ctypes.byref(c_trq))

    if ret != 0:
        return [None]*4

    trq = []
    for k in range(4):
        trq.append(c_trq[k])

    return trq


def dyn_inv_agile_all(kin_model, lpoe_model, ms, rs, ivals, qs, qds, qdds, g,
                      full_model):
    c_kin_model = convert_robdef(kin_model)
    c_lpoe_model = convert_lpoe_def(lpoe_model)

    c_dyn_pars = convert_dyn_params2(ms, rs, ivals)
    c_tool_dyn_pars = convert_tool_params2(0, np.zeros((3,)), np.zeros((3, 3)))

    trqs = []
    for q, qd, qdd in zip(qs, qds, qdds):
        trqs.append(dyn_inv_agile(c_kin_model, c_lpoe_model, c_dyn_pars,
                                  c_tool_dyn_pars, q, qd, qdd, g, full_model))

    return trqs


def inertia_matrix(c_kin_model, c_lpoe_model, c_dyn_pars, c_tool_dyn_pars, q):

    c_q = (ctypes.c_double * 4)(*q)

    c_imat = (ctypes.c_double * 4 * 4)()

    _load()
    ret = _LIB.inertia_matrix(
        ctypes.byref(c_kin_model),
        ctypes.byref(c_lpoe_model),
        ctypes.byref(c_dyn_pars),
        ctypes.byref(c_tool_dyn_pars),
        c_q,
        ctypes.byref(c_imat))

    if ret != 0:
        return [[None]*4]*4

    imat = []
    for k in range(4):
        tmp = []
        for j in range(4):
            tmp.append(c_imat[k][j])
        imat.append(tmp)

    return imat


def inv_with_vel_acc(c_kin_model, pos, ori, v, a):
    c_pos = (ctypes.c_double * 3)(*pos)
    c_ori = (ctypes.c_double)(ori)
    c_v = (ctypes.c_double * 4)(*v)
    c_a = (ctypes.c_double * 4)(*a)

    c_q = (ctypes.c_double * 4)()
    c_qd = (ctypes.c_double * 4)()
    c_qdd = (ctypes.c_double * 4)()

    _load()
    ret = _LIB.inv_with_vel_acc(
        ctypes.byref(c_kin_model),
        c_pos,
        c_ori,
        c_v,
        c_a,
        ctypes.byref(c_q),
        ctypes.byref(c_qd),
        ctypes.byref(c_qdd))

    if ret != 0:
        return [None]*4, [None]*4, [None]*4

    q, qd, qdd = [], [], []
    for k in range(4):
        q.append(c_q[k])
        qd.append(c_qd[k])
        qdd.append(c_qdd[k])

    return q, qd, qdd


def main():
    import agile_pkm_model
    import dyn

    import numpy as np

    kin_model = agile_pkm_model.ConceptPrototype()
    lpoe_model = dyn.AgileRobModel(kin_model)

    c_kin_model = convert_robdef(kin_model)
    c_lpoe_model = convert_lpoe_def(lpoe_model)

    # Parameters for tool/load
    m_load = 4.0
    rcg_load = np.array([0.0, 0.0, 0.0])
    i_load = np.zeros((3, 3))

    # Dynamic parameters for robot according to document
    m_link1 = 25.73
    m_forearms = 1.919
    m_wristassembly = 2.069
    m_arm_ax2 = 1.219
    m_ax2_parrods = 0.214
    ms = np.array([0.0, m_link1, 0.0, m_forearms, 0.0, m_wristassembly, 0.0,
                   m_arm_ax2, 0.0, m_ax2_parrods])
    rcgs = np.array([[0.0, 0.0, 0.0],
                     [0.14264, -0.0423, 0.0],
                     [0.0, 0.0, 0.0],
                     [0.0, 0.19988, 0.0],
                     [0.0, 0.0, 0.0],
                     [0.0, 0.62455 - kin_model.L1 + kin_model.d2, 0.0],
                     [0.0, 0.0, 0.0],
                     [0.09929, 0.0, 0.0],
                     [0.0, 0.0, 0.0],
                     [kin_model.Lpar/2, 0.0, 0.0]])
    ivals = np.zeros((10, 3, 3))
    ivals[1, 2, 2] = 0.72301890389 + 0.7
    ivals[2, 0, 0] = 0.7
    ivals[3, 0, 0] = 0.083745879
    ivals[3, 2, 2] = 0.060760597
    ivals[6, 2, 2] = 0.004843647
    ivals[7, 2, 2] = 0.003822624 + 0.7

    c_dyn_pars = convert_dyn_params2(ms, rcgs, ivals)
    c_tool_dyn_pars = convert_tool_params2(m_load, rcg_load, i_load)

    q = np.array([0.1, 1.5, -0.3, 45.0])
    qd = np.array([0.5, -0.5, 0.5, 30.0])
    qdd = np.array([2.3, 4.5, 6.23, -301.4])
    g = np.array([0.0, 0.0, -9.82])

    print("Without all coriolis/centrifugal forces terms:")
    trq = dyn_inv_agile(c_kin_model, c_lpoe_model, c_dyn_pars, c_tool_dyn_pars,
                        q, qd, qdd, g, 0)
    print("trq:", trq)

    print("Including all coriolis/centrifugal forces terms:")
    trq = dyn_inv_agile(c_kin_model, c_lpoe_model, c_dyn_pars, c_tool_dyn_pars,
                        q, qd, qdd, g, 1)
    print("trq:", trq)

    print("Inertia matrix:")
    imat = inertia_matrix(c_kin_model, c_lpoe_model, c_dyn_pars, c_tool_dyn_pars, q)
    print(np.array(imat))

    print("Inverse kin including velocity/acceleration:")
    pos = [-0.944527339, 0.8684955767, -0.125531043]
    ori = -0.7853981633974483
    v = [-0.0, 4.1955727185, -0.4267976723, 0.0]
    a = [0.0, -0.0, -0.0, -0.0]
    hkm1800 = agile_pkm_model.ConceptPrototype_long()
    hkm1800_c = convert_robdef(hkm1800)
    q, qd, qdd = inv_with_vel_acc(hkm1800_c, pos, ori, v, a)
    print("q:", q)
    print("qd:", qd)
    print("qdd:", qdd)

    # Test also Mark1_5
    mark15 = agile_pkm_model.Mark1_5()
    mark15_lpoe = dyn.AgileRobModel(mark15)
    mark15_c = convert_robdef(mark15)
    mark15_lpoe_c = convert_lpoe_def(mark15_lpoe)

    print("Torque for Mark 1 5:")
    q = np.array([0.1, 1.5, -0.3, 45.0])
    qd = np.array([0.5, -0.5, 0.5, 30.0])
    qdd = np.array([2.3, 4.5, 6.23, -301.4])
    g = np.array([0.0, 0.0, -9.82])
    trq = dyn_inv_agile(mark15_c, mark15_lpoe_c, c_dyn_pars, c_tool_dyn_pars,
                        q, qd, qdd, g, 0)
    print("trq:", trq)


if __name__ == '__main__':
    main()

