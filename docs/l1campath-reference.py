#!/usr/bin/env python3
"""Independent reference implementation of the L1 optimal camera path LP.

This mirrors src/l1campathoptimization.c exactly -- same parametrisation, same
residuals, same constraints -- but builds the problem with numpy and solves it
with scipy/HiGHS instead of the vendored solver.  It exists so that the model
in the C code can be cross-checked against something that was written from the
paper independently of it:

  Grundmann, Kwatra, Essa: "Auto-Directed Video Stabilization with Robust L1
  Optimal Camera Paths", CVPR 2011.

Run it to obtain the value that tests/test_campathopt.c compares against:

    python3 docs/l1campath-reference.py

Requires numpy and scipy; neither is needed to build or test vid.stab.
"""

import math
import numpy as np
from scipy.optimize import linprog

X, Y, A, B = 0, 1, 2, 3


# ------------------------------------------------------- similarity transforms
#
#     [ a  b  x ]
#     [-b  a  y ]   stored as (x, y, a, b), acting on centre-relative coords
#     [ 0  0  1 ]

def ident():
    return np.array([0.0, 0.0, 1.0, 0.0])


def concat(t1, t2):
    """matrix product t1 * t2, i.e. t2 applied first"""
    x1, y1, a1, b1 = t1
    x2, y2, a2, b2 = t2
    return np.array([a1 * x2 + b1 * y2 + x1,
                     -b1 * x2 + a1 * y2 + y1,
                     a1 * a2 - b1 * b2,
                     a1 * b2 + b1 * a2])


def invert(t):
    x, y, a, b = t
    z = a * a + b * b
    ra, rb = a / z, -b / z
    return np.array([-(ra * x + rb * y), -(-rb * x + ra * y), ra, rb])


def apply_pt(t, px, py):
    x, y, a, b = t
    return a * px + b * py + x, -b * px + a * py + y


# ------------------------------------------------------------ the LP, verbatim

class Layout:
    """same numbering as vs_l1_col / vs_l1_row in the C code"""

    def __init__(self, N):
        self.N = N

    def col(self, group, t, param):
        return 4 * (group * self.N - (group * (group - 1)) // 2) + 4 * t + param

    def numcols(self):
        return self.col(3, self.N - 3, 0)


def mul_FB(r, F, t, s, lay):
    """accumulate s * (F B_t) into the four component expressions r"""
    fx, fy, fa, fb = F
    r[X][0][lay.col(0, t, X)] = r[X][0].get(lay.col(0, t, X), 0.0) + s * fa
    r[X][0][lay.col(0, t, Y)] = r[X][0].get(lay.col(0, t, Y), 0.0) + s * fb
    r[X][1] += s * fx
    r[Y][0][lay.col(0, t, X)] = r[Y][0].get(lay.col(0, t, X), 0.0) - s * fb
    r[Y][0][lay.col(0, t, Y)] = r[Y][0].get(lay.col(0, t, Y), 0.0) + s * fa
    r[Y][1] += s * fy
    r[A][0][lay.col(0, t, A)] = r[A][0].get(lay.col(0, t, A), 0.0) + s * fa
    r[A][0][lay.col(0, t, B)] = r[A][0].get(lay.col(0, t, B), 0.0) - s * fb
    r[B][0][lay.col(0, t, A)] = r[B][0].get(lay.col(0, t, A), 0.0) + s * fb
    r[B][0][lay.col(0, t, B)] = r[B][0].get(lay.col(0, t, B), 0.0) + s * fa


def add_B(r, t, s, lay):
    for p in (X, Y, A, B):
        r[p][0][lay.col(0, t, p)] = r[p][0].get(lay.col(0, t, p), 0.0) + s


def residual(order, F, t, lay):
    """eqs. (4)-(6) with R_t = F_{t+1} B_{t+1} - B_t"""
    r = [[{}, 0.0] for _ in range(4)]
    if order == 1:
        mul_FB(r, F[t + 1], t + 1, +1.0, lay)
        add_B(r, t, -1.0, lay)
    elif order == 2:
        mul_FB(r, F[t + 2], t + 2, +1.0, lay)
        add_B(r, t + 1, -1.0, lay)
        mul_FB(r, F[t + 1], t + 1, -1.0, lay)
        add_B(r, t, +1.0, lay)
    elif order == 3:
        mul_FB(r, F[t + 3], t + 3, +1.0, lay)
        add_B(r, t + 2, -1.0, lay)
        mul_FB(r, F[t + 2], t + 2, -2.0, lay)
        add_B(r, t + 1, +2.0, lay)
        mul_FB(r, F[t + 1], t + 1, +1.0, lay)
        add_B(r, t, -1.0, lay)
    return r


DEFAULTS = dict(w1=10.0, w2=1.0, w3=100.0, w_affine=100.0,
                crop_ratio=1.0 / 1.15, min_scale=1.0, max_scale=1.1,
                max_skew=0.1)


def solve(F, width, height, **kw):
    conf = dict(DEFAULTS)
    conf.update(kw)
    N = len(F)
    lay = Layout(N)
    ncols = lay.numcols()

    c = np.zeros(ncols)
    A_ub, b_ub = [], []

    def add_row(coeffs, rhs):
        row = np.zeros(ncols)
        for j, v in coeffs.items():
            row[j] += v
        A_ub.append(row)
        b_ub.append(rhs)

    weights = (conf["w1"], conf["w2"], conf["w3"])
    for order in (1, 2, 3):
        for t in range(N - order):
            r = residual(order, F, t, lay)
            for p in (X, Y, A, B):
                lin, konst = r[p]
                ecol = lay.col(order, t, p)
                c[ecol] = weights[order - 1] * (conf["w_affine"]
                                                if p in (A, B) else 1.0)
                # -e <= lin + konst <= e
                add_row({**lin, ecol: -1.0}, -konst)
                add_row({**{k: -v for k, v in lin.items()}, ecol: -1.0}, konst)

    # inclusion: the four crop corners, transformed by B_t, stay in the frame
    x2, y2 = width / 2.0, height / 2.0
    cw, ch = x2 * conf["crop_ratio"], y2 * conf["crop_ratio"]
    for t in range(N):
        for (cx, cy) in [(-cw, -ch), (cw, -ch), (cw, ch), (-cw, ch)]:
            row = {lay.col(0, t, X): 1.0, lay.col(0, t, A): cx,
                   lay.col(0, t, B): cy}
            add_row(row, x2)
            add_row({k: -v for k, v in row.items()}, x2)
            row = {lay.col(0, t, Y): 1.0, lay.col(0, t, B): -cx,
                   lay.col(0, t, A): cy}
            add_row(row, y2)
            add_row({k: -v for k, v in row.items()}, y2)

    # proximity on B, non-negativity on the slacks
    bounds = []
    for _ in range(N):
        bounds += [(None, None), (None, None),
                   (conf["min_scale"], conf["max_scale"]),
                   (-conf["max_skew"], conf["max_skew"])]
    bounds += [(0.0, None)] * (ncols - 4 * N)

    res = linprog(c, A_ub=np.array(A_ub), b_ub=np.array(b_ub), bounds=bounds,
                  method="highs")
    if not res.success:
        raise RuntimeError(res.message)
    return res.x[:4 * N].reshape(N, 4), res.fun


# ------------------------------------- the synthetic path used by the C tests

def ground_truth(t):
    """must stay identical to campath_ground_truth() in tests/test_campathopt.c"""
    s = float(t)
    ang = 0.02 * math.sin(s * 0.11) + 0.01 * math.sin(s * 1.9)
    return np.array([12.0 * s + 5.0 * math.sin(s * 1.7),
                     20.0 * math.sin(s * 0.05) + 3.0 * math.cos(s * 2.3),
                     math.cos(ang), math.sin(ang)])


def frame_pairs(N):
    F = [ident()]
    for t in range(1, N):
        F.append(concat(invert(ground_truth(t - 1)), ground_truth(t)))
    return np.array(F)


def diffnorm(P, order):
    d = P * np.array([1.0, 1.0, 100.0, 100.0])
    for _ in range(order):
        d = np.diff(d, axis=0)
    return float(np.abs(d).sum())


if __name__ == "__main__":
    W, H = 640.0, 480.0

    N = 24
    F = frame_pairs(N)
    upd, obj = solve(F, W, H)
    print(f"N = {N}, reference objective = {obj:.10g}")
    print()
    print("  #define L1_REFERENCE_OBJECTIVE %.12g" % obj)
    print()

    for N in (60, 200):
        F = frame_pairs(N)
        C = np.array([ground_truth(t) for t in range(N)])
        upd, obj = solve(F, W, H)
        P = np.array([concat(C[t], upd[t]) for t in range(N)])
        print(f"N = {N}: objective {obj:12.4f}", end="")
        for o in (1, 2, 3):
            print(f"   |D^{o}| {diffnorm(C, o):9.2f} -> {diffnorm(P, o):8.2f}",
                  end="")
        print()
