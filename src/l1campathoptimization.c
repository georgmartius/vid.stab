/*
 *  l1campathoptimization.c
 *
 *  Copyright (C) Georg Martius - January 2014 - 2026
 *   georg dot martius at web dot de
 *
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  This file is part of vid.stab video stabilization library
 *
 *  vid.stab is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published
 *  by the Free Software Foundation; either version 2.1 of the License, or
 *  (at your option) any later version.
 *
 *  vid.stab is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with vid.stab; see the file COPYING.LESSER.  If not, see
 *  <https://www.gnu.org/licenses/>.
 *
 */

#include "l1campathoptimization.h"
#include "transform_internal.h"
#include "transformtype_operations.h"
#include "vidstabdefines.h"
#include "lpsolver.h"

#include <math.h>
#include <string.h>

/* ****************************************************************************
 * operations on VSTransformLS
 * ************************************************************************** */

VSTransformLS id_transformLS(void){
  VSTransformLS t = { 0.0, 0.0, 1.0, 0.0, 0 };
  return t;
}

/* matrix product t1 * t2, i.e. t2 is applied first.  With
     [ a  b  x ]           (t1 t2).x = a1 x2 + b1 y2 + x1
     [-b  a  y ]           (t1 t2).a = a1 a2 - b1 b2
   this is just the product of the two similarity matrices. */
VSTransformLS concat_transformLS(const VSTransformLS* t1, const VSTransformLS* t2){
  VSTransformLS t;
  t.x     = t1->a * t2->x + t1->b * t2->y + t1->x;
  t.y     = -t1->b * t2->x + t1->a * t2->y + t1->y;
  t.a     = t1->a * t2->a - t1->b * t2->b;
  t.b     = t1->a * t2->b + t1->b * t2->a;
  t.extra = t1->extra || t2->extra;
  return t;
}

VSTransformLS invert_transformLS(const VSTransformLS* t){
  VSTransformLS r;
  double z = t->a * t->a + t->b * t->b;
  if (z == 0.0) return id_transformLS();  // degenerate, should not happen
  r.a     =  t->a / z;
  r.b     = -t->b / z;
  r.x     = -(r.a * t->x + r.b * t->y);
  r.y     = -(-r.b * t->x + r.a * t->y);
  r.extra = t->extra;
  return r;
}

void transformLS_vec(double* rx, double* ry, const VSTransformLS* t,
                     double x, double y){
  *rx =  t->a * x + t->b * y + t->x;
  *ry = -t->b * x + t->a * y + t->y;
}

VSTransformLS transformAZtoLS(const VSTransform* t){
  VSTransformLS r;
  double z = 1.0 + t->zoom / 100.0;
  r.x     = t->x;
  r.y     = t->y;
  r.a     = z * cos(t->alpha);
  r.b     = z * sin(t->alpha);
  r.extra = t->extra;
  return r;
}

VSTransform transformLStoAZ(const VSTransformLS* t){
  VSTransform r = null_transform();
  r.x     = t->x;
  r.y     = t->y;
  r.alpha = atan2(t->b, t->a);
  r.zoom  = (sqrt(t->a * t->a + t->b * t->b) - 1.0) * 100.0;
  r.extra = t->extra;
  return r;
}

/* ****************************************************************************
 * layout of the linear program
 *
 * Columns (4 per group and time step, in the order x, y, a, b):
 *   group 0:  B_t,       t = 0 .. N-1   the update transforms
 *   group 1:  e^1_t,     t = 0 .. N-2   slack bounding |D(P)|
 *   group 2:  e^2_t,     t = 0 .. N-3   slack bounding |D^2(P)|
 *   group 3:  e^3_t,     t = 0 .. N-4   slack bounding |D^3(P)|
 *
 * Rows are grouped by time step, 8 inclusion rows (4 crop corners x 2
 * coordinates) followed by 8 rows for every derivative order that is still
 * defined at t (4 parameters x lower/upper bound):
 *
 *   t = 0 .. N-4 : inclusion, order 1, order 2, order 3   (32 rows)
 *   t = N-3      : inclusion, order 1, order 2            (24 rows)
 *   t = N-2      : inclusion, order 1                     (16 rows)
 *   t = N-1      : inclusion                              ( 8 rows)
 *
 * Grouping by time rather than by order is what keeps the constraint matrix
 * banded: a row at time t only touches B_t .. B_{t+3}, so two rows share a
 * column only if their time steps are at most 3 apart.  The interior point
 * backend relies on that to factorize the normal equations in O(N).
 * ************************************************************************** */

enum { PX = 0, PY = 1, PA = 2, PB = 3 };

/** first column of group `group` (0 = B, 1..3 = slack of that order).
    Group g starts after the groups before it, which hold N, N-1, N-2 entries
    of 4 columns each: 4*(g*N - g(g-1)/2). */
int vs_l1_col(int group, int t, int param, int N){
  return 4 * (group * N - (group * (group - 1)) / 2) + 4 * t + param;
}

/** number of rows belonging to time step t */
static int vs_l1_rowsAt(int t, int N){
  return 8 + (t < N - 1 ? 8 : 0) + (t < N - 2 ? 8 : 0) + (t < N - 3 ? 8 : 0);
}

/** index of the first row of time step t */
int vs_l1_rowBase(int t, int N){
  /* every time step before N-3 contributes the full 32 rows */
  int full = (t < N - 3) ? t : (N - 3);
  int base = 32 * full;
  for (int k = N - 3; k < t; k++) base += vs_l1_rowsAt(k, N);
  return base;
}

/** inclusion row k = 0..7 of time step t */
int vs_l1_rowCorner(int t, int k, int N){
  return vs_l1_rowBase(t, N) + k;
}

/** row of the smoothness constraint of the given order (1..3).
    upperorlower is 0 for the lower and 1 for the upper bound. */
int vs_l1_row(int order, int t, int param, int upperorlower, int N){
  return vs_l1_rowBase(t, N) + 8 * order + 2 * param + upperorlower;
}

int vs_l1_numrows(int N){ return vs_l1_rowBase(N - 1, N) + 8; }
int vs_l1_numcols(int N){ return vs_l1_col(3, N - 3, 0, N); }

/** upper bound on the number of nonzero matrix entries:
    order 1 rows have 4, order 2 rows have 6, order 3 rows have 8 and
    inclusion rows have 3 entries. */
static int vs_l1_maxentries(int N){
  return 8 * (N - 1) * 4 + 8 * (N - 2) * 6 + 8 * (N - 3) * 8 + 8 * N * 3;
}

/* ****************************************************************************
 * building the residuals
 *
 * A residual is held as a sparse linear expression over the B columns plus a
 * constant, so that
 *
 *     residual = sum_i val[i] * x[col[i]] + konst.
 *
 * Both the lower and the upper constraint row of a residual are emitted from
 * that single expression, which is what keeps the two rows of a pair in sync.
 * ************************************************************************** */

#define L1_MAXTERMS 12

typedef struct {
  int    col[L1_MAXTERMS];
  double val[L1_MAXTERMS];
  int    n;
  double konst;
} L1Expr;

static void exprReset(L1Expr* e){ e->n = 0; e->konst = 0.0; }

static void exprAdd(L1Expr* e, int col, double v){
  if (v == 0.0) return;
  for (int i = 0; i < e->n; i++) {
    if (e->col[i] == col) { e->val[i] += v; return; }
  }
  if (e->n >= L1_MAXTERMS) return;  // cannot happen, see L1_MAXTERMS
  e->col[e->n] = col;
  e->val[e->n] = v;
  e->n++;
}

/** accumulates s * (F B_t) into the four component expressions r.
      (F B).x =  fa Bx + fb By + fx        (F B).a = fa Ba - fb Bb
      (F B).y = -fb Bx + fa By + fy        (F B).b = fa Bb + fb Ba  */
static void exprAddFB(L1Expr r[4], const VSTransformLS* F, int t, double s, int N){
  exprAdd(&r[PX], vs_l1_col(0, t, PX, N),  s * F->a);
  exprAdd(&r[PX], vs_l1_col(0, t, PY, N),  s * F->b);
  r[PX].konst += s * F->x;
  exprAdd(&r[PY], vs_l1_col(0, t, PX, N), -s * F->b);
  exprAdd(&r[PY], vs_l1_col(0, t, PY, N),  s * F->a);
  r[PY].konst += s * F->y;
  exprAdd(&r[PA], vs_l1_col(0, t, PA, N),  s * F->a);
  exprAdd(&r[PA], vs_l1_col(0, t, PB, N), -s * F->b);
  exprAdd(&r[PB], vs_l1_col(0, t, PA, N),  s * F->b);
  exprAdd(&r[PB], vs_l1_col(0, t, PB, N),  s * F->a);
}

/** accumulates s * B_t into the four component expressions r */
static void exprAddB(L1Expr r[4], int t, double s, int N){
  for (int p = PX; p <= PB; p++) exprAdd(&r[p], vs_l1_col(0, t, p, N), s);
}

/** residual of order 1..3 at time t, following eqs. (4)-(6) of the paper with
      R_t = F_{t+1} B_{t+1} - B_t
    Note that the operations are additive, not compositional: the identity I
    below denotes the identity of the parameter space, not the identity
    transform composed with B. */
static void buildResidual(L1Expr r[4], int order, const VSTransformLS* F,
                          int t, int N){
  for (int p = PX; p <= PB; p++) exprReset(&r[p]);
  switch (order) {
   case 1:  // R_t
    exprAddFB(r, &F[t + 1], t + 1, +1.0, N);
    exprAddB (r,             t,    -1.0, N);
    break;
   case 2:  // R_{t+1} - R_t = F_{t+2}B_{t+2} - (I + F_{t+1})B_{t+1} + B_t
    exprAddFB(r, &F[t + 2], t + 2, +1.0, N);
    exprAddB (r,             t + 1, -1.0, N);
    exprAddFB(r, &F[t + 1], t + 1, -1.0, N);
    exprAddB (r,             t,     +1.0, N);
    break;
   case 3:  // R_{t+2} - 2R_{t+1} + R_t
            //   = F_{t+3}B_{t+3} - (I+2F_{t+2})B_{t+2} + (2I+F_{t+1})B_{t+1} - B_t
    exprAddFB(r, &F[t + 3], t + 3, +1.0, N);
    exprAddB (r,             t + 2, -1.0, N);
    exprAddFB(r, &F[t + 2], t + 2, -2.0, N);
    exprAddB (r,             t + 1, +2.0, N);
    exprAddFB(r, &F[t + 1], t + 1, +1.0, N);
    exprAddB (r,             t,     -1.0, N);
    break;
   default:
    break;
  }
}

/** emits the constraint pair  -e <= expr <= e  as
      lower row:  expr_linear + e >= -konst
      upper row:  expr_linear - e <= -konst
    Deriving both rows from the same expression is deliberate: in the earlier
    hand-written version the two rows of one pair had drifted apart, which
    silently turned an absolute value into an equality. */
static void emitPair(VSLinProg* lp, int rowlo, int rowup, int ecol,
                     const L1Expr* e){
  for (int i = 0; i < e->n; i++) {
    vs_lp_add_entry(lp, rowlo, e->col[i], e->val[i]);
    vs_lp_add_entry(lp, rowup, e->col[i], e->val[i]);
  }
  vs_lp_add_entry(lp, rowlo, ecol,  1.0);
  vs_lp_add_entry(lp, rowup, ecol, -1.0);
  vs_lp_set_row_bounds(lp, rowlo, -e->konst,  VS_LP_INF);
  vs_lp_set_row_bounds(lp, rowup, -VS_LP_INF, -e->konst);
}

/** value of a residual at a given point */
static double exprEval(const L1Expr* e, const double* x){
  double s = e->konst;
  for (int i = 0; i < e->n; i++) s += e->val[i] * x[e->col[i]];
  return s;
}

/** Objective of a candidate solution, evaluated directly from B rather than
    taken from the solver.

    The slack variables of the program are only pushed down to |residual| by the
    objective, so a solver that stops slightly short of optimality reports a
    value that is not the objective of the B it returns -- it can even be below
    the true optimum.  Recomputing here makes the number exact and comparable
    across backends. */
static double objectiveOf(const VSTransformLS* F, int N, const VSL1Config* conf,
                          const double* Bflat){
  const double weight[3] = { conf->w1, conf->w2, conf->w3 };
  double total = 0.0;
  L1Expr r[4];
  for (int order = 1; order <= 3; order++) {
    for (int t = 0; t < N - order; t++) {
      buildResidual(r, order, F, t, N);
      for (int p = PX; p <= PB; p++) {
        double w = weight[order - 1] * ((p == PA || p == PB) ? conf->wAffine : 1.0);
        total += w * fabs(exprEval(&r[p], Bflat));
      }
    }
  }
  return total;
}

/** Removes any residual constraint violation.

    The inclusion and proximity constraints are only satisfied up to whatever
    accuracy the solver reached.  Both describe a convex set that contains the
    identity transform, so shrinking B_t towards the identity restores
    feasibility exactly, and because the violation is tiny the shrink factor is
    1 or a hair below it.  This is what turns "the LP said so" into an actual
    guarantee that the crop window never leaves the frame. */
static void enforceFeasibility(VSTransformLS* B, int N, const VSL1Config* conf){
  const double x2 = conf->frameWidth  / 2.0;
  const double y2 = conf->frameHeight / 2.0;
  const double cw = x2 * conf->cropRatio;
  const double ch = y2 * conf->cropRatio;
  const double cornerx[4] = { -cw,  cw, cw, -cw };
  const double cornery[4] = { -ch, -ch, ch,  ch };

  for (int t = 0; t < N; t++) {
    double lambda = 1.0;
    /* value at the identity, and the difference to the candidate, for every
       constraint; feasibility is affine in lambda */
    double idv[12], curv[12], lo[12], up[12];
    int n = 0;
    for (int i = 0; i < 4; i++) {
      double ix, iy, bx, by;
      VSTransformLS id = id_transformLS();
      transformLS_vec(&ix, &iy, &id,   cornerx[i], cornery[i]);
      transformLS_vec(&bx, &by, &B[t], cornerx[i], cornery[i]);
      idv[n] = ix; curv[n] = bx; lo[n] = -x2; up[n] = x2; n++;
      idv[n] = iy; curv[n] = by; lo[n] = -y2; up[n] = y2; n++;
    }
    idv[n] = 1.0; curv[n] = B[t].a; lo[n] = conf->minScale; up[n] = conf->maxScale; n++;
    idv[n] = 0.0; curv[n] = B[t].b; lo[n] = -conf->maxSkewDev; up[n] = conf->maxSkewDev; n++;

    for (int i = 0; i < n; i++) {
      double d = curv[i] - idv[i];
      if (d > 0.0 && curv[i] > up[i]) {
        double l = (up[i] - idv[i]) / d;
        if (l < lambda) lambda = l;
      } else if (d < 0.0 && curv[i] < lo[i]) {
        double l = (lo[i] - idv[i]) / d;
        if (l < lambda) lambda = l;
      }
    }
    if (lambda < 0.0) lambda = 0.0;
    if (lambda < 1.0) {
      B[t].x = lambda * B[t].x;
      B[t].y = lambda * B[t].y;
      B[t].a = 1.0 + lambda * (B[t].a - 1.0);
      B[t].b = lambda * B[t].b;
    }
  }
}

/* ************************************************************************* */

VSL1Config vsL1GetDefaultConfig(void){
  VSL1Config c;
  /* fig. 8(d) of the paper: eliminating jerks matters most, so w3 is an order
     of magnitude above w1 and w2 */
  c.w1          = 10.0;
  c.w2          = 1.0;
  c.w3          = 100.0;
  c.wAffine     = 100.0;
  c.frameWidth  = 0.0;
  c.frameHeight = 0.0;
  c.cropRatio   = 1.0 / (1.0 + VS_L1_DEFAULT_ZOOM / 100.0);
  c.minScale    = 1.0;
  c.maxScale    = 1.1;
  c.maxSkewDev  = 0.1;
  c.verbose     = 0;
  return c;
}

int vsCameraPathOptimalL1LS(const VSTransformLS* F, int N, VSTransformLS* B,
                            const VSL1Config* conf, double* objective){
  if (!F || !B || !conf) return VS_ERROR;
  /* the third derivative needs frames t .. t+3 */
  if (N < 4) return VS_ERROR;
  if (!(conf->frameWidth > 0.0) || !(conf->frameHeight > 0.0)) return VS_ERROR;
  if (!(conf->cropRatio > 0.0) || conf->cropRatio > 1.0) return VS_ERROR;
  if (!(conf->minScale > 0.0) || conf->minScale > conf->maxScale) return VS_ERROR;

  const int numrows = vs_l1_numrows(N);
  const int numcols = vs_l1_numcols(N);
  VSLinProg* lp = vs_lp_new("vid.stab L1 camera path", numrows, numcols,
                            vs_l1_maxentries(N));
  if (!lp) return VS_ERROR;

  const double weight[3] = { conf->w1, conf->w2, conf->w3 };

  /* --- columns: B is free except for the proximity bounds, the slacks are
     non-negative and carry the whole objective --------------------------- */
  const double x2 = conf->frameWidth  / 2.0;
  const double y2 = conf->frameHeight / 2.0;
  for (int t = 0; t < N; t++) {
    /* |B.x| <= x2 and |B.y| <= y2 are implied by the inclusion rows below:
       averaging the two rows of a pair of opposite corners cancels the a and b
       terms and leaves exactly these bounds.  Stating them explicitly costs
       nothing, tightens the relaxation, and gives every variable of the
       program a finite lower bound, which the interior point backend needs. */
    vs_lp_set_col_bounds(lp, vs_l1_col(0, t, PX, N), -x2, x2);
    vs_lp_set_col_bounds(lp, vs_l1_col(0, t, PY, N), -y2, y2);
    /* proximity constraints, section 2.1 of the paper: keep the update
       transform close to the identity so the crop does not distort */
    vs_lp_set_col_bounds(lp, vs_l1_col(0, t, PA, N),
                         conf->minScale, conf->maxScale);
    vs_lp_set_col_bounds(lp, vs_l1_col(0, t, PB, N),
                         -conf->maxSkewDev, conf->maxSkewDev);
  }
  for (int order = 1; order <= 3; order++) {
    for (int t = 0; t < N - order; t++) {
      for (int p = PX; p <= PB; p++) {
        int col = vs_l1_col(order, t, p, N);
        vs_lp_set_col_bounds(lp, col, 0.0, VS_LP_INF);
        vs_lp_set_obj(lp, col, weight[order - 1]
                      * ((p == PA || p == PB) ? conf->wAffine : 1.0));
      }
    }
  }

  /* --- smoothness rows -------------------------------------------------- */
  L1Expr r[4];
  for (int order = 1; order <= 3; order++) {
    for (int t = 0; t < N - order; t++) {
      buildResidual(r, order, F, t, N);
      for (int p = PX; p <= PB; p++) {
        emitPair(lp,
                 vs_l1_row(order, t, p, 0, N),
                 vs_l1_row(order, t, p, 1, N),
                 vs_l1_col(order, t, p, N),
                 &r[p]);
      }
    }
  }

  /* --- inclusion rows: all four corners of the crop rectangle, transformed
     by B_t, have to stay inside the frame rectangle (eq. 8, in coordinates
     relative to the frame centre) ---------------------------------------- */
  const double cw = x2 * conf->cropRatio;
  const double ch = y2 * conf->cropRatio;
  const double cornerx[4] = { -cw,  cw, cw, -cw };
  const double cornery[4] = { -ch, -ch, ch,  ch };
  for (int t = 0; t < N; t++) {
    for (int i = 0; i < 4; i++) {
      double cx = cornerx[i], cy = cornery[i];
      int rowx = vs_l1_rowCorner(t, 2 * i,     N);
      int rowy = vs_l1_rowCorner(t, 2 * i + 1, N);
      /* -x2 <= Bx + a cx + b cy <= x2 */
      vs_lp_add_entry(lp, rowx, vs_l1_col(0, t, PX, N), 1.0);
      vs_lp_add_entry(lp, rowx, vs_l1_col(0, t, PA, N), cx);
      vs_lp_add_entry(lp, rowx, vs_l1_col(0, t, PB, N), cy);
      vs_lp_set_row_bounds(lp, rowx, -x2, x2);
      /* -y2 <= By - b cx + a cy <= y2 */
      vs_lp_add_entry(lp, rowy, vs_l1_col(0, t, PY, N), 1.0);
      vs_lp_add_entry(lp, rowy, vs_l1_col(0, t, PB, N), -cx);
      vs_lp_add_entry(lp, rowy, vs_l1_col(0, t, PA, N), cy);
      vs_lp_set_row_bounds(lp, rowy, -y2, y2);
    }
  }

  /* --- solve ------------------------------------------------------------ */
  int status = vs_lp_solve(lp, conf->verbose & VS_DEBUG);
  if (status != VS_OK) {
    vs_log_error("vid.stab", "L1 camera path: %s (%s, %i rows, %i cols)",
                 vs_lp_status_msg(lp), vs_lp_backend_name(), numrows, numcols);
    vs_lp_free(lp);
    return VS_ERROR;
  }
  for (int t = 0; t < N; t++) {
    B[t].x     = vs_lp_get_col_value(lp, vs_l1_col(0, t, PX, N));
    B[t].y     = vs_lp_get_col_value(lp, vs_l1_col(0, t, PY, N));
    B[t].a     = vs_lp_get_col_value(lp, vs_l1_col(0, t, PA, N));
    B[t].b     = vs_lp_get_col_value(lp, vs_l1_col(0, t, PB, N));
    B[t].extra = 0;
  }
  vs_lp_free(lp);

  enforceFeasibility(B, N, conf);
  if (objective) {
    double* Bflat = (double*)vs_malloc(sizeof(double) * 4 * N);
    if (Bflat) {
      for (int t = 0; t < N; t++) {
        Bflat[vs_l1_col(0, t, PX, N)] = B[t].x;
        Bflat[vs_l1_col(0, t, PY, N)] = B[t].y;
        Bflat[vs_l1_col(0, t, PA, N)] = B[t].a;
        Bflat[vs_l1_col(0, t, PB, N)] = B[t].b;
      }
      *objective = objectiveOf(F, N, conf, Bflat);
      vs_free(Bflat);
    }
  }
  return VS_OK;
}

/* ************************************************************************* */

VSL1Config vsL1ConfigFromTransformConfig(const VSTransformData* td){
  VSL1Config c = vsL1GetDefaultConfig();

  /* --- zoom budget, from zoom and optZoom ---------------------------------
     The optimization gets a crop window that is `budget` percent smaller than
     the frame and keeps that window inside the frame; the zoom that goes with
     it ends up in the render transforms.  That is precisely what zoom/optZoom
     already mean, so they are what we read:

       optZoom == 0            the user wants no automatic zoom: the budget is
                               whatever conf.zoom asks for, and nothing more.
                               With conf.zoom <= 0 there is no room to move at
                               all and cameraPathOptimalL1() gives up.
       optZoom != 0            conf.zoom if it was given, else the default.

     optZoom == 2 (adaptive) has no L1 counterpart -- the crop window has to be
     fixed before the LP is built -- so it is treated like the static case. */
  double budget = VS_MAX(td->conf.zoom, 0.0);
  if (td->conf.optZoom != 0 && budget <= 0.0) budget = VS_L1_DEFAULT_ZOOM;
  c.cropRatio = 1.0 / (1.0 + budget / 100.0);

  /* --- objective weights, from smoothing ----------------------------------
     The LP has no data term competing with the objective -- closeness to the
     original path comes from the inclusion and proximity constraints -- so it
     is invariant under a common scaling of w1..w3 and only their ratio counts.
     Read conf.smoothing as the timescale T (in frames) over which the path
     should look rigid: |D^k P| is of order (amplitude / t^k) for motion of
     period t, so weighting the k-th term with T^k makes all three contribute
     equally at t = T, and motion faster than T is punished by the higher
     orders.  Raising T therefore shifts weight from |D(P)| to |D^3(P)|: the
     path goes from short static holds with quick transitions towards long
     sweeping parabolic moves.  Written as s = T / T0 the weights are
     10 s, s^2, 100 s^3, which we divide by s^2 to keep the numbers in a range
     the solver is comfortable with; at T = T0 this reproduces the ratio of
     fig. 8(d) of the paper exactly. */
  const double T0 = 15.0;  /* the default of conf.smoothing */
  double s = VS_MAX(td->conf.smoothing, 1) / T0;
  c.w1 = 10.0 / s;
  c.w2 = 1.0;
  c.w3 = 100.0 * s;

  c.frameWidth  = td->fiSrc.width;
  c.frameHeight = td->fiSrc.height;
  c.verbose     = td->conf.verbose;
  return c;
}

/** Converts an update transform B_t into the VSTransform that the warping code
    has to be fed to realize it.

    transformPlanar() in transformfloat.c maps a destination pixel p_d, taken
    relative to the frame centre, to the source pixel

        p_s = z R(-alpha) p_d - (x,y),      z = 1 - zoom/100

    while the destination frame corresponds to the crop rectangle, which is
    cropRatio as large as the frame.  We therefore want

        p_s = B(cropRatio * p_d)

    Comparing the two, with theta and |B| the rotation and the scale of B,
    gives the exact conversion below.  The zoom needed to see only the crop
    rectangle is part of it, which is why cameraPathOptimalL1() switches the
    heuristic zoom estimation off: the optimization already guaranteed that
    this exact crop stays inside the frame. */
static VSTransform updateToRenderTransform(const VSTransformLS* B, double cropRatio){
  VSTransform r = null_transform();
  r.x     = -B->x;
  r.y     = -B->y;
  r.alpha = -atan2(B->b, B->a);
  r.zoom  = (1.0 - cropRatio * sqrt(B->a * B->a + B->b * B->b)) * 100.0;
  return r;
}

int cameraPathOptimalL1(VSTransformData* td, VSTransformations* trans){
  if (!td || !trans || !trans->ts) return VS_ERROR;
  const int N = trans->len;
  if (N < 1) return VS_ERROR;
  if (!td->conf.relative) {
    vs_log_error(td->conf.modName,
                 "L1 camera path optimization requires relative transforms");
    return VS_ERROR;
  }
  if (N < 4) {
    vs_log_error(td->conf.modName,
                 "L1 camera path optimization needs at least 4 frames, got %i", N);
    return VS_ERROR;
  }
  if (td->conf.verbose & VS_DEBUG) {
    vs_log_msg(td->conf.modName, "L1 optimization of camera path (%s)",
               vs_lp_backend_name());
  }

  VSTransformLS* F = (VSTransformLS*)vs_malloc(sizeof(VSTransformLS) * N);
  VSTransformLS* B = (VSTransformLS*)vs_malloc(sizeof(VSTransformLS) * N);
  if (!F || !B) { vs_free(F); vs_free(B); return VS_ERROR; }
  /* The camera path is the running composition of the relative transforms,
     C_t = C_{t-1} F_t, so the frame-pair transforms are the relative
     transforms themselves.  F[0] is never read (C_0 drops out of every
     residual), but is initialised so the array is fully defined. */
  F[0] = id_transformLS();
  for (int t = 1; t < N; t++) F[t] = transformAZtoLS(&trans->ts[t]);

  VSL1Config conf = vsL1ConfigFromTransformConfig(td);
  if (conf.cropRatio >= 1.0) {
    /* optZoom == 0 and no zoom asked for: the crop window is the whole frame,
       so the inclusion constraints pin the path to the original and there is
       nothing to optimize.  Say so and let the caller fall back. */
    vs_log_error(td->conf.modName,
                 "L1 camera path optimization needs a zoom budget: "
                 "use optzoom!=0 or zoom>0");
    vs_free(F); vs_free(B);
    return VS_ERROR;
  }
  /* The LP behind this can take a noticeable while on a long clip, and it runs
     after detection has already finished, so without a word here the tool looks
     hung.  Reported unconditionally for that reason. */
  vs_log_info(td->conf.modName,
              "Camera path optimization in progress (L1, %i frames, %s)...\n",
              N, vs_lp_backend_name());

  double objective = 0.0;
  int status = vsCameraPathOptimalL1LS(F, N, B, &conf, &objective);
  if (status == VS_OK) {
    if (td->conf.verbose & VS_DEBUG) {
      vs_log_msg(td->conf.modName, "L1 camera path objective: %g", objective);
    }
    for (int t = 0; t < N; t++) {
      int extra = trans->ts[t].extra;
      trans->ts[t] = updateToRenderTransform(&B[t], conf.cropRatio);
      trans->ts[t].extra = extra;
    }
    /* The inclusion constraints already bound the crop window exactly, and the
       zoom that goes with it is part of the transforms above.  Both the
       heuristic estimators and the global zoom in vsPreprocessTransforms()
       would only add more zoom on top of that, so mark the budget as spent:
       it went into the optimization, not on top of it. */
    td->conf.optZoom = 0;
    td->conf.zoom    = 0.0;
  }
  vs_free(F);
  vs_free(B);
  return status;
}

/*
 * Local variables:
 *   c-file-style: "stroustrup"
 *   c-file-offsets: ((case-label . *) (statement-case-intro . *))
 *   indent-tabs-mode: nil
 *   c-basic-offset: 2 t
 * End:
 *
 * vim: expandtab shiftwidth=2:
 */
