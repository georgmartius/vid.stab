/*
 *  lpsolver_ipm.c
 *
 *  Dependency-free backend for the LP interface in lpsolver.h: a primal-dual
 *  interior point method (Mehrotra predictor-corrector) whose normal equations
 *  are factorized with a banded Cholesky.
 *
 *  Why this and not a simplex: a simplex needs a sparse LU of the basis with
 *  pivoting and refactorization, which is a large piece of numerical software.
 *  An interior point method needs only one kind of factorization, always of the
 *  same symmetric positive definite matrix
 *
 *      A Theta A^T,   Theta > 0 diagonal,
 *
 *  and the camera path program is banded: a constraint at time t only involves
 *  the update transforms of t .. t+3, so with the rows ordered by time (see the
 *  layout comment in l1campathoptimization.c) the matrix above has a half
 *  bandwidth of about 130 regardless of the number of frames.  A banded
 *  Cholesky is then O(N) per iteration and about fifty lines.
 *
 *  Copyright (C) Georg Martius - 2026
 *   georg dot martius at web dot de
 *
 *  This file is part of vid.stab video stabilization library
 *
 *  vid.stab is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License,
 *  as published by the Free Software Foundation; either version 2, or
 *  (at your option) any later version.
 *
 *  vid.stab is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with GNU Make; see the file COPYING.  If not, write to
 *  the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include "lpsolver.h"

#ifdef USE_IPM

#include "vidstabdefines.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* The tuning constants are overridable so that they can be swept from a test
   harness without editing this file. */
#define IPM_MAXITER   200
/* accuracy at which the iteration stops early, having converged */
#ifndef IPM_TOL
#define IPM_TOL       1e-9
#endif
/* Theta = (Z/V + W/T)^-1 is clamped to this range: it legitimately goes to
   zero for the variables that end up at a bound and to infinity for the
   others, and without a clamp the normal equations overflow the exponent
   range long before the iteration is done */
#ifndef IPM_MINTHETA
#define IPM_MINTHETA  1e-14
#endif
#ifndef IPM_MAXTHETA
#define IPM_MAXTHETA  1e+14
#endif
/* Accuracy accepted when the iteration can no longer improve.
   Up to a few hundred frames the method reaches 1e-9 or better.  Beyond that
   the triangular solves accumulate enough round-off along the band that it
   plateaus around 1e-5.  Measured against an exact simplex solution at 200 and
   500 frames, an iterate at that level is strictly inside the inclusion and
   proximity constraints (by 1e-5 pixels) and its true objective is within 1e-4
   of the optimum, i.e. an equally smooth camera path.  1e-4 therefore leaves
   headroom without accepting anything that could be seen. */
#ifndef IPM_ACCEPT
#define IPM_ACCEPT    1e-4
#endif
/* relative shift on the diagonal of the normal equations, see bandCholesky */
#ifndef IPM_REG
#define IPM_REG       1e-11
#endif
/* refinement passes on the full Newton system, see ipmDirectionRefined */
#ifndef IPM_REFINE
#define IPM_REFINE    2
#endif

struct VSLinProg {
  int     numrows;
  int     numcols;
  int     maxentries;
  int     numentries;
  int*    ent_row;
  int*    ent_col;
  double* ent_val;
  double* rowlo;
  double* rowup;
  double* collo;
  double* colup;
  double* obj;
  double* sol;         // primal solution of the structural variables
  double  objval;
  int     solved;
  const char* status;
  char    statusbuf[96];
};

static int isinf_lp(double v){ return v >= VS_LP_INF || v <= -VS_LP_INF; }

/* ****************************************************************************
 * banded Cholesky
 *
 * The lower triangle of a symmetric matrix with half bandwidth bw is stored
 * column by column: band[j*(bw+1) + d] holds the element (j+d, j).
 * ************************************************************************** */

typedef struct {
  int     m;
  int     bw;
  double* dat;
} Band;

static double* bandAt(Band* b, int col, int d){ return &b->dat[(size_t)col * (b->bw + 1) + d]; }

/** in place Cholesky, L L^T = M.  The diagonal is scaled by 1 + `releps` as a
    regularization; a relative shift is essential here because the entries of
    Theta span many orders of magnitude near the solution, so any absolute
    shift is either meaningless or destroys the small rows.  If a pivot still
    comes out non-positive it is replaced by a tiny positive value: the result
    is then no longer an exact factorization, but iterative refinement on the
    normal equations recovers the accuracy. */
static void bandCholesky(Band* b, double releps){
  const int m = b->m, bw = b->bw;
  for (int j = 0; j < m; j++) {
    double d = *bandAt(b, j, 0) * (1.0 + releps);
    int p0 = j - bw; if (p0 < 0) p0 = 0;
    for (int p = p0; p < j; p++) {
      double v = *bandAt(b, p, j - p);
      d -= v * v;
    }
    if (!(d > 1e-300)) d = 1e-300;
    double piv = sqrt(d);
    *bandAt(b, j, 0) = piv;
    int imax = j + bw; if (imax > m - 1) imax = m - 1;
    for (int i = j + 1; i <= imax; i++) {
      double s = *bandAt(b, j, i - j);
      int q0 = i - bw; if (q0 < 0) q0 = 0;
      for (int p = q0; p < j; p++) {
        s -= *bandAt(b, p, i - p) * *bandAt(b, p, j - p);
      }
      *bandAt(b, j, i - j) = s / piv;
    }
  }
}

/** solves L L^T x = r in place on x (which may alias r) */
static void bandSolve(Band* b, const double* r, double* x){
  const int m = b->m, bw = b->bw;
  for (int i = 0; i < m; i++) {
    double s = r[i];
    int p0 = i - bw; if (p0 < 0) p0 = 0;
    for (int p = p0; p < i; p++) s -= *bandAt(b, p, i - p) * x[p];
    x[i] = s / *bandAt(b, i, 0);
  }
  for (int i = m - 1; i >= 0; i--) {
    double s = x[i];
    int pmax = i + bw; if (pmax > m - 1) pmax = m - 1;
    for (int p = i + 1; p <= pmax; p++) s -= *bandAt(b, i, p - i) * x[p];
    x[i] = s / *bandAt(b, i, 0);
  }
}

static double vecmaxabs_(const double* v, int n){
  double r = 0.0;
  for (int i = 0; i < n; i++) { double a = fabs(v[i]); if (a > r) r = a; }
  return r;
}

/* ****************************************************************************
 * problem construction
 * ************************************************************************** */

VSLinProg* vs_lp_new(const char* name, int numrows, int numcols, int maxentries){
  (void)name;
  if (numrows <= 0 || numcols <= 0 || maxentries <= 0) return 0;
  VSLinProg* p = (VSLinProg*)vs_zalloc(sizeof(VSLinProg));
  if (!p) return 0;
  p->numrows    = numrows;
  p->numcols    = numcols;
  p->maxentries = maxentries;
  p->status     = "";
  p->ent_row = (int*)   vs_malloc(sizeof(int)    * maxentries);
  p->ent_col = (int*)   vs_malloc(sizeof(int)    * maxentries);
  p->ent_val = (double*)vs_malloc(sizeof(double) * maxentries);
  p->rowlo   = (double*)vs_malloc(sizeof(double) * numrows);
  p->rowup   = (double*)vs_malloc(sizeof(double) * numrows);
  p->collo   = (double*)vs_malloc(sizeof(double) * numcols);
  p->colup   = (double*)vs_malloc(sizeof(double) * numcols);
  p->obj     = (double*)vs_zalloc(sizeof(double) * numcols);
  p->sol     = (double*)vs_zalloc(sizeof(double) * numcols);
  if (!p->ent_row || !p->ent_col || !p->ent_val || !p->rowlo || !p->rowup ||
      !p->collo || !p->colup || !p->obj || !p->sol) {
    vs_lp_free(p);
    return 0;
  }
  for (int i = 0; i < numrows; i++) { p->rowlo[i] = -VS_LP_INF; p->rowup[i] = VS_LP_INF; }
  for (int j = 0; j < numcols; j++) { p->collo[j] = -VS_LP_INF; p->colup[j] = VS_LP_INF; }
  return p;
}

void vs_lp_free(VSLinProg* p){
  if (!p) return;
  vs_free(p->ent_row); vs_free(p->ent_col); vs_free(p->ent_val);
  vs_free(p->rowlo); vs_free(p->rowup);
  vs_free(p->collo); vs_free(p->colup);
  vs_free(p->obj);   vs_free(p->sol);
  vs_free(p);
}

void vs_lp_set_row_bounds(VSLinProg* p, int row, double lo, double up){
  if (!p || row < 0 || row >= p->numrows) return;
  p->rowlo[row] = lo; p->rowup[row] = up;
}

void vs_lp_set_col_bounds(VSLinProg* p, int col, double lo, double up){
  if (!p || col < 0 || col >= p->numcols) return;
  p->collo[col] = lo; p->colup[col] = up;
}

void vs_lp_set_obj(VSLinProg* p, int col, double coef){
  if (!p || col < 0 || col >= p->numcols) return;
  p->obj[col] = coef;
}

void vs_lp_add_entry(VSLinProg* p, int row, int col, double val){
  if (!p) return;
  if (row < 0 || row >= p->numrows || col < 0 || col >= p->numcols) return;
  if (p->numentries >= p->maxentries) { p->status = "matrix entry buffer overflow"; return; }
  int k = p->numentries++;
  p->ent_row[k] = row; p->ent_col[k] = col; p->ent_val[k] = val;
}

/* ****************************************************************************
 * the solver
 *
 * After normalization every variable v (the structural ones followed by one
 * slack per row) has a finite lower bound, so shifting by it puts the program
 * into
 *
 *      min c^T v   s.t.  A v = b,   0 <= v <= h,   h_j possibly infinite.
 *
 * The primal-dual system is the textbook one:
 *
 *      A v = b,  v + t = h,  A^T y + z - w = c,  V Z e = 0,  T W e = 0
 *
 * with z, w >= 0 and w_j = 0 wherever h_j is infinite.  Eliminating everything
 * but y leaves (A Theta A^T) dy = rhs with Theta = (Z/V + W/T)^-1.
 * ************************************************************************** */

typedef struct {
  int      m;          // equality rows
  int      nv;         // variables: structural + one slack per row
  int      n;          // structural variables
  int*     colptr;     // CSC of the structural part of A, size n+1
  int*     rowidx;
  double*  val;
  double*  b;
  double*  c;
  double*  lo;         // shift that was applied
  double*  h;          // upper bound of the shifted variable, VS_LP_INF if none
  double*  scale;      // equilibration factor: true value = lo + scale * v
  double   objscale;   // the objective was divided by this
  signed char* sign;   // -1 if the variable was negated during normalization
} IpmProb;

static void ipmProbFree(IpmProb* q){
  vs_free(q->colptr); vs_free(q->rowidx); vs_free(q->val);
  vs_free(q->b); vs_free(q->c); vs_free(q->lo); vs_free(q->h);
  vs_free(q->scale); vs_free(q->sign);
  memset(q, 0, sizeof(*q));
}

/** Equilibrates the program.  The columns of the camera path matrix differ by
    six orders of magnitude -- a translation is measured in pixels while a and b
    are of order one -- and the objective coefficients by four.  Without this
    the normal equations are too ill conditioned to reach optimality on longer
    sequences.

    Substituting v_j = scale_j * v~_j and multiplying row i by r_i keeps the
    slack columns equal to -e_i if the slack of row i is scaled by 1/r_i. */
static void ipmEquilibrate(IpmProb* q){
  const int n = q->n, m = q->m;
  double* r = (double*)vs_malloc(sizeof(double) * m);
  if (!r) return;

  for (int j = 0; j < q->nv; j++) q->scale[j] = 1.0;
  for (int i = 0; i < m; i++) r[i] = 1.0;

  for (int pass = 0; pass < 3; pass++) {
    /* columns: largest entry of every structural column becomes 1 */
    for (int j = 0; j < n; j++) {
      double mx = 0.0;
      for (int k = q->colptr[j]; k < q->colptr[j + 1]; k++) {
        double a = fabs(q->val[k]);
        if (a > mx) mx = a;
      }
      if (mx <= 0.0) continue;
      double s = 1.0 / mx;
      for (int k = q->colptr[j]; k < q->colptr[j + 1]; k++) q->val[k] *= s;
      q->scale[j] *= s;
      q->c[j] *= s;
      if (!isinf_lp(q->h[j])) q->h[j] /= s;
    }
    /* rows: largest entry of every row becomes 1 */
    for (int i = 0; i < m; i++) r[i] = 0.0;
    for (int j = 0; j < n; j++)
      for (int k = q->colptr[j]; k < q->colptr[j + 1]; k++) {
        double a = fabs(q->val[k]);
        if (a > r[q->rowidx[k]]) r[q->rowidx[k]] = a;
      }
    for (int i = 0; i < m; i++) r[i] = (r[i] > 0.0) ? 1.0 / r[i] : 1.0;
    for (int j = 0; j < n; j++)
      for (int k = q->colptr[j]; k < q->colptr[j + 1]; k++)
        q->val[k] *= r[q->rowidx[k]];
    for (int i = 0; i < m; i++) {
      int j = n + i;
      q->b[i] *= r[i];
      q->scale[j] /= r[i];
      if (!isinf_lp(q->h[j])) q->h[j] *= r[i];
    }
  }

  /* finally bring the objective to order one */
  q->objscale = vecmaxabs_(q->c, q->nv);
  if (!(q->objscale > 0.0)) q->objscale = 1.0;
  for (int j = 0; j < q->nv; j++) q->c[j] /= q->objscale;
  vs_free(r);
}

/** brings the problem into the form described above.  Returns VS_ERROR if a
    variable or a row is free on both sides, which this backend does not
    support (the camera path program never produces one). */
static int ipmBuild(VSLinProg* p, IpmProb* q){
  const int m = p->numrows, n = p->numcols;
  memset(q, 0, sizeof(*q));
  q->m = m; q->n = n; q->nv = n + m; q->objscale = 1.0;

  q->lo    = (double*)vs_malloc(sizeof(double) * q->nv);
  q->h     = (double*)vs_malloc(sizeof(double) * q->nv);
  q->scale = (double*)vs_malloc(sizeof(double) * q->nv);
  q->c     = (double*)vs_zalloc(sizeof(double) * q->nv);
  q->b     = (double*)vs_zalloc(sizeof(double) * m);
  q->sign  = (signed char*)vs_malloc(sizeof(signed char) * q->nv);
  q->colptr = (int*)vs_zalloc(sizeof(int) * (n + 1));
  q->rowidx = (int*)vs_malloc(sizeof(int) * (p->numentries > 0 ? p->numentries : 1));
  q->val    = (double*)vs_malloc(sizeof(double) * (p->numentries > 0 ? p->numentries : 1));
  if (!q->lo || !q->h || !q->scale || !q->c || !q->b || !q->sign || !q->colptr ||
      !q->rowidx || !q->val) { ipmProbFree(q); return VS_ERROR; }

  /* structural variables: negate the ones that are bounded from above only */
  for (int j = 0; j < n; j++) {
    double l = p->collo[j], u = p->colup[j];
    if (isinf_lp(l) && isinf_lp(u)) { ipmProbFree(q); return VS_ERROR; }
    if (isinf_lp(l)) {
      q->sign[j] = -1;
      q->lo[j] = -u;
      q->h[j]  = VS_LP_INF;
      q->c[j]  = -p->obj[j];
    } else {
      q->sign[j] = 1;
      q->lo[j] = l;
      q->h[j]  = isinf_lp(u) ? VS_LP_INF : (u - l);
      q->c[j]  = p->obj[j];
    }
  }
  /* row slacks: A_i x - s_i = 0 with s_i between the row bounds; a row that is
     bounded from above only is negated, together with its entries */
  signed char* rowsign = (signed char*)vs_malloc(sizeof(signed char) * m);
  if (!rowsign) { ipmProbFree(q); return VS_ERROR; }
  for (int i = 0; i < m; i++) {
    double l = p->rowlo[i], u = p->rowup[i];
    if (isinf_lp(l) && isinf_lp(u)) { vs_free(rowsign); ipmProbFree(q); return VS_ERROR; }
    int j = n + i;
    q->sign[j] = 1;
    if (isinf_lp(l)) {
      rowsign[i] = -1;
      q->lo[j] = -u;
      q->h[j]  = VS_LP_INF;
    } else {
      rowsign[i] = 1;
      q->lo[j] = l;
      q->h[j]  = isinf_lp(u) ? VS_LP_INF : (u - l);
    }
  }

  /* CSC of A, with the row and column signs folded in */
  for (int k = 0; k < p->numentries; k++) q->colptr[p->ent_col[k] + 1]++;
  for (int j = 0; j < n; j++) q->colptr[j + 1] += q->colptr[j];
  int* fill = (int*)vs_malloc(sizeof(int) * n);
  if (!fill) { vs_free(rowsign); ipmProbFree(q); return VS_ERROR; }
  for (int j = 0; j < n; j++) fill[j] = q->colptr[j];
  for (int k = 0; k < p->numentries; k++) {
    int j = p->ent_col[k], i = p->ent_row[k];
    int pos = fill[j]++;
    q->rowidx[pos] = i;
    q->val[pos] = p->ent_val[k] * rowsign[i] * q->sign[j];
  }
  vs_free(fill);
  vs_free(rowsign);

  /* A(v + lo) - (s + lo_s) = 0  =>  A v - s = lo_s - A lo */
  for (int j = 0; j < n; j++) {
    for (int k = q->colptr[j]; k < q->colptr[j + 1]; k++)
      q->b[q->rowidx[k]] -= q->val[k] * q->lo[j];
  }
  for (int i = 0; i < m; i++) q->b[i] += q->lo[n + i];

  ipmEquilibrate(q);
  return VS_OK;
}

/** res = A_struct * v_struct, the structural block alone */
static void ipmAxStruct(const IpmProb* q, const double* v, double* res){
  memset(res, 0, sizeof(double) * q->m);
  for (int j = 0; j < q->n; j++) {
    double vj = v[j];
    if (vj == 0.0) continue;
    for (int k = q->colptr[j]; k < q->colptr[j + 1]; k++)
      res[q->rowidx[k]] += q->val[k] * vj;
  }
}

/** res = A_struct * v_struct - v_slack  (the full A times v) */
static void ipmAv(const IpmProb* q, const double* v, double* res){
  ipmAxStruct(q, v, res);
  for (int i = 0; i < q->m; i++) res[i] -= v[q->n + i];
}

/** res = A^T y */
static void ipmATy(const IpmProb* q, const double* y, double* res){
  for (int j = 0; j < q->n; j++) {
    double s = 0.0;
    for (int k = q->colptr[j]; k < q->colptr[j + 1]; k++)
      s += q->val[k] * y[q->rowidx[k]];
    res[j] = s;
  }
  for (int i = 0; i < q->m; i++) res[q->n + i] = -y[i];
}

/** half bandwidth of A Theta A^T, i.e. the largest row span of any column */
static int ipmBandwidth(const IpmProb* q){
  int bw = 0;
  for (int j = 0; j < q->n; j++) {
    if (q->colptr[j] == q->colptr[j + 1]) continue;
    int lo = q->rowidx[q->colptr[j]], hi = lo;
    for (int k = q->colptr[j] + 1; k < q->colptr[j + 1]; k++) {
      int i = q->rowidx[k];
      if (i < lo) lo = i;
      if (i > hi) hi = i;
    }
    if (hi - lo > bw) bw = hi - lo;
  }
  return bw;
}

/** assembles A Theta A^T into the band; the slack part contributes only to the
    diagonal because those columns of A are -e_i */
static void ipmAssemble(const IpmProb* q, const double* theta, Band* band){
  memset(band->dat, 0, sizeof(double) * (size_t)band->m * (band->bw + 1));
  for (int j = 0; j < q->n; j++) {
    double th = theta[j];
    if (th == 0.0) continue;
    for (int k1 = q->colptr[j]; k1 < q->colptr[j + 1]; k1++) {
      int i1 = q->rowidx[k1];
      double v1 = th * q->val[k1];
      for (int k2 = q->colptr[j]; k2 < q->colptr[j + 1]; k2++) {
        int i2 = q->rowidx[k2];
        if (i2 < i1) continue;          // lower triangle: store (i2, i1) with i2 >= i1
        *bandAt(band, i1, i2 - i1) += v1 * q->val[k2];
      }
    }
  }
  for (int i = 0; i < q->m; i++) *bandAt(band, i, 0) += theta[q->n + i];
}

/** res = A Theta A^T x.  tmp must have room for nv doubles. */
static void ipmNormalMul(const IpmProb* q, const double* theta, const double* x,
                         double* res, double* tmp){
  ipmATy(q, x, tmp);
  for (int j = 0; j < q->nv; j++) tmp[j] *= theta[j];
  ipmAv(q, tmp, res);
}

/** solves (A Theta A^T) sol = rhs using the banded Cholesky as a
    preconditioner, with a few steps of iterative refinement.  The normal
    equations are badly conditioned by construction -- Theta goes to zero for
    the variables that end up at a bound and to infinity for the others -- so a
    plain triangular solve stalls at a primal residual around 1e-6, far short of
    what is needed to declare optimality. */
static void ipmSolveRefined(Band* band, const IpmProb* q, const double* theta,
                            const double* rhs, double* sol,
                            double* work_m, double* work_nv, double* corr_m){
  const int m = q->m;
  bandSolve(band, rhs, sol);
  for (int it = 0; it < 5; it++) {
    ipmNormalMul(q, theta, sol, work_m, work_nv);
    double rmax = 0.0, bmax = 0.0;
    for (int i = 0; i < m; i++) {
      work_m[i] = rhs[i] - work_m[i];
      double a = fabs(work_m[i]); if (a > rmax) rmax = a;
      double c = fabs(rhs[i]);    if (c > bmax) bmax = c;
    }
    if (rmax <= 1e-15 * (bmax + 1.0)) break;
    bandSolve(band, work_m, corr_m);
    for (int i = 0; i < m; i++) sol[i] += corr_m[i];
  }
}

/* ****************************************************************************
 * one Newton direction
 *
 * The system solved here is
 *
 *      A dv                        = e1
 *      dv + dt                     = e2      (only where h is finite)
 *      A^T dy + dz - dw            = e3
 *      Z dv + V dz                 = e4
 *      W dt + T dw                 = e5      (only where h is finite)
 *
 * which reduces, with Theta = (Z/V + W/T)^-1, to
 *
 *      g  = e3 - e4/v + e5/t - w e2/t
 *      (A Theta A^T) dy = e1 + A Theta g
 *      dv = Theta (A^T dy - g)
 * ************************************************************************** */

typedef struct {
  const IpmProb* q;
  Band*          band;
  const double*  theta;
  const double*  v;
  const double*  t;
  const double*  z;
  const double*  w;
  double*        gbuf;   // nv
  double*        tmpn;   // nv
  double*        tmpm;   // m
  double*        wrk1;   // m
  double*        wrk2;   // m
  double*        wrk3;   // nv
} IpmCtx;

static void ipmDirection(IpmCtx* C,
                         const double* e1, const double* e2, const double* e3,
                         const double* e4, const double* e5,
                         double* dv, double* dy, double* dz,
                         double* dt, double* dw){
  const IpmProb* q = C->q;
  const int m = q->m, nv = q->nv;
  for (int j = 0; j < nv; j++) {
    double gj = e3[j] - e4[j] / C->v[j];
    if (!isinf_lp(q->h[j])) gj += (e5[j] - C->w[j] * e2[j]) / C->t[j];
    C->gbuf[j] = gj;
    C->tmpn[j] = C->theta[j] * gj;
  }
  ipmAv(q, C->tmpn, C->tmpm);
  for (int i = 0; i < m; i++) C->tmpm[i] += e1[i];
  ipmSolveRefined(C->band, q, C->theta, C->tmpm, dy, C->wrk1, C->wrk3, C->wrk2);
  ipmATy(q, dy, C->tmpn);
  for (int j = 0; j < nv; j++) {
    dv[j] = C->theta[j] * (C->tmpn[j] - C->gbuf[j]);
    dz[j] = (e4[j] - C->z[j] * dv[j]) / C->v[j];
    if (isinf_lp(q->h[j])) { dt[j] = 0.0; dw[j] = 0.0; }
    else {
      dt[j] = e2[j] - dv[j];
      dw[j] = (e5[j] - C->w[j] * dt[j]) / C->t[j];
    }
  }
}

/** Same, followed by iterative refinement on the *full* Newton system.

    Refining only the normal equations is not enough: near the solution Theta
    spans close to thirty orders of magnitude, and the recovery
    dv = Theta (A^T dy - g) then loses so much accuracy that the primal residual
    of the whole system sticks around 1e-6 no matter how exactly dy was
    computed.  Evaluating the five equations above in their original scaling and
    solving again for the correction brings it down to round-off, which is what
    finally lets the method reach a duality gap below 1e-9. */
typedef struct {
  double *f1, *f2, *f3, *f4, *f5;
  double *cv, *cy, *cz, *ct, *cw;
} IpmRefine;

static void ipmDirectionRefined(IpmCtx* C, IpmRefine* R,
                                const double* e1, const double* e2, const double* e3,
                                const double* e4, const double* e5,
                                double* dv, double* dy, double* dz,
                                double* dt, double* dw){
  const IpmProb* q = C->q;
  const int m = q->m, nv = q->nv;
  ipmDirection(C, e1, e2, e3, e4, e5, dv, dy, dz, dt, dw);
  for (int pass = 0; pass < IPM_REFINE; pass++) {
    ipmAv(q, dv, C->tmpm);
    for (int i = 0; i < m; i++) R->f1[i] = e1[i] - C->tmpm[i];
    ipmATy(q, dy, C->tmpn);
    for (int j = 0; j < nv; j++) {
      R->f3[j] = e3[j] - C->tmpn[j] - dz[j] + dw[j];
      R->f4[j] = e4[j] - C->z[j] * dv[j] - C->v[j] * dz[j];
      if (isinf_lp(q->h[j])) { R->f2[j] = 0.0; R->f5[j] = 0.0; }
      else {
        R->f2[j] = e2[j] - dv[j] - dt[j];
        R->f5[j] = e5[j] - C->w[j] * dt[j] - C->t[j] * dw[j];
      }
    }
    ipmDirection(C, R->f1, R->f2, R->f3, R->f4, R->f5,
                 R->cv, R->cy, R->cz, R->ct, R->cw);
    for (int j = 0; j < nv; j++) {
      dv[j] += R->cv[j]; dz[j] += R->cz[j];
      dt[j] += R->ct[j]; dw[j] += R->cw[j];
    }
    for (int i = 0; i < m; i++) dy[i] += R->cy[i];
  }
}

/** largest step that keeps every component non-negative */
static void ipmStepLengths(const IpmProb* q, const double* v, const double* t,
                           const double* z, const double* w,
                           const double* dv, const double* dt,
                           const double* dz, const double* dw,
                           double* alpha_p, double* alpha_d){
  double ap = 1.0, ad = 1.0;
  for (int j = 0; j < q->nv; j++) {
    if (dv[j] < 0) { double a = -v[j] / dv[j]; if (a < ap) ap = a; }
    if (dz[j] < 0) { double a = -z[j] / dz[j]; if (a < ad) ad = a; }
    if (!isinf_lp(q->h[j])) {
      if (dt[j] < 0) { double a = -t[j] / dt[j]; if (a < ap) ap = a; }
      if (dw[j] < 0) { double a = -w[j] / dw[j]; if (a < ad) ad = a; }
    }
  }
  *alpha_p = ap; *alpha_d = ad;
}

int vs_lp_solve(VSLinProg* p, int verbose){
  if (!p) return VS_ERROR;
  p->solved = 0;
  if (p->status && p->status[0]) return VS_ERROR;   // e.g. entry buffer overflow

  IpmProb q;
  if (ipmBuild(p, &q) != VS_OK) {
    p->status = "problem contains a variable or row that is free on both sides";
    return VS_ERROR;
  }
  const int m = q.m, nv = q.nv;

  int bw = ipmBandwidth(&q);
  /* Guard against a program whose rows are not ordered so that the matrix is
     banded: the band would degenerate into a dense matrix and exhaust memory. */
  if ((double)(bw + 1) * m * (double)sizeof(double) > 2e9) {
    p->status = "constraint matrix is not banded enough for this backend";
    ipmProbFree(&q);
    return VS_ERROR;
  }
  Band band;
  band.m = m; band.bw = bw;
  band.dat = (double*)vs_malloc(sizeof(double) * (size_t)m * (bw + 1));

  /* one block for all working vectors, so the cleanup stays a single free */
  const int nvecs_nv = 25, nvecs_m = 8;
  double* pool = (double*)vs_zalloc(sizeof(double) *
                                    ((size_t)nvecs_nv * nv + (size_t)nvecs_m * m));
  if (!band.dat || !pool) {
    p->status = "out of memory";
    vs_free(band.dat); vs_free(pool); ipmProbFree(&q);
    return VS_ERROR;
  }
  double* pn = pool;
#define TAKE_NV(name) double* name = pn; pn += nv;
  TAKE_NV(v)    TAKE_NV(t)    TAKE_NV(z)    TAKE_NV(w)
  TAKE_NV(theta) TAKE_NV(dv)  TAKE_NV(dt)   TAKE_NV(dz)
  TAKE_NV(dw)   TAKE_NV(rd)   TAKE_NV(ru)   TAKE_NV(rvz)
  TAKE_NV(rtw)  TAKE_NV(best) TAKE_NV(gbuf) TAKE_NV(tmpn)
  TAKE_NV(wrk3) TAKE_NV(f2)   TAKE_NV(f3)   TAKE_NV(f4)
  TAKE_NV(f5)   TAKE_NV(cv)   TAKE_NV(cz)   TAKE_NV(ct)
  TAKE_NV(cw)
#undef TAKE_NV
  double* pm = pn;
#define TAKE_M(name) double* name = pm; pm += m;
  TAKE_M(y)  TAKE_M(dy) TAKE_M(rp)   TAKE_M(tmpm)
  TAKE_M(wrk1) TAKE_M(wrk2) TAKE_M(f1) TAKE_M(cy)
#undef TAKE_M

  /* --- starting point ---------------------------------------------------- */
  for (int j = 0; j < nv; j++) {
    if (isinf_lp(q.h[j])) {
      v[j] = 1.0;
      t[j] = VS_LP_INF;
      w[j] = 0.0;
    } else {
      v[j] = 0.5 * q.h[j];
      if (v[j] < 1e-6) v[j] = 1e-6;
      t[j] = q.h[j] - v[j];
      if (t[j] < 1e-6) t[j] = 1e-6;
      w[j] = 1.0;
    }
    z[j] = 1.0;
  }

  IpmCtx C;
  C.q = &q; C.band = &band; C.theta = theta;
  C.v = v; C.t = t; C.z = z; C.w = w;
  C.gbuf = gbuf; C.tmpn = tmpn; C.tmpm = tmpm;
  C.wrk1 = wrk1; C.wrk2 = wrk2; C.wrk3 = wrk3;
  IpmRefine R;
  R.f1 = f1; R.f2 = f2; R.f3 = f3; R.f4 = f4; R.f5 = f5;
  R.cv = cv; R.cy = cy; R.cz = cz; R.ct = ct; R.cw = cw;

  const double cnorm = 1.0 + vecmaxabs_(q.c, nv);
  const double bnorm = 1.0 + vecmaxabs_(q.b, m);
  double bestmerit = 1e300;
  int    bestiter = -1;
  int    stall = 0;

  int itersdone = 0;
  for (int iter = 0; iter < IPM_MAXITER; iter++) {
    itersdone = iter;
    /* --- residuals and convergence --------------------------------------- */
    ipmAv(&q, v, tmpm);
    for (int i = 0; i < m; i++) rp[i] = q.b[i] - tmpm[i];
    ipmATy(&q, y, tmpn);
    int npairs = 0;
    double comp = 0.0, primalobj = 0.0, dualobj = 0.0;
    for (int j = 0; j < nv; j++) {
      rd[j] = q.c[j] - tmpn[j] - z[j] + w[j];
      ru[j] = isinf_lp(q.h[j]) ? 0.0 : (q.h[j] - v[j] - t[j]);
      comp += v[j] * z[j];
      npairs++;
      primalobj += q.c[j] * v[j];
      if (!isinf_lp(q.h[j])) { comp += t[j] * w[j]; npairs++; dualobj -= q.h[j] * w[j]; }
    }
    for (int i = 0; i < m; i++) dualobj += q.b[i] * y[i];
    double mu = comp / (double)npairs;

    double perr = vecmaxabs_(rp, m)  / bnorm;
    double uerr = vecmaxabs_(ru, nv) / bnorm;
    double derr = vecmaxabs_(rd, nv) / cnorm;
    double gap  = fabs(primalobj - dualobj) / (1.0 + fabs(primalobj));
    double merit = perr;
    if (uerr > merit) merit = uerr;
    if (derr > merit) merit = derr;
    if (gap  > merit) merit = gap;

    if (verbose) {
      fprintf(stderr, "  ipm %3i  mu %9.2e  primal %9.2e  dual %9.2e  bnd %9.2e"
              "  gap %9.2e\n", iter, mu, perr, derr, uerr, gap);
    }
    if (merit < bestmerit * (1.0 - 1e-3)) {
      bestmerit = merit;
      bestiter  = iter;
      memcpy(best, v, sizeof(double) * nv);
      stall = 0;
    } else {
      stall++;
    }
    if (merit < IPM_TOL) break;
    /* Once the iterates stop improving there is nothing left to gain: the
       normal equations have become too ill conditioned for double precision.
       Continuing reliably ends in a numerical blow-up, so stop and keep the
       best point seen. */
    if (stall >= 6 || merit > 1e4 * bestmerit) break;

    /* --- factorization --------------------------------------------------- */
    for (int j = 0; j < nv; j++) {
      double d = z[j] / v[j];
      if (!isinf_lp(q.h[j])) d += w[j] / t[j];
      double th = 1.0 / d;
      if (!(th > IPM_MINTHETA)) th = IPM_MINTHETA;
      if (th > IPM_MAXTHETA)    th = IPM_MAXTHETA;
      theta[j] = th;
    }
    ipmAssemble(&q, theta, &band);
    bandCholesky(&band, IPM_REG);

    /* --- predictor ------------------------------------------------------- */
    for (int j = 0; j < nv; j++) {
      rvz[j] = -v[j] * z[j];
      rtw[j] = isinf_lp(q.h[j]) ? 0.0 : -t[j] * w[j];
    }
    ipmDirectionRefined(&C, &R, rp, ru, rd, rvz, rtw, dv, dy, dz, dt, dw);

    double ap, ad;
    ipmStepLengths(&q, v, t, z, w, dv, dt, dz, dw, &ap, &ad);
    double compaff = 0.0;
    for (int j = 0; j < nv; j++) {
      compaff += (v[j] + ap * dv[j]) * (z[j] + ad * dz[j]);
      if (!isinf_lp(q.h[j])) compaff += (t[j] + ap * dt[j]) * (w[j] + ad * dw[j]);
    }
    double sigma = (compaff / (double)npairs) / mu;
    sigma = sigma * sigma * sigma;
    if (sigma > 1.0)  sigma = 1.0;
    if (sigma < 1e-9) sigma = 1e-9;

    /* --- corrector, reusing the factorization ---------------------------- */
    for (int j = 0; j < nv; j++) {
      rvz[j] = sigma * mu - v[j] * z[j] - dv[j] * dz[j];
      rtw[j] = isinf_lp(q.h[j]) ? 0.0
                                : (sigma * mu - t[j] * w[j] - dt[j] * dw[j]);
    }
    ipmDirectionRefined(&C, &R, rp, ru, rd, rvz, rtw, dv, dy, dz, dt, dw);

    /* --- step ------------------------------------------------------------ */
    ipmStepLengths(&q, v, t, z, w, dv, dt, dz, dw, &ap, &ad);
    const double frac = 0.9995;
    ap *= frac; ad *= frac;
    if (ap > 1.0) ap = 1.0;
    if (ad > 1.0) ad = 1.0;
    if (ap < 1e-14 || ad < 1e-14) break;   // stuck against the boundary

    for (int j = 0; j < nv; j++) {
      v[j] += ap * dv[j];
      z[j] += ad * dz[j];
      if (!isinf_lp(q.h[j])) { t[j] += ap * dt[j]; w[j] += ad * dw[j]; }
    }
    for (int i = 0; i < m; i++) y[i] += ad * dy[i];
  }

  if (bestmerit <= IPM_ACCEPT) {
    p->objval = 0.0;
    for (int j = 0; j < p->numcols; j++) {
      double val = q.lo[j] + q.scale[j] * best[j];
      if (q.sign[j] < 0) val = -val;
      p->sol[j] = val;
      p->objval += p->obj[j] * val;
    }
    p->solved = 1;
    p->status = "";
    if (verbose) {
      fprintf(stderr, "  ipm finished after %i iterations, best iterate %i,"
              " accuracy %.3e\n", itersdone, bestiter, bestmerit);
    }
  } else {
    snprintf(p->statusbuf, sizeof(p->statusbuf),
             "interior point method stalled at an accuracy of %.2e", bestmerit);
    p->status = p->statusbuf;
  }

  vs_free(band.dat);
  vs_free(pool);
  ipmProbFree(&q);
  return p->solved ? VS_OK : VS_ERROR;
}

double vs_lp_get_obj_value(const VSLinProg* p){
  return (p && p->solved) ? p->objval : 0.0;
}

double vs_lp_get_col_value(const VSLinProg* p, int col){
  if (!p || !p->solved || col < 0 || col >= p->numcols) return 0.0;
  return p->sol[col];
}

const char* vs_lp_status_msg(const VSLinProg* p){
  return (p && p->status) ? p->status : "";
}

const char* vs_lp_backend_name(void){
  return "built-in interior point";
}

#endif /* USE_IPM */

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
