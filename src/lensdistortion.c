/*
 *  lensdistortion.c
 *
 *  See lensdistortion.h and docs/lens-distortion.md.
 *
 *  Copyright (C) Georg Martius - 2026
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
#include "lensdistortion.h"
#include "vidstabdefines.h"
#include "transformtype_operations.h"

#include <math.h>
#include <stdlib.h>

VSLensDistortion vsLensDistortionInit(const VSFrameInfo* fi, double k){
  VSLensDistortion ld;
  ld.k   = k;
  ld.cx  = fi->width  / 2.0;
  ld.cy  = fi->height / 2.0;
  /* half the diagonal, so a normalised radius of 1 lands on the image corner
     and k is independent of the encoded resolution */
  ld.rho = 0.5 * sqrt((double)fi->width * fi->width +
                      (double)fi->height * fi->height);
  return ld;
}

int vsLensUndistortPoint(const VSLensDistortion* ld,
                         double xi, double yi, double* xo, double* yo){
  double dx = xi - ld->cx;
  double dy = yi - ld->cy;
  /* squared radius in units of rho */
  double r2 = (dx*dx + dy*dy) / (ld->rho * ld->rho);
  double denom = 1.0 + ld->k * r2;
  /* only reachable for pincushion strong enough to fold the plane onto itself */
  if(denom <= 0) return VS_ERROR;
  *xo = ld->cx + dx/denom;
  *yo = ld->cy + dy/denom;
  return VS_OK;
}

int vsLensDistortPoint(const VSLensDistortion* ld,
                       double xi, double yi, double* xo, double* yo){
  double dx = xi - ld->cx;
  double dy = yi - ld->cy;
  double r2 = (dx*dx + dy*dy) / (ld->rho * ld->rho);
  double disc = 1.0 - 4.0 * ld->k * r2;
  double scale;
  if(disc < 0) return VS_ERROR;
  /* Rationalised root of k*r_u*r_d^2 - r_d + r_u = 0.  Written as
     2/(1+sqrt(disc)) rather than (1-sqrt(disc))/(2*k*r_u^2) so that it stays
     accurate for small k and is exactly 1 at k=0, with no special case. */
  scale = 2.0 / (1.0 + sqrt(disc));
  *xo = ld->cx + dx*scale;
  *yo = ld->cy + dy*scale;
  return VS_OK;
}

/* Jacobian of the distortion map D at the centred point (wx,wy), row major.

   D acts radially: in units of rho it is u -> u*g(t) with t=|u|^2 and
   g(t) = 2/(1+sqrt(1-4kt)), so the derivative is g*I + 2g'*u*u^T.  The rho
   scaling cancels, because D in pixels is rho*Dnorm(w/rho). */
static int lensDistortJacobian(const VSLensDistortion* ld,
                               double wx, double wy, double J[4]){
  double inv = 1.0/ld->rho;
  double ux = wx*inv, uy = wy*inv;
  double t  = ux*ux + uy*uy;
  double disc = 1.0 - 4.0*ld->k*t;
  double root, g, gp;
  if(disc < 0) return VS_ERROR;
  root = sqrt(disc);
  g  = 2.0/(1.0 + root);
  /* dg/dt; the root in the denominator is why t=0 with large positive k is the
     only place this needs the domain guard above */
  gp = (root > 1e-12) ? (4.0*ld->k) / ((1.0+root)*(1.0+root)*root) : 0.0;
  J[0] = g + 2.0*gp*ux*ux;
  J[1] =     2.0*gp*ux*uy;
  J[2] =     2.0*gp*uy*ux;
  J[3] = g + 2.0*gp*uy*uy;
  return VS_OK;
}

/* Solves the symmetric 4x4 system A x = b in place by Gaussian elimination with
   partial pivoting.  Returns VS_ERROR when the system is singular, which for
   these normal equations means the matches do not constrain the similarity. */
static int lensSolve4(double A[16], double b[4], double x[4]){
  int i, j, p, q;
  for(i=0; i<4; i++){
    p = i;
    for(j=i+1; j<4; j++) if(fabs(A[j*4+i]) > fabs(A[p*4+i])) p = j;
    if(fabs(A[p*4+i]) < 1e-12) return VS_ERROR;
    if(p != i){
      for(q=0; q<4; q++){ double t = A[i*4+q]; A[i*4+q] = A[p*4+q]; A[p*4+q] = t; }
      { double t = b[i]; b[i] = b[p]; b[p] = t; }
    }
    for(j=i+1; j<4; j++){
      double f = A[j*4+i]/A[i*4+i];
      if(f == 0) continue;
      for(q=i; q<4; q++) A[j*4+q] -= f*A[i*4+q];
      b[j] -= f*b[i];
    }
  }
  for(i=3; i>=0; i--){
    double sum = b[i];
    for(q=i+1; q<4; q++) sum -= A[i*4+q]*x[q];
    x[i] = sum/A[i*4+i];
  }
  return VS_OK;
}

/* The forward camera map on a CENTRED point: where a source point ends up
   before the lens redistorts it.  Every model site in this file goes through
   this one struct so that the fit, its Jacobian and the residual used for
   outlier rejection cannot drift apart.

   f <= 0 is the similarity model.  f > 0 is the rotational one, reading the
   same four parameters:
   (tx,ty) are yaw and pitch scaled by f, and (c,s) carry roll and zoom as
   z*cos(alpha), z*sin(alpha) -- the same reinterpretation prepare_transform_fov
   makes, so the VSTransform this fit produces means the same thing to the
   rest of the library either way. */
typedef struct {
  double f;
  double c, s, tx, ty;   /* the similarity parameters, used when f <= 0 */
  double z;              /* sqrt(c^2+s^2), used when f > 0 */
  double rf[9];          /* forward rotation, used when f > 0 */
} LensFwd;

static void lensFwdInit(LensFwd* w, double f, double tx, double ty,
                        double c, double s){
  w->f = f; w->c = c; w->s = s; w->tx = tx; w->ty = ty;
  w->z = 1.0;
  if(f > 0.0){
    double b[9];
    w->z = sqrt(c*c + s*s);
    rotation_matrix_backward(tx/f, ty/f, atan2(s, c), b);
    /* forward = backward^-1 = backward^T */
    w->rf[0]=b[0]; w->rf[1]=b[3]; w->rf[2]=b[6];
    w->rf[3]=b[1]; w->rf[4]=b[4]; w->rf[5]=b[7];
    w->rf[6]=b[2]; w->rf[7]=b[5]; w->rf[8]=b[8];
  }
}

static void lensFwdApply(const LensFwd* w, double ax, double ay,
                         double* wx, double* wy){
  if(w->f > 0.0){
    double X = w->rf[0]*ax + w->rf[1]*ay + w->rf[2]*w->f;
    double Y = w->rf[3]*ax + w->rf[4]*ay + w->rf[5]*w->f;
    double Z = w->rf[6]*ax + w->rf[7]*ay + w->rf[8]*w->f;
    *wx = w->z * w->f * X/Z;
    *wy = w->z * w->f * Y/Z;
    return;
  }
  *wx =  w->c*ax + w->s*ay + w->tx;
  *wy = -w->s*ax + w->c*ay + w->ty;
}

/* Finite-difference steps for d(forward map)/d(parameter) on the rotational
   path.  Only the ROTATION is differenced -- the distortion Jacobian stays
   analytic and is applied to the result -- so this costs four evaluations of
   nine multiplies and a divide per point, not four distortion evaluations.
   tx,ty are pixels (order 10), c,s are dimensionless (order 1); both steps
   sit far above double's noise floor for the derivative magnitudes here
   (~1e-14 px) and far below the scale on which the map curves. */
#define LENS_H_T   1e-3
#define LENS_H_CS  1e-6

int vsLensFitSimilarity(const VSLensDistortion* ld, const VSPointMatches* m,
                        int gaussNewtonSteps, VSTransform* out, double* residual){
  return vsLensFitSimilarityFov(ld, m, gaussNewtonSteps, 0.0, out, residual);
}

int vsLensFitSimilarityFov(const VSLensDistortion* ld, const VSPointMatches* m,
                           int gaussNewtonSteps, double f,
                           VSTransform* out, double* residual){
  int n, j, it, ret = VS_OK, nAct = 0;
  double *ax, *ay, *bx, *by;
  double c = 1, s = 0, tx = 0, ty = 0;
  double sumRR = 0, res2 = 0;
  double mAx = 0, mAy = 0, mBx = 0, mBy = 0, num_c = 0, num_s = 0;
  const unsigned char* act;

  if(!ld || !m || !out || m->n < 2) return VS_ERROR;
  n = m->n;
  act = m->active;
  for(j=0; j<n; j++) if(!act || act[j]) nAct++;
  /* four parameters need at least two point pairs, and in practice a few more
     before the fit means anything */
  if(nAct < 3) return VS_ERROR;

  /* ax,ay: undistorted sources, centred -- fixed throughout, U does not depend
     on the similarity.  bx,by: OBSERVED destinations, centred: the residual is
     taken in image space, so these are deliberately not undistorted. */
  ax = (double*)vs_malloc(sizeof(double)*4*n);
  if(!ax) return VS_ERROR;
  ay = ax + n; bx = ax + 2*n; by = ax + 3*n;

  {
    /* the closed-form initialiser needs undistorted destinations too */
    double* ubx = (double*)vs_malloc(sizeof(double)*2*n);
    double* uby;
    if(!ubx){ vs_free(ax); return VS_ERROR; }
    uby = ubx + n;

    for(j=0; j<n; j++){
      double ux, uy, vx, vy;
      if(act && !act[j]) continue;
      if(vsLensUndistortPoint(ld, m->px[j], m->py[j], &ux, &uy) != VS_OK ||
         vsLensUndistortPoint(ld, m->qx[j], m->qy[j], &vx, &vy) != VS_OK){
        vs_free(ubx); vs_free(ax); return VS_ERROR;
      }
      ax[j]  = ux - ld->cx;  ay[j]  = uy - ld->cy;
      ubx[j] = vx - ld->cx;  uby[j] = vy - ld->cy;
      bx[j]  = m->qx[j] - ld->cx;
      by[j]  = m->qy[j] - ld->cy;
      mAx += ax[j]; mAy += ay[j]; mBx += ubx[j]; mBy += uby[j];
    }
    mAx /= nAct; mAy /= nAct; mBx /= nAct; mBy /= nAct;

    /* Least squares similarity on the undistorted correspondences, which are
       related by S exactly.  With b = [[c,s],[-s,c]]a + t as in
       transform_vec_double, the normal equations decouple into these two. */
    for(j=0; j<n; j++){
      double dax, day, dbx, dby;
      if(act && !act[j]) continue;
      dax = ax[j]-mAx; day = ay[j]-mAy;
      dbx = ubx[j]-mBx; dby = uby[j]-mBy;
      sumRR += dax*dax + day*day;
      num_c += dax*dbx + day*dby;
      num_s += day*dbx - dax*dby;
    }
    vs_free(ubx);
    if(sumRR < 1e-12){ vs_free(ax); return VS_ERROR; }
    c = num_c/sumRR;
    s = num_s/sumRR;
    tx = mBx - ( c*mAx + s*mAy);
    ty = mBy - (-s*mAx + c*mAy);
  }

  /* Gauss-Newton on the image-space residual e = q - D(S(U(p))).
     Parameterised by (tx,ty,c,s) rather than (x,y,alpha,zoom) so the inner
     linear algebra stays well conditioned; converted back at the end. */
  for(it=0; it<gaussNewtonSteps; it++){
    double H[16], g[4], delta[4];
    LensFwd W, Wp[4];
    int i, q;
    for(i=0; i<16; i++) H[i] = 0;
    for(i=0; i<4; i++)  g[i] = 0;

    /* Built once per iteration, not per point: the map depends only on the
       parameters.  Wp are the four perturbed copies the rotational path
       differences against; on the similarity path they are never touched. */
    lensFwdInit(&W, f, tx, ty, c, s);
    if(f > 0.0){
      lensFwdInit(&Wp[0], f, tx+LENS_H_T, ty, c, s);
      lensFwdInit(&Wp[1], f, tx, ty+LENS_H_T, c, s);
      lensFwdInit(&Wp[2], f, tx, ty, c+LENS_H_CS, s);
      lensFwdInit(&Wp[3], f, tx, ty, c, s+LENS_H_CS);
    }

    for(j=0; j<n; j++){
      double wx, wy, dpx, dpy, ex, ey;
      /* zeroed only to silence a maybe-uninitialized warning; the guard below
         returns before any use */
      double J[4] = {0,0,0,0};
      double Jp[4][2];
      if(act && !act[j]) continue;
      lensFwdApply(&W, ax[j], ay[j], &wx, &wy);
      if(vsLensDistortPoint(ld, wx + ld->cx, wy + ld->cy, &dpx, &dpy) != VS_OK ||
         lensDistortJacobian(ld, wx, wy, J) != VS_OK){
        vs_free(ax); return VS_ERROR;
      }
      ex = bx[j] - (dpx - ld->cx);
      ey = by[j] - (dpy - ld->cy);
      /* dw/d(tx,ty,c,s) pushed through the 2x2 distortion Jacobian */
      if(f > 0.0){
        static const double H[4] = {LENS_H_T, LENS_H_T, LENS_H_CS, LENS_H_CS};
        for(i=0; i<4; i++){
          double pwx, pwy, dwx, dwy;
          lensFwdApply(&Wp[i], ax[j], ay[j], &pwx, &pwy);
          dwx = (pwx - wx)/H[i];
          dwy = (pwy - wy)/H[i];
          Jp[i][0] = J[0]*dwx + J[1]*dwy;
          Jp[i][1] = J[2]*dwx + J[3]*dwy;
        }
      } else {
      Jp[0][0] = J[0];               Jp[0][1] = J[2];
      Jp[1][0] = J[1];               Jp[1][1] = J[3];
      Jp[2][0] = J[0]*ax[j] + J[1]*ay[j];
      Jp[2][1] = J[2]*ax[j] + J[3]*ay[j];
      Jp[3][0] = J[0]*ay[j] - J[1]*ax[j];
      Jp[3][1] = J[2]*ay[j] - J[3]*ax[j];
      }
      for(i=0; i<4; i++){
        g[i] += Jp[i][0]*ex + Jp[i][1]*ey;
        for(q=i; q<4; q++)
          H[i*4+q] += Jp[i][0]*Jp[q][0] + Jp[i][1]*Jp[q][1];
      }
    }
    for(i=0; i<4; i++) for(q=0; q<i; q++) H[i*4+q] = H[q*4+i];

    if(lensSolve4(H, g, delta) != VS_OK) break;  /* keep the current estimate */
    tx += delta[0]; ty += delta[1]; c += delta[2]; s += delta[3];
  }

  /* final residual at the converged parameters, over the active matches only */
  {
  LensFwd W;
  lensFwdInit(&W, f, tx, ty, c, s);
  for(j=0; j<n; j++){
    double wx, wy, dpx, dpy, ex, ey;
    if(act && !act[j]) continue;
    lensFwdApply(&W, ax[j], ay[j], &wx, &wy);
    if(vsLensDistortPoint(ld, wx + ld->cx, wy + ld->cy, &dpx, &dpy) != VS_OK){
      ret = VS_ERROR; break;
    }
    ex = bx[j] - (dpx - ld->cx);
    ey = by[j] - (dpy - ld->cy);
    res2 += ex*ex + ey*ey;
  }
  }

  if(ret == VS_OK){
    double z = sqrt(c*c + s*s);
    out->x        = tx;
    out->y        = ty;
    out->alpha    = atan2(s, c);
    out->zoom     = (z - 1.0)*100.0;
    out->barrel   = ld->k;
    out->rshutter = 0;
    out->extra    = 0;
    if(residual) *residual = sqrt(res2/nAct);
  }
  vs_free(ax);
  return ret;
}

VSLensEstimateConfig vsLensEstimateGetDefaultConfig(void){
  VSLensEstimateConfig c;
  /* Asymmetric: strong barrel is the larger physical effect, but the bracket
     must stay open above zero, or the undistorted case sits on the boundary
     where nothing can be bracketed and the curvature is meaningless.  kMax
     stays below the model's own limit, k = 1/(4*r^2) = 0.25 at the corner,
     past which D_k has no preimage. */
  c.kMin = -0.6;
  c.kMax =  0.2;
  c.tolerance = 1e-6;
  c.maxIterations = 100;
  c.gaussNewtonSteps = 3;
  c.maxUncertainty = 0.02;
  c.rejectOutliers = 1;
  c.outlierStddevs = 2.5;
  c.outlierPasses  = 3;
  /* 0 selects the similarity model; set from VSTransformConfig.fov by
     vsLocalmotions2Transforms. */
  c.f = 0.0;
  return c;
}

/* Value assigned where the model has no preimage for some point, so the search
   walks away from that region instead of failing outright.  Large against any
   real residual in px^2, small enough to stay well clear of overflow. */
#define LENS_PENALTY 1e12

/* Curvature below this counts as numerically zero rather than small: over the
   whole bracket it moves the objective by far less than one squared pixel, so
   there is no evidence about k at all.  This is what separates a genuinely flat
   objective -- a rotation-only path on exact data -- from a merely shallow one,
   where the statistical uncertainty below is the right measure. */
#define LENS_CURVATURE_FLOOR 1e-9
/* Reported in place of a standard error when there is no curvature to divide
   by; large enough that no sane maxUncertainty would accept it. */
#define LENS_UNDETERMINED_SIGMA 1e6

/* Masks out matches whose residual exceeds median + stddevs*1.4826*MAD, and
   returns how many were newly masked.

   Median absolute deviation rather than the mean and standard deviation used by
   disableFields() in localmotion2transform.c: a moving object covering a fifth
   of the fields inflates the standard deviation enough to hide itself inside
   its own threshold, whereas MAD tolerates up to half the data being bad.  The
   1.4826 makes it agree with the standard deviation on gaussian data. */
static int lensMaskOutliers(const double* res, unsigned char* active,
                            int n, double stddevs){
  double* tmp;
  double median, mad, thresh;
  int i, m = 0, cut = 0;

  for(i=0; i<n; i++) if(active[i]) m++;
  if(m < 8) return 0;   /* too few left for a meaningful robust spread */
  tmp = (double*)vs_malloc(sizeof(double)*m);
  if(!tmp) return 0;

  m = 0;
  for(i=0; i<n; i++) if(active[i]) tmp[m++] = res[i];
  qsort(tmp, m, sizeof(double), cmp_double);
  median = tmp[m/2];
  for(i=0; i<m; i++) tmp[i] = fabs(tmp[i] - median);
  qsort(tmp, m, sizeof(double), cmp_double);
  mad = tmp[m/2];
  vs_free(tmp);

  /* An all but exact fit gives mad == 0; there is nothing to reject then, and
     without this the threshold would collapse onto the median and cut half. */
  if(mad <= 1e-9) return 0;
  thresh = median + stddevs*1.4826*mad;

  for(i=0; i<n; i++){
    if(active[i] && res[i] > thresh){ active[i] = 0; cut++; }
  }
  return cut;
}

struct LensObjective {
  const VSFrameInfo*    fi;
  const VSPointMatches* frames;
  int numFrames;
  int gnSteps;
  int evals;
  double f;             /* focal length in px; 0 keeps the similarity model */
};

/* E(k): the similarities are profiled out, i.e. every frame is refitted from
   scratch at this k and only the residual it cannot explain is returned.
   Mean squared image-space residual per correspondence, in px^2. */
static double lensObjective(double k, struct LensObjective* o){
  VSLensDistortion ld = vsLensDistortionInit(o->fi, k);
  double sum2 = 0;
  int total = 0, i;
  o->evals++;
  for(i=0; i<o->numFrames; i++){
    VSTransform t;
    double r;
    if(o->frames[i].n < 3) continue;   /* cannot pin a similarity down */
    if(vsLensFitSimilarityFov(&ld, &o->frames[i], o->gnSteps, o->f, &t, &r) != VS_OK)
      return LENS_PENALTY;
    sum2  += r*r*o->frames[i].n;
    total += o->frames[i].n;
  }
  if(total < 1) return LENS_PENALTY;
  sum2 /= total;
  /* Non-finite input (a NaN coordinate) must not propagate into k and come back
     looking like an answer; treat it the same as leaving the model's domain. */
  if(!(sum2 == sum2) || sum2 > LENS_PENALTY) return LENS_PENALTY;
  return sum2;
}

/* Brent's method: golden section with parabolic interpolation where the
   parabola behaves.  Derivative free, which suits an objective whose every
   evaluation is itself a nonlinear fit. */
static double lensBrentMinimise(double a, double b, double tol, int maxIter,
                                struct LensObjective* o, int* usedIter){
  const double GOLD = 0.3819660112501051;  /* (3-sqrt(5))/2 */
  double x, w, v, fx, fw, fv, m, e = 0, d = 0, u, fu, p, q, r, tol1, tol2;
  int it;

  x = w = v = a + GOLD*(b - a);
  fx = fw = fv = lensObjective(x, o);

  for(it=0; it<maxIter; it++){
    m = 0.5*(a + b);
    tol1 = tol*fabs(x) + 1e-10;
    tol2 = 2.0*tol1;
    if(fabs(x - m) <= tol2 - 0.5*(b - a)) break;

    p = q = r = 0;
    if(fabs(e) > tol1){
      r = (x - w)*(fx - fv);
      q = (x - v)*(fx - fw);
      p = (x - v)*q - (x - w)*r;
      q = 2.0*(q - r);
      if(q > 0) p = -p; else q = -q;
      r = e; e = d;
    }
    if(fabs(p) < fabs(0.5*q*r) && p > q*(a - x) && p < q*(b - x)){
      d = p/q;                       /* parabolic step is acceptable */
      u = x + d;
      if(u - a < tol2 || b - u < tol2) d = (x < m) ? tol1 : -tol1;
    }else{
      e = (x < m) ? b - x : a - x;   /* fall back to golden section */
      d = GOLD*e;
    }
    u  = (fabs(d) >= tol1) ? x + d : x + ((d > 0) ? tol1 : -tol1);
    fu = lensObjective(u, o);

    if(fu <= fx){
      if(u < x) b = x; else a = x;
      v = w; fv = fw; w = x; fw = fx; x = u; fx = fu;
    }else{
      if(u < x) a = u; else b = u;
      if(fu <= fw || w == x){ v = w; fv = fw; w = u; fw = fu; }
      else if(fu <= fv || v == x || v == w){ v = u; fv = fu; }
    }
  }
  if(usedIter) *usedIter = it;
  return x;
}

VSLensEstimate vsEstimateLensDistortionFromMatches(const VSFrameInfo* fi,
                                                   const VSPointMatches* frames,
                                                   int numFrames,
                                                   const VSLensEstimateConfig* cfg){
  VSLensEstimate est;
  VSLensEstimateConfig defcfg = vsLensEstimateGetDefaultConfig();
  struct LensObjective o;
  VSPointMatches* masked = 0;
  unsigned char* flags = 0;
  double* res = 0;
  double h, e0, ep, em;
  int iters = 0, nPoints = 0, i;

  est.k = 0; est.residual = 0; est.curvature = 0; est.uncertainty = 0;
  est.iterations = 0; est.determined = 0; est.rejected = 0; est.used = 0;
  if(!fi || !frames || numFrames < 1) return est;
  if(!cfg) cfg = &defcfg;

  for(i=0; i<numFrames; i++) nPoints += frames[i].n;
  if(nPoints < 3) return est;
  est.used = nPoints;

  o.fi = fi; o.frames = frames; o.numFrames = numFrames;
  o.gnSteps = cfg->gaussNewtonSteps; o.evals = 0; o.f = cfg->f;

  est.k = lensBrentMinimise(cfg->kMin, cfg->kMax, cfg->tolerance,
                            cfg->maxIterations, &o, &iters);

  /* Reject-and-refit.  The mask is deliberately held FIXED for the whole of
     each Brent search and only updated between passes.  Rejecting inside the
     objective would let a wrong k discard more points and so lower E(k) for
     free, flattening and biasing the profile, and would make E discontinuous
     where the rejected set changes -- which is not a function Brent can
     minimise.  Rejection also happens at the current k, never at k=0, so that
     the systematic radial residual of unmodelled distortion is not mistaken
     for a frame full of outliers. */
  if(cfg->rejectOutliers && cfg->outlierPasses > 1){
    masked = (VSPointMatches*)vs_malloc(sizeof(VSPointMatches)*numFrames);
    flags  = (unsigned char*)vs_malloc(sizeof(unsigned char)*nPoints);
    res    = (double*)vs_malloc(sizeof(double)*nPoints);
  }
  if(masked && flags && res){
    int pass, off = 0;
    for(i=0; i<nPoints; i++) flags[i] = 1;
    for(i=0; i<numFrames; i++){
      masked[i] = frames[i];
      masked[i].active = flags + off;
      off += frames[i].n;
    }
    for(pass=1; pass<cfg->outlierPasses; pass++){
      VSLensDistortion ld = vsLensDistortionInit(fi, est.k);
      int cut = 0;
      off = 0;
      for(i=0; i<numFrames; i++){
        VSTransform t;
        int n = frames[i].n;
        if(vsLensFitSimilarity(&ld, &masked[i], cfg->gaussNewtonSteps, &t, 0) == VS_OK &&
           vsLensMatchResiduals(&ld, &masked[i], &t, res + off) == VS_OK){
          /* Threshold per frame, not globally: residual scale follows the
             motion magnitude, which differs from frame to frame. */
          cut += lensMaskOutliers(res + off, flags + off, n, cfg->outlierStddevs);
        }
        off += n;
      }
      if(cut == 0) break;   /* converged, nothing new to remove */
      o.frames = masked;
      est.k = lensBrentMinimise(cfg->kMin, cfg->kMax, cfg->tolerance,
                                cfg->maxIterations, &o, &iters);
    }
    o.frames = masked;   /* the reported residual must describe the final fit */
    for(i=0; i<nPoints; i++) if(!flags[i]) est.rejected++;
    est.used = nPoints - est.rejected;
  }

  /* Curvature of the profile curve at the minimum by central difference.  This
     is the whole point of profiling rather than alternating: a flat curve means
     the data cannot pin k down, which is a different situation from a
     confidently estimated k that happens to be near zero. */
  h  = 1e-3;
  e0 = lensObjective(est.k, &o);
  ep = lensObjective(est.k + h, &o);
  em = lensObjective(est.k - h, &o);
  if(ep >= LENS_PENALTY || em >= LENS_PENALTY){
    est.curvature = 0;   /* ran into the model's domain, treat as no evidence */
  }else{
    est.curvature = (ep - 2.0*e0 + em)/(h*h);
  }

  est.residual   = sqrt(e0 >= LENS_PENALTY ? 0 : e0);
  est.iterations = o.evals;

  /* Standard error of k.  With E the mean squared residual over nPoints
     correspondences, the sum of squares curves as nPoints*curvature, and the
     usual least squares result var(k) = 2*sigma^2 / d2SSE/dk2 reduces to
     E/(nPoints*curvature).  Unlike raw curvature this is scale free, so one
     threshold works regardless of field count, motion size or noise level. */
  if(est.curvature > LENS_CURVATURE_FLOOR && est.used > 0)
    est.uncertainty = est.residual/sqrt((double)est.used*est.curvature);
  else
    est.uncertainty = LENS_UNDETERMINED_SIGMA;

  /* A minimum sitting on the bracket edge was never bracketed at all: Brent
     converges to an endpoint when the objective still decreases through it, so
     the value is a boundary artefact rather than an estimate. */
  {
    double edge = 1e-6*(cfg->kMax - cfg->kMin) + cfg->tolerance;
    int pinned = (est.k - cfg->kMin < edge) || (cfg->kMax - est.k < edge);
    est.determined = !pinned && (est.uncertainty < cfg->maxUncertainty);
  }
  vs_free(masked); vs_free(flags); vs_free(res);
  return est;
}

VSLensEstimate vsEstimateLensDistortion(const VSFrameInfo* fi,
                                        const VSManyLocalMotions* motions,
                                        const VSLensEstimateConfig* cfg){
  VSLensEstimate est;
  VSPointMatches* frames;
  double* storage;
  int numFrames, i, j, totalFields = 0, offset = 0;

  est.k = 0; est.residual = 0; est.curvature = 0;
  est.iterations = 0; est.determined = 0;
  if(!fi || !motions) return est;
  numFrames = vs_vector_size((const VSVector*)motions);
  if(numFrames < 1) return est;

  for(i=0; i<numFrames; i++)
    totalFields += vs_vector_size(VSMLMGet(motions, i));
  if(totalFields < 1) return est;

  frames  = (VSPointMatches*)vs_malloc(sizeof(VSPointMatches)*numFrames);
  storage = (double*)vs_malloc(sizeof(double)*4*totalFields);
  if(!frames || !storage){ vs_free(frames); vs_free(storage); return est; }

  for(i=0; i<numFrames; i++){
    const LocalMotions* lms = VSMLMGet(motions, i);
    int n = vs_vector_size(lms);
    double* px = storage + 4*offset;
    double* py = px + n, *qx = px + 2*n, *qy = px + 3*n;
    for(j=0; j<n; j++){
      const LocalMotion* lm = LMGet(lms, j);
      px[j] = lm->f.x;
      py[j] = lm->f.y;
      qx[j] = (double)lm->f.x + lm->v.x;
      qy[j] = (double)lm->f.y + lm->v.y;
    }
    frames[i].px = px; frames[i].py = py;
    frames[i].qx = qx; frames[i].qy = qy;
    /* No caller-supplied mask; the estimator masks internally.  NULL is what
       the readers expect on the first, unmasked pass -- vsLensFitSimilarity
       tests (!act || act[j]) -- and vs_malloc does not zero, so leaving it
       alone read whatever was on the heap and dereferenced it.  That is a
       crash whenever the allocator hands back a dirtied page rather than a
       fresh one, which is why it survived the tests and would not have
       survived a long-running filter graph. */
    frames[i].active = 0;
    frames[i].n  = n;
    offset += n;
  }

  est = vsEstimateLensDistortionFromMatches(fi, frames, numFrames, cfg);
  vs_free(frames);
  vs_free(storage);
  return est;
}

int vsLensMatchResiduals(const VSLensDistortion* ld, const VSPointMatches* m,
                         const VSTransform* t, double* residuals){
  return vsLensMatchResidualsFov(ld, m, t, 0.0, residuals);
}

int vsLensMatchResidualsFov(const VSLensDistortion* ld, const VSPointMatches* m,
                            const VSTransform* t, double f, double* residuals){
  int j;
  double z;
  LensFwd w;
  if(!ld || !m || !t || !residuals) return VS_ERROR;
  z = 1.0 + t->zoom/100.0;
  /* Must use the SAME model the fit used, or stage two rejects the frame edge
     systematically at wide field of view -- the failure this function's own
     comment in vsLensMotionsToTransform warns about for k, one model up. */
  lensFwdInit(&w, f, t->x, t->y, z*cos(t->alpha), z*sin(t->alpha));
  for(j=0; j<m->n; j++){
    double ux, uy, rx, ry, wx, wy, dpx, dpy;
    if(vsLensUndistortPoint(ld, m->px[j], m->py[j], &ux, &uy) != VS_OK) return VS_ERROR;
    rx = ux - ld->cx;
    ry = uy - ld->cy;
    /* same convention as transform_vec_double */
    lensFwdApply(&w, rx, ry, &wx, &wy);
    wx += ld->cx; wy += ld->cy;
    if(vsLensDistortPoint(ld, wx, wy, &dpx, &dpy) != VS_OK) return VS_ERROR;
    residuals[j] = sqrt(sqr(m->qx[j]-dpx) + sqr(m->qy[j]-dpy));
  }
  return VS_OK;
}

VSTransform vsLensMotionsToTransform(const VSFrameInfo* fi, const VSLensDistortion* ld,
                                     const LocalMotions* motions,
                                     const VSLensEstimateConfig* cfg,
                                     double* residual){
  VSLensEstimateConfig defcfg = vsLensEstimateGetDefaultConfig();
  VSTransform t = null_transform();
  VSPointMatches m;
  double *buf, *res;
  unsigned char* act;
  int n, j;

  if(!fi || !ld || !motions) { t.extra = 1; return t; }
  n = vs_vector_size(motions);
  if(n < 3){ t.extra = 1; return t; }
  if(!cfg) cfg = &defcfg;

  buf = (double*)vs_malloc(sizeof(double)*5*n);
  act = (unsigned char*)vs_malloc(sizeof(unsigned char)*n);
  if(!buf || !act){ vs_free(buf); vs_free(act); t.extra = 1; return t; }
  res = buf + 4*n;

  for(j=0; j<n; j++){
    const LocalMotion* lm = LMGet(motions, j);
    buf[j]       = lm->f.x;
    buf[n+j]     = lm->f.y;
    buf[2*n+j]   = (double)lm->f.x + lm->v.x;
    buf[3*n+j]   = (double)lm->f.y + lm->v.y;
    act[j]       = 1;
    res[j]       = lm->match;    /* reused below as the stage one criterion */
  }
  m.px = buf; m.py = buf+n; m.qx = buf+2*n; m.qy = buf+3*n;
  m.active = act; m.n = n;

  /* Stage one, as in localmotion2transform.c: drop fields the matcher itself
     was unhappy with.  This criterion is model independent -- it says nothing
     about any assumed transform -- so it is safe to apply before k is known. */
  if(cfg->rejectOutliers) lensMaskOutliers(res, act, n, cfg->outlierStddevs);

  if(vsLensFitSimilarityFov(ld, &m, cfg->gaussNewtonSteps, cfg->f, &t, residual) != VS_OK){
    vs_free(buf); vs_free(act);
    t = null_transform(); t.extra = 1;
    return t;
  }

  /* Stage two: residual rejection, evaluated at the known k.  This is the step
     that must not be done blind -- at k=0 on distorted footage it would reject
     the frame edge, where the residual is systematic rather than anomalous. */
  if(cfg->rejectOutliers &&
     vsLensMatchResidualsFov(ld, &m, &t, cfg->f, res) == VS_OK &&
     lensMaskOutliers(res, act, n, cfg->outlierStddevs) > 0){
    VSTransform refined;
    double r2;
    if(vsLensFitSimilarityFov(ld, &m, cfg->gaussNewtonSteps, cfg->f, &refined, &r2) == VS_OK){
      t = refined;
      if(residual) *residual = r2;
    }
  }

  t.barrel = ld->k;
  vs_free(buf); vs_free(act);
  return t;
}
