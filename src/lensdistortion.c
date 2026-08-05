/*
 * lensdistortion.c
 *
 *  See lensdistortion.h and docs/lens-distortion.md.
 *
 *  Copyright (C) Georg Martius - 2026
 *
 *  This file is part of vid.stab video stabilization library
 *  and is licensed under the GNU GPL v2 or later, see lensdistortion.h.
 */

#include "lensdistortion.h"
#include "vidstabdefines.h"

#include <math.h>

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

int vsLensFitSimilarity(const VSLensDistortion* ld, const VSPointMatches* m,
                        int gaussNewtonSteps, VSTransform* out, double* residual){
  int n, j, it, ret = VS_OK;
  double *ax, *ay, *bx, *by;
  double c = 1, s = 0, tx = 0, ty = 0;
  double sumRR = 0, res2 = 0;
  double mAx = 0, mAy = 0, mBx = 0, mBy = 0, num_c = 0, num_s = 0;

  if(!ld || !m || !out || m->n < 2) return VS_ERROR;
  n = m->n;

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
    mAx /= n; mAy /= n; mBx /= n; mBy /= n;

    /* Least squares similarity on the undistorted correspondences, which are
       related by S exactly.  With b = [[c,s],[-s,c]]a + t as in
       transform_vec_double, the normal equations decouple into these two. */
    for(j=0; j<n; j++){
      double dax = ax[j]-mAx, day = ay[j]-mAy;
      double dbx = ubx[j]-mBx, dby = uby[j]-mBy;
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
    int i, q;
    for(i=0; i<16; i++) H[i] = 0;
    for(i=0; i<4; i++)  g[i] = 0;

    for(j=0; j<n; j++){
      double wx = c*ax[j] + s*ay[j] + tx;
      double wy = -s*ax[j] + c*ay[j] + ty;
      double dpx, dpy, J[4], ex, ey;
      double Jp[4][2];
      if(vsLensDistortPoint(ld, wx + ld->cx, wy + ld->cy, &dpx, &dpy) != VS_OK ||
         lensDistortJacobian(ld, wx, wy, J) != VS_OK){
        vs_free(ax); return VS_ERROR;
      }
      ex = bx[j] - (dpx - ld->cx);
      ey = by[j] - (dpy - ld->cy);
      /* dw/d(tx,ty,c,s) pushed through the 2x2 distortion Jacobian */
      Jp[0][0] = J[0];               Jp[0][1] = J[2];
      Jp[1][0] = J[1];               Jp[1][1] = J[3];
      Jp[2][0] = J[0]*ax[j] + J[1]*ay[j];
      Jp[2][1] = J[2]*ax[j] + J[3]*ay[j];
      Jp[3][0] = J[0]*ay[j] - J[1]*ax[j];
      Jp[3][1] = J[2]*ay[j] - J[3]*ax[j];
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

  /* final residual at the converged parameters */
  for(j=0; j<n; j++){
    double wx = c*ax[j] + s*ay[j] + tx;
    double wy = -s*ax[j] + c*ay[j] + ty;
    double dpx, dpy, ex, ey;
    if(vsLensDistortPoint(ld, wx + ld->cx, wy + ld->cy, &dpx, &dpy) != VS_OK){
      ret = VS_ERROR; break;
    }
    ex = bx[j] - (dpx - ld->cx);
    ey = by[j] - (dpy - ld->cy);
    res2 += ex*ex + ey*ey;
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
    if(residual) *residual = sqrt(res2/n);
  }
  vs_free(ax);
  return ret;
}
