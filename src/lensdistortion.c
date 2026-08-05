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
#include "transformtype_operations.h"

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
      double dpx, dpy, ex, ey;
      /* zeroed so the optimiser can see it is never read unset: the guard below
         returns before any use, but that is not visible to it across the call */
      double J[4] = {0,0,0,0};
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

VSLensEstimateConfig vsLensEstimateGetDefaultConfig(void){
  VSLensEstimateConfig c;
  /* Asymmetric on purpose: strong barrel is the larger physical effect, but the
     bracket must stay open above zero.  Clamping at 0 would put the undistorted
     case exactly on the boundary, where a minimum cannot be bracketed and the
     curvature is meaningless, and would hide an estimator that wants to go
     positive -- which is a real signal, both of already-corrected footage and
     of a mis-specified model.

     Pincushion additionally has a hard model limit: D_k requires
     1 - 4*k*r^2 >= 0, so beyond k = 1/(4*r^2) -- a quarter at the image corner
     -- points simply have no preimage under the model.  kMax stays below that. */
  c.kMin = -0.6;
  c.kMax =  0.2;
  c.tolerance = 1e-6;
  c.maxIterations = 100;
  c.gaussNewtonSteps = 3;
  c.maxUncertainty = 0.02;
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

struct LensObjective {
  const VSFrameInfo*    fi;
  const VSPointMatches* frames;
  int numFrames;
  int gnSteps;
  int evals;
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
    if(vsLensFitSimilarity(&ld, &o->frames[i], o->gnSteps, &t, &r) != VS_OK)
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
  double h, e0, ep, em;
  int iters = 0, nPoints = 0;

  est.k = 0; est.residual = 0; est.curvature = 0; est.uncertainty = 0;
  est.iterations = 0; est.determined = 0;
  if(!fi || !frames || numFrames < 1) return est;
  if(!cfg) cfg = &defcfg;

  o.fi = fi; o.frames = frames; o.numFrames = numFrames;
  o.gnSteps = cfg->gaussNewtonSteps; o.evals = 0;
  for(iters=0; iters<numFrames; iters++) nPoints += frames[iters].n;
  iters = 0;

  est.k = lensBrentMinimise(cfg->kMin, cfg->kMax, cfg->tolerance,
                            cfg->maxIterations, &o, &iters);

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
  if(est.curvature > LENS_CURVATURE_FLOOR && nPoints > 0)
    est.uncertainty = est.residual/sqrt((double)nPoints*est.curvature);
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
    frames[i].n  = n;
    offset += n;
  }

  est = vsEstimateLensDistortionFromMatches(fi, frames, numFrames, cfg);
  vs_free(frames);
  vs_free(storage);
  return est;
}
