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
