/*
 *  lensmap.h
 *
 *  Render-path radial lens map: the per-pixel scale functions and their
 *  lookup tables, shared by every warp loop that applies lens correction.
 *
 *  See docs/lens-distortion.md for the derivation, the sign and normalisation
 *  conventions, and the identifiability analysis that motivates the estimator.
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
#ifndef __LENSMAP_H
#define __LENSMAP_H

#include <math.h>
#include <stdint.h>
#include "frameinfo.h"
#include "transformtype.h"
#include "vidstab_api.h"

/** How the estimated lens distortion is applied to the rendered picture.

    With M the stabilising similarity, U_k undistort and D_k distort:

      Off     x_src = M(x_out)                 no lens correction
      Wobble  x_src = D_k(M(U_k(x_out)))       same lens, camera held still
      Full    x_src = D_k(M(x_out))            ideal lens, straight lines straight

    Wobble collapses to the identity when M does, so it costs no field of view
    and leaves locked-off shots untouched.  See
    docs/superpowers/specs/2026-08-09-lens-correction-render-design.md */
typedef enum {
  VSLensCorrectOff = 0,
  VSLensCorrectWobble,
  VSLensCorrectFull
} VSLensCorrectMode;

/* Both U_k and D_k are radial: they leave a point's direction from the lens
   centre alone and only scale its distance, by a factor g that depends on the
   radius and nothing else.  Evaluating g costs a division (U_k) or a square
   root (D_k), which is too much per pixel, so each map's g is precomputed into
   a lookup table indexed by t = r^2 -- the squared radius, which the inner
   loop already has and which saves it a square root of its own.  gU and gD
   below are those tables; the warp loops read them through vsLensLutF/Fp,
   linearly interpolating between neighbouring samples.

   1024 samples make that interpolation accurate to under 1e-6 in g across the
   whole of gU's table, and across gD's up to tIn = 1/(1+k)^2, the largest
   radius still reachable from inside the frame.  Past tIn, D_k's derivative
   diverges towards its domain edge tDomD and gD is only guaranteed finite,
   positive and monotone -- but those samples land outside the source frame by
   construction, so the interpolator returns the border value whatever gD
   says. */
#define VS_LENS_LUT_N       1024
/* Sentinel destination for a sample outside the model's domain.  Far enough
   outside any frame that every interpolator returns the border value, small
   enough that iToFp16() of it does not overflow. */
#define VS_LENS_OUTSIDE_PX  (-30000)

/** Everything one plane's inner loop needs to evaluate the radial maps. */
typedef struct _vslensplanemap {
  int      active;      /* 0 -> the caller takes the plain affine path         */
  VSLensCorrectMode mode;
  double   k;
  double   cdx, cdy;    /* destination plane centre, plane units               */
  double   csx, csy;    /* source plane centre, plane units                    */
  double   tMaxU, tMaxD;/* LUT domains in t = r^2                              */
  /* t beyond which D_k is undefined, or -1.0 for "no bound" when k <= 0 (a
     genuine t is a squared radius, so never negative).  A finite sentinel
     rather than INFINITY because the build uses -ffast-math, under which
     comparisons against INFINITY are unreliable; readers test tDomD < 0.0. */
  double   tDomD;
  double   invRho2;     /* 1/rho^2 in luma-equivalent pixels^2                 */
  int      sxShift;     /* wsub: plane x units -> luma units is << sxShift     */
  int      syShift;     /* hsub                                                */
  int32_t  idxScaleU;   /* fp32: (N-1)/(tMaxU * rho^2), see vsLensLutFp        */
  int32_t  idxScaleD;
  int32_t* gU;          /* VS_LENS_LUT_N entries, 16.16                        */
  int32_t* gD;
#ifdef TESTING
  float*   gUf;
  float*   gDf;
#endif
} VSLensPlaneMap;

/** g of the division model, evaluated directly in double precision.
    Return values are only meaningful inside the model's domain; the callers
    check the domain themselves. */
static inline double vsLensScaleUDirectI(double k, double t){ return 1.0/(1.0 + k*t); }
static inline double vsLensScaleDDirectI(double k, double t){ return 2.0/(1.0 + sqrt(1.0 - 4.0*k*t)); }
VS_API double vsLensScaleUDirect(double k, double t);
VS_API double vsLensScaleDDirect(double k, double t);

/** Builds the map for one plane.  Returns VS_ERROR on allocation failure, and
    leaves m->active == 0 when mode is Off or k is zero. */
VS_API int vsLensPlaneMapInit(VSLensPlaneMap* m, const VSFrameInfo* fiSrc,
                              const VSFrameInfo* fiDest, int plane,
                              double k, VSLensCorrectMode mode);
VS_API void vsLensPlaneMapFree(VSLensPlaneMap* m);

/** Double-precision reference for the whole backward map of this plane, used by
    the zoom budget and the tests.  The inner loops inline the same arithmetic
    rather than calling this.  Returns VS_ERROR when the point leaves the
    model's domain, in which case *xs and *ys are set to VS_LENS_OUTSIDE_PX. */
VS_API int vsLensMapBackward(const VSLensPlaneMap* m, const VSTransform* t,
                             double xd, double yd, double* xs, double* ys);

#ifdef TESTING
/** float table lookup; t is r^2 in luma-equivalent units. */
static inline float vsLensLutF(const float* tab, double tMax, double t){
  double u = t * ((VS_LENS_LUT_N-1) / tMax);
  int    i;
  float  f;
  if(u >= VS_LENS_LUT_N-1) return tab[VS_LENS_LUT_N-1];
  i = (int)u;
  f = (float)(u - i);
  return tab[i] + (tab[i+1] - tab[i])*f;
}
#endif

/** 16.16 table lookup.  r2fp32 is the squared radius in luma-equivalent
    pixels^2 at scale 2^32 (i.e. the sum of squares of two 16.16 values), and
    idxScale is (N-1)/(tMax*rho^2) at scale 2^32. */
static inline int32_t vsLensLutFp(const int32_t* tab, int64_t r2fp32, int32_t idxScale){
  int64_t u = ((r2fp32 >> 16) * (int64_t)idxScale) >> 32;   /* index at 16.16 */
  int32_t i = (int32_t)(u >> 16);
  int32_t f;
  if(i >= VS_LENS_LUT_N-1) return tab[VS_LENS_LUT_N-1];
  f = (int32_t)(u & 0xFFFF);
  return tab[i] + (int32_t)((((int64_t)(tab[i+1] - tab[i])) * f) >> 16);
}

#endif

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
