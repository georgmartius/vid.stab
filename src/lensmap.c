/*
 *  lensmap.c
 *
 *  See lensmap.h and docs/lens-distortion.md.
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
#include "lensmap.h"
#include "transform.h"                /* CHROMA_SIZE, VS_OK, VS_ERROR */
#include "transformtype_operations.h" /* rotation_matrix_backward */
#include "vidstabdefines.h"
#include <stdlib.h>
#include <string.h>

double vsLensScaleUDirect(double k, double t){ return vsLensScaleUDirectI(k, t); }
double vsLensScaleDDirect(double k, double t){ return vsLensScaleDDirectI(k, t); }

static int32_t toFp16d(double v){
  if(v >  32000.0) v =  32000.0;
  if(v < -32000.0) v = -32000.0;
  return (int32_t)(v * 65536.0 + (v >= 0 ? 0.5 : -0.5));
}

int vsLensPlaneMapInit(VSLensPlaneMap* m, const VSFrameInfo* fiSrc,
                       const VSFrameInfo* fiDest, int plane,
                       double k, VSLensCorrectMode mode){
  double rho, rhoDest, rho2;
  int i;
  memset(m, 0, sizeof(*m));
  m->tDomD = -1.0;    /* no bound; see lensmap.h -- never compare with INFINITY */

  /* The plane's geometry is filled even when there is no distortion to
     correct, before the early return below.  It describes the frame, not the
     lens, and vsLensMapBackward needs it to evaluate the affine or the
     rotational map on a plane whose radial terms are both off.  Leaving it
     zeroed would put the centre at the corner.  mode and k are deliberately
     NOT set here, so an inactive map stays Off with k = 0 and both radial
     branches stay unreachable. */
  m->sxShift = vsGetPlaneWidthSubS(fiSrc, plane);
  m->syShift = vsGetPlaneHeightSubS(fiSrc, plane);
  m->cdx  = (fiDest->width  >> m->sxShift)/2.0;
  m->cdy  = (fiDest->height >> m->syShift)/2.0;
  m->csx  = (fiSrc->width   >> m->sxShift)/2.0;
  m->csy  = (fiSrc->height  >> m->syShift)/2.0;

  if(mode == VSLensCorrectOff || k == 0.0) return VS_OK;

  m->k       = k;
  m->mode    = mode;

  /* rho is the SOURCE half-diagonal in luma pixels: the same lens, whatever
     the destination geometry.  See the spec, section 2.2. */
  rho  = 0.5*sqrt((double)fiSrc->width*fiSrc->width +
                  (double)fiSrc->height*fiSrc->height);
  rhoDest = 0.5*sqrt((double)fiDest->width*fiDest->width +
                     (double)fiDest->height*fiDest->height);
  rho2 = rho*rho;
  m->invRho2 = 1.0/rho2;

  /* U_k only ever sees destination-centred coordinates, so the destination
     half-diagonal plus 20% covers it.  D_k sees ideal coordinates after the
     similarity, so it needs room; r = 2 is twice the corner radius and every
     point beyond r = 1 is off-frame anyway. */
  m->tMaxU = 1.2 * (rhoDest/rho)*(rhoDest/rho);
  m->tMaxD = 4.0;
  if(k > 0){
    m->tDomD = 1.0/(4.0*k);
    if(m->tMaxD > 0.99*m->tDomD) m->tMaxD = 0.99*m->tDomD;
  }
  /* U_k's own domain edge, 1 + k*t <= 0, sits at t = -1/k for barrel; that is
     t >= 3.3 for k = -0.3, far beyond tMaxU. Assert rather than handle. */

  m->gU = (int32_t*)vs_malloc(sizeof(int32_t)*VS_LENS_LUT_N);
  m->gD = (int32_t*)vs_malloc(sizeof(int32_t)*VS_LENS_LUT_N);
  if(!m->gU || !m->gD){ vsLensPlaneMapFree(m); return VS_ERROR; }
#ifdef TESTING
  m->gUf = (float*)vs_malloc(sizeof(float)*VS_LENS_LUT_N);
  m->gDf = (float*)vs_malloc(sizeof(float)*VS_LENS_LUT_N);
  if(!m->gUf || !m->gDf){ vsLensPlaneMapFree(m); return VS_ERROR; }
#endif
  for(i=0; i<VS_LENS_LUT_N; i++){
    double tU = m->tMaxU * i/(double)(VS_LENS_LUT_N-1);
    double tD = m->tMaxD * i/(double)(VS_LENS_LUT_N-1);
    double gu = vsLensScaleUDirectI(k, tU);
    double gd = vsLensScaleDDirectI(k, tD);
    m->gU[i] = toFp16d(gu);
    m->gD[i] = toFp16d(gd);
#ifdef TESTING
    m->gUf[i] = (float)gu;
    m->gDf[i] = (float)gd;
#endif
  }
  /* idxScale = (N-1)/(tMax*rho^2) at scale 2^32, consumed by vsLensLutFp.  The
     int32_t it lands in overflows on tiny frames -- idxScaleU once the
     destination half-diagonal drops below ~41 px, idxScaleD once the source
     one drops below ~23 px -- so compute in double and bail before the
     narrowing cast.  This also catches an infinite k, whose tMaxD -> 0. */
  { double idxScaleUd = (VS_LENS_LUT_N-1)/(m->tMaxU*rho2) * 4294967296.0;
    double idxScaleDd = (VS_LENS_LUT_N-1)/(m->tMaxD*rho2) * 4294967296.0;
    if(idxScaleUd > (double)INT32_MAX || idxScaleDd > (double)INT32_MAX){
      vs_log_error("vid.stab", "lens map: geometry too small for k=%g, "
                   "disabling lens correction\n", k);
      vsLensPlaneMapFree(m);
      m->active = 0;
      return VS_OK;
    }
    m->idxScaleU = (int32_t)idxScaleUd;
    m->idxScaleD = (int32_t)idxScaleDd;
  }
  m->active = 1;
  return VS_OK;
}

void vsLensPlaneMapFree(VSLensPlaneMap* m){
  if(m->gU)  { vs_free(m->gU);  m->gU  = 0; }
  if(m->gD)  { vs_free(m->gD);  m->gD  = 0; }
#ifdef TESTING
  if(m->gUf) { vs_free(m->gUf); m->gUf = 0; }
  if(m->gDf) { vs_free(m->gDf); m->gDf = 0; }
#endif
  m->active = 0;
}

/* t = r^2 in luma-equivalent units for a plane-unit offset. */
static double tOf(const VSLensPlaneMap* m, double dx, double dy){
  double lx = dx * (1 << m->sxShift);
  double ly = dy * (1 << m->syShift);
  return (lx*lx + ly*ly) * m->invRho2;
}

int vsLensMapBackward(const VSLensPlaneMap* m, const VSTransform* t,
                      double xd, double yd, double* xs, double* ys){
  double dx = xd - m->cdx, dy = yd - m->cdy;
  double ax = (double)(1 << m->sxShift), ay = (double)(1 << m->syShift);
  double z, ca, sa, tx, ty, xi, yi, ex, ey, tt;
  if(m->mode == VSLensCorrectWobble){
    double g = vsLensScaleUDirectI(m->k, tOf(m, dx, dy));
    dx *= g; dy *= g;
  }
  z  = 1.0 - t->zoom/100.0;
  if(m->f > 0.0){
    /* Rotational model, in luma units so one f serves every plane -- the same
       arithmetic the warp loops run (transformfloat.c, transformfixedpoint.c);
       this function is what keeps them honest. */
    double rb[9], lx = dx*ax, ly = dy*ay, X, Y, Z;
    rotation_matrix_backward(t->x/m->f, t->y/m->f, t->alpha, rb);
    X = rb[0]*lx + rb[1]*ly + rb[2]*m->f;
    Y = rb[3]*lx + rb[4]*ly + rb[5]*m->f;
    Z = rb[6]*lx + rb[7]*ly + rb[8]*m->f;
    xi = z*m->f*X/Z / ax + m->csx;
    yi = z*m->f*Y/Z / ay + m->csy;
  } else {
  ca = z*cos(-t->alpha); sa = z*sin(-t->alpha);
  tx = t->x / ax;
  ty = t->y / ay;
  /* The rotation mixes the two axes, which are to different luma scales when
     wsub != hsub, so its cross terms convert between them -- the same
     conversion the warp loops apply (issue #79).  Both factors are 1 when
     wsub == hsub, which includes the luma and every packed plane. */
  xi =  ca*dx + sa*(ay/ax)*dy + m->csx - tx;
  yi = -sa*(ax/ay)*dx + ca*dy + m->csy - ty;
  }
  if(m->mode != VSLensCorrectOff){
    ex = xi - m->csx; ey = yi - m->csy;
    tt = tOf(m, ex, ey);
    if(m->tDomD >= 0.0 && tt > m->tDomD){ *xs = *ys = VS_LENS_OUTSIDE_PX; return VS_ERROR; }
    { double g = vsLensScaleDDirectI(m->k, tt);
      xi = m->csx + ex*g; yi = m->csy + ey*g; }
  }
  *xs = xi; *ys = yi;
  return VS_OK;
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
