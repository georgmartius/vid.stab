/*
 *  l1campathoptimization.h
 *
 *  Copyright (C) Georg Martius - January 2014 - 2026
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
#ifndef __L1CAMPATHOPTIMIZATION_H
#define __L1CAMPATHOPTIMIZATION_H

#include "transform.h"

/** Optimal camera path via L1 minimization, after
 *
 *  @INPROCEEDINGS{GrundmannKwatra2011,
 *   author    = {M. Grundmann and V. Kwatra and I. Essa},
 *   title     = {Auto-Directed Video Stabilization with Robust L1 Optimal
 *                Camera Paths},
 *   booktitle = {IEEE Conference on Computer Vision and Pattern Recognition
 *                (CVPR)},
 *   year      = {2011}}
 *
 *  The original camera path C_t is given by the frame-pair transforms F_t via
 *  C_t = C_{t-1} F_t.  We look for update ("crop window") transforms B_t such
 *  that the stabilized path P_t = C_t B_t is composed of static, linear and
 *  parabolic segments only.  That is achieved by minimizing the weighted L1
 *  norm of the first three derivatives of P, which -- after dropping the known
 *  factor C_t, see eq. (4) of the paper -- becomes a linear program in B.
 */

/* --------------------------------------------------------------------------
 * 4-DOF linear similarity transform.
 *
 * Corresponds to the matrix
 *
 *       [ a   b   x ]
 *       [-b   a   y ]
 *       [ 0   0   1 ]
 *
 * acting on coordinates relative to the frame centre.  This is the same
 * transform as VSTransform, only parametrized by (x,y,a,b) instead of
 * (x,y,alpha,zoom), which turns composition into a bilinear -- and the LP
 * constraints into a linear -- expression.  The relation to VSTransform is
 *
 *      a = (1 + zoom/100) cos(alpha)
 *      b = (1 + zoom/100) sin(alpha)
 *
 * matching prepare_transform() in transformtype.c.
 * -------------------------------------------------------------------------- */
typedef struct VS_API _VSTransformLS {
  double x;
  double y;
  double a;
  double b;
  int    extra;
} VSTransformLS;

/// identity transform (x=y=0, a=1, b=0)
VS_API VSTransformLS id_transformLS(void);
/// matrix product t1 * t2 (t2 applied first)
VS_API VSTransformLS concat_transformLS(const VSTransformLS* t1, const VSTransformLS* t2);
/// matrix inverse
VS_API VSTransformLS invert_transformLS(const VSTransformLS* t);
/// applies the transform to a point given relative to the frame centre
VS_API void transformLS_vec(double* rx, double* ry, const VSTransformLS* t,
                            double x, double y);
VS_API VSTransformLS transformAZtoLS(const VSTransform* t);
VS_API VSTransform   transformLStoAZ(const VSTransformLS* t);

/** zoom in percent the optimization spends when the VSTransformConfig asks for
    an automatic zoom (optZoom != 0) without naming an amount (zoom == 0) */
#define VS_L1_DEFAULT_ZOOM 15.0

/** parameters of the L1 optimization, independent of VSTransformData so that
    the core can be exercised on its own */
typedef struct VS_API _VSL1Config {
  double w1;          ///< weight of |D(P)|_1   (constant segments)
  double w2;          ///< weight of |D^2(P)|_1 (linear segments)
  double w3;          ///< weight of |D^3(P)|_1 (parabolic segments)
  /** relative weight of the (a,b) part of the residual against the (x,y) part.
      The paper uses 100, since translations live on a pixel scale while a and
      b are of order 1. */
  double wAffine;
  double frameWidth;  ///< width of the frame in pixels
  double frameHeight; ///< height of the frame in pixels
  /** size of the crop window relative to the frame, in (0,1].  The corners of
      that window, transformed by B_t, are constrained to stay inside the
      frame; the smaller it is, the more freedom the optimization has.  The
      paper uses 0.75. */
  double cropRatio;
  /** proximity constraints (paper section 2.1: 0.9 <= a <= 1.1, |b| <= 0.1).
      a below 1 lets the optimization shrink the sampled region to buy extra
      translation freedom, at the price of magnifying the output further: the
      total magnification is 1/(cropRatio * minScale).  We therefore default
      minScale to 1, so that cropRatio alone determines how much is zoomed;
      lower it to trade sharpness for smoothness. */
  double minScale;    ///< lower bound on a, default 1.0
  double maxScale;    ///< upper bound on a, default 1.1
  double maxSkewDev;  ///< proximity: |b| <= maxSkewDev (paper: 0.1)
  int    verbose;
} VSL1Config;

VS_API VSL1Config vsL1GetDefaultConfig(void);

/** Core of the algorithm, free of any VSTransformData dependency.

    @param F     frame-pair transforms, F[t] takes frame t to frame t-1's
                 coordinate system so that C_t = C_{t-1} F_t.  F[0] is never
                 read; pass the identity.
    @param N     number of frames, must be >= 4
    @param B     output array of N update transforms (caller allocated)
    @param conf  parameters, must not be NULL
    @param objective if not NULL, receives the optimal objective value
    @return VS_OK on success, VS_ERROR if the LP could not be solved
 */
VS_API int vsCameraPathOptimalL1LS(const VSTransformLS* F, int N,
                                   VSTransformLS* B, const VSL1Config* conf,
                                   double* objective);

/** Camera path optimization for a list of relative vid.stab transforms.
    trans->ts is replaced in place by the update transforms B_t, in the same
    sense as cameraPathGaussian(): applying ts[t] to frame t yields the
    stabilized frame. */
VS_API int cameraPathOptimalL1(VSTransformData* td, VSTransformations* trans);

/** builds the VSL1Config that cameraPathOptimalL1() would use for td */
VS_API VSL1Config vsL1ConfigFromTransformConfig(const VSTransformData* td);

#ifdef TESTING
/* index helpers, exposed for the unit tests */
int vs_l1_row(int order, int t, int param, int upperorlower, int N);
int vs_l1_rowCorner(int t, int k, int N);
int vs_l1_rowBase(int t, int N);
int vs_l1_col(int group, int t, int param, int N);
int vs_l1_numrows(int N);
int vs_l1_numcols(int N);
#endif

#endif  /* __L1CAMPATHOPTIMIZATION_H */

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
