/*
 * lensdistortion.h
 *
 *  Single-parameter radial lens distortion (division model) and recovery of
 *  its parameter from detected local motions, assuming a rigid scene.
 *
 *  See docs/lens-distortion.md for the derivation, the sign and normalisation
 *  conventions, and the identifiability analysis that motivates the estimator.
 *
 *  Copyright (C) Georg Martius - 2026
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
 *  the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 *  Boston, MA 02110-1301, USA.
 *
 */

#ifndef __LENSDISTORTION_H
#define __LENSDISTORTION_H

#include "frameinfo.h"
#include "serialize.h"
#include "transformtype.h"
#include "vidstab_api.h"

/** Radial distortion of a frame, division model, centred on the frame centre.

    Undistort (observed -> ideal) divides:   U(x) = x / (1 + k*r^2)
    Distort   (ideal -> observed) inverts:   D(x) = x * 2/(1 + sqrt(1 - 4*k*r^2))
    with r the radius measured in units of rho.  Barrel distortion is k < 0.
 */
typedef struct VS_API _vslensdistortion {
  double k;    // distortion strength, dimensionless; <0 barrel, >0 pincushion, 0 none
  double cx;   // distortion centre x in pixels, always the frame centre width/2
  double cy;   // distortion centre y in pixels, always the frame centre height/2
  double rho;  // radius normalisation in pixels, half the frame diagonal, so r=1 at a corner
} VSLensDistortion;

/** builds the distortion of the given frame geometry with strength k */
VS_API VSLensDistortion vsLensDistortionInit(const VSFrameInfo* fi, double k);

/** maps an observed (distorted) pixel to where an ideal lens would have put it.
    Returns VS_ERROR without writing the outputs if the point is outside the
    model's domain (1 + k*r^2 <= 0), which only happens for strong pincushion. */
VS_API int vsLensUndistortPoint(const VSLensDistortion* ld,
                                double xi, double yi, double* xo, double* yo);

/** maps an ideal pixel to where the real lens puts it; inverse of the above.
    Returns VS_ERROR without writing the outputs if the point is outside the
    model's domain (1 - 4*k*r^2 < 0), which only happens for strong pincushion. */
VS_API int vsLensDistortPoint(const VSLensDistortion* ld,
                              double xi, double yi, double* xo, double* yo);

/** Observed point correspondences for one frame pair, in pixels.

    These are still *distorted* image positions: p is where a measurement field
    sat, q is where it was found.  Parallel arrays of length n. */
typedef struct VS_API _vspointmatches {
  const double* px;  // x of the source points, length n
  const double* py;  // y of the source points, length n
  const double* qx;  // x of the matched destination points, length n
  const double* qy;  // y of the matched destination points, length n
  const unsigned char* active; // optional per-match mask, 0 to ignore; NULL means use all
  int n;             // number of correspondences in this frame pair
} VSPointMatches;

/** Fits the similarity that best explains the matches under a known distortion.

    Minimises the image-space residual sum |q - D(S(U(p)))|^2 over the four
    similarity parameters, starting from the closed-form least squares solution
    on the undistorted correspondences and refining with gaussNewtonSteps
    Gauss-Newton iterations.  Writes the RMS residual per correspondence in
    pixels to residual when that is non-NULL.  Returns VS_ERROR if there are
    too few matches or a point leaves the model's domain. */
VS_API int vsLensFitSimilarity(const VSLensDistortion* ld, const VSPointMatches* m,
                               int gaussNewtonSteps, VSTransform* out, double* residual);

/** Per-correspondence distance |q - D(S(U(p)))| in pixels, under the given
    distortion and similarity.  Writes m->n values, including for matches that
    are masked out.  Returns VS_ERROR if a point leaves the model's domain. */
VS_API int vsLensMatchResiduals(const VSLensDistortion* ld, const VSPointMatches* m,
                                const VSTransform* t, double* residuals);

/** Tuning for the distortion search; vsLensEstimateGetDefaultConfig fills it. */
typedef struct VS_API _vslensestimateconfig {
  double kMin;           // low end of the bracket searched for k, default -0.6
  double kMax;           // high end of the bracket; stays >0 so k=0 is interior, default 0.3
  double tolerance;      // absolute convergence tolerance on k, default 1e-6
  int    maxIterations;  // cap on golden-section/parabolic iterations, default 100
  int    gaussNewtonSteps; // inner similarity refinement steps per frame, default 3
  double maxUncertainty; // largest standard error in k still called determined, default 0.02
  int    rejectOutliers; // 1 to run the reject-and-refit passes below, 0 to fit everything
  double outlierStddevs; // reject beyond median + this many robust sigmas of residual, default 2.5
  int    outlierPasses;  // total search passes; 1 means no rejection ever happens, default 3
} VSLensEstimateConfig;

/** Outcome of the search. */
typedef struct VS_API _vslensestimate {
  double k;           // recovered distortion strength, same convention as VSLensDistortion.k
  double residual;    // RMS image-space residual per correspondence at the minimum, in pixels
  double curvature;   // d2E/dk2 at the minimum; near zero means the data cannot pin k down
  double uncertainty; // standard error of k, residual/sqrt(N*curvature); scale free
  int    iterations;  // how many objective evaluations the search used
  int    rejected;    // correspondences dropped as outliers, 0 when rejection is off
  int    used;        // correspondences the final estimate was actually fitted to
  int    determined;  // 0 when k is not identifiable: flat objective, or pinned to the bracket
} VSLensEstimate;

VS_API VSLensEstimateConfig vsLensEstimateGetDefaultConfig(void);

/** recovers one distortion parameter for the whole clip from point matches */
VS_API VSLensEstimate vsEstimateLensDistortionFromMatches(const VSFrameInfo* fi,
                                                          const VSPointMatches* frames,
                                                          int numFrames,
                                                          const VSLensEstimateConfig* cfg);

/** Per-frame camera motion from local motions, under an already known lens.

    The distortion-aware counterpart of vsMotionsToTransform: it fits the same
    four similarity parameters, but through the lens model rather than assuming
    none, so barrel distortion is not absorbed into the reported motion.  Runs
    the two stage outlier handling when cfg->rejectOutliers is set -- first on
    the matcher's own confidence, then on the residual at this k.  Sets
    extra = 1 when no usable fit was possible.  Writes the RMS residual to
    residual when that is non-NULL. */
VS_API VSTransform vsLensMotionsToTransform(const VSFrameInfo* fi,
                                            const VSLensDistortion* ld,
                                            const LocalMotions* motions,
                                            const VSLensEstimateConfig* cfg,
                                            double* residual);

/** as above, taking the local motions that vsMotionDetection produces.
    Displacements there are integers, so expect roughly a hundredth of
    distortion strength in quantisation-induced error. */
VS_API VSLensEstimate vsEstimateLensDistortion(const VSFrameInfo* fi,
                                               const VSManyLocalMotions* motions,
                                               const VSLensEstimateConfig* cfg);

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
