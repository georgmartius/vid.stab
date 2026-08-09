/*
 *  transform.c
 *
 *  Copyright (C) Georg Martius - June 2007 - 2011
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

#include "transform.h"
#include "transform_internal.h"
#include "transformtype_operations.h"
#include "l1campathoptimization.h"

#include "transformfixedpoint.h"
#ifdef TESTING
#include "transformfloat.h"
#endif

#include <math.h>
#ifndef _MSC_VER
    #include <libgen.h>
#endif
#include <string.h>

const char* interpol_type_names[5] = {"No (0)", "Linear (1)", "Bi-Linear (2)",
                                      "Bi-Cubic (3)"};

const char* getInterpolationTypeName(VSInterpolType type){
  if (type >= VS_Zero && type < VS_NBInterPolTypes)
    return interpol_type_names[(int) type];
  else
    return "unknown";
}

// default initialization: attention the ffmpeg filter cannot call it
VSTransformConfig vsTransformGetDefaultConfig(const char* modName){
  VSTransformConfig conf;
  /* Options */
  conf.maxShift           = -1;
  conf.maxAngle           = -1;
  conf.crop               = VSKeepBorder;
  conf.relative           = 1;
  conf.invert             = 0;
  conf.smoothing          = 15;
  conf.zoom               = 0;
  conf.optZoom            = 1;
  conf.zoomSpeed          = 0.25;
  conf.interpolType       = VS_BiLinear;
  conf.verbose            = 0;
  conf.modName            = modName;
  conf.simpleMotionCalculation = 0;
  conf.storeTransforms    = 0;
  conf.smoothZoom         = 0;
  conf.camPathAlgo        = VSOptimalL1;
  /* On by default: it costs one extra pass over the local motions and reverts
     to the previous behaviour whenever the estimate is not trustworthy. */
  conf.estimateLensDistortion = 1;
  /* Wobble is safe to default on: identity in, identity out. */
  conf.lensCorrection = VSLensCorrectWobble;
  conf.lensK          = 0.0;
  return conf;
}

void vsTransformGetConfig(VSTransformConfig* conf, const VSTransformData* td){
  if(td && conf)
    *conf = td->conf;
}

const VSFrameInfo* vsTransformGetSrcFrameInfo(const VSTransformData* td){
  return &td->fiSrc;
}

const VSFrameInfo* vsTransformGetDestFrameInfo(const VSTransformData* td){
  return &td->fiDest;
}


int vsTransformDataInit(VSTransformData* td, const VSTransformConfig* conf,
                        const VSFrameInfo* fi_src, const VSFrameInfo* fi_dest){
  td->conf = *conf;

  td->fiSrc = *fi_src;
  td->fiDest = *fi_dest;

  vsFrameNull(&td->src);
  td->srcMalloced = 0;

  vsFrameNull(&td->destbuf);
  vsFrameNull(&td->dest);

  if (td->conf.maxShift > td->fiDest.width/2)
    td->conf.maxShift = td->fiDest.width/2;
  if (td->conf.maxShift > td->fiDest.height/2)
    td->conf.maxShift = td->fiDest.height/2;

  td->conf.interpolType = VS_MAX(VS_MIN(td->conf.interpolType,VS_BiCubic),VS_Zero);

  switch(td->conf.interpolType){
   case VS_Zero:     td->interpolate = &interpolateZero; break;
   case VS_Linear:   td->interpolate = &interpolateLin; break;
   case VS_BiLinear: td->interpolate = &interpolateBiLin; break;
   case VS_BiCubic:  td->interpolate = &interpolateBiCub; break;
   default: td->interpolate = &interpolateBiLin;
  }
#ifdef TESTING
  switch(td->conf.interpolType){
   case VS_Zero:     td->_FLT(interpolate) = &_FLT(interpolateZero); break;
   case VS_Linear:   td->_FLT(interpolate) = &_FLT(interpolateLin); break;
   case VS_BiLinear: td->_FLT(interpolate) = &_FLT(interpolateBiLin); break;
   case VS_BiCubic:  td->_FLT(interpolate) = &_FLT(interpolateBiCub); break;
   default: td->_FLT(interpolate)          = &_FLT(interpolateBiLin);
  }

#endif
  td->lensMode   = td->conf.lensCorrection;
  td->lensActive = 0;
  td->lensK      = td->conf.lensK;      /* 0.0 unless the caller overrode it */
  /* Sentinel meaning "no map built yet", distinct from every value k can
     actually take: lensEnsureMaps() always leaves lensMapK at 0.0 or at a
     real, estimated/user k, and the division model's bracket keeps |k| well
     under 1 (see lensdistortion.h), so -1.0 can never be a genuine effective
     k and never collides with "not built yet". */
  td->lensMapK   = -1.0;
  memset(td->lensMaps, 0, sizeof(td->lensMaps));
  return VS_OK;
}

void vsTransformDataCleanup(VSTransformData* td){
  int p;
  for(p=0; p<3; p++) vsLensPlaneMapFree(&td->lensMaps[p]);
  if (td->srcMalloced && !vsFrameIsNull(&td->src)) {
    vsFrameFree(&td->src);
  }
  if (td->conf.crop == VSKeepBorder && !vsFrameIsNull(&td->destbuf)) {
    vsFrameFree(&td->destbuf);
  }
}

void vsTransformSetLensK(VSTransformData* td, double k){
  td->lensK = k;
}

/* Builds (or rebuilds) the per-plane maps.  k arrives after
   vsTransformDataInit, so this cannot live there.  Cheap when nothing
   changed: one double compare. */
void lensEnsureMaps(VSTransformData* td){
  double k = td->lensK;
  int planes, p;
  /* |k| below this moves a corner pixel by well under a pixel; acting on it
     would only add noise.  Same guard the estimator uses.  This also
     subsumes "no override" (lensK == 0.0), so there is no separate unset
     check: 0.0 and "too small to matter" collapse to the same effective
     k = 0.0, which is exactly the point -- see VSTransformConfig.lensK. */
  if(td->lensMode == VSLensCorrectOff || fabs(k) <= 0.01) k = 0.0;
  if(td->lensMapK == k) return;
  planes = td->fiSrc.pFormat < PF_PACKED ? td->fiSrc.planes : 1;
  for(p=0; p<3; p++) vsLensPlaneMapFree(&td->lensMaps[p]);
  td->lensActive = 0;
  for(p=0; p<planes; p++){
    /* k is exactly 0.0 here whenever correction isn't wanted (mode Off, or
       |k| too small to matter -- see above), and a real nonzero estimate
       otherwise, so k != 0.0 is exactly "wanted", with no separate flag. */
    if(vsLensPlaneMapInit(&td->lensMaps[p], &td->fiSrc, &td->fiDest, p, k,
                          k != 0.0 ? td->lensMode : VSLensCorrectOff) != VS_OK){
      int q;
      vs_log_error(td->conf.modName, "lens map allocation failed, correction off\n");
      for(q=0; q<3; q++) vsLensPlaneMapFree(&td->lensMaps[q]);
      td->lensActive = 0;
      /* Latch off at this k, not at 0.0: k is the effective (nonzero, since
         k==0.0 never reaches vsLensPlaneMapInit's allocation path) value that
         just failed.  Recording 0.0 here would leave the guard above
         permanently unable to match a nonzero k, so a persistent allocation
         failure would retry -- and re-log -- on every single frame. */
      td->lensMapK = k;
      return;
    }
  }
  td->lensActive = td->lensMaps[0].active;
  td->lensMapK   = k;
}

int vsTransformPrepare(VSTransformData* td, const VSFrame* src, VSFrame* dest){
  // we first copy the frame to td->src and then overwrite the destination
  // with the transformed version
  td->dest = *dest;
  if(src==dest || td->srcMalloced){ // in place operation: we have to copy the src first
    // We must own td->src before copying into it. Testing vsFrameIsNull() here
    // is not enough: after a previous frame took the else branch below, td->src
    // still aliases *that* caller's buffer, so the copy would write into memory
    // we do not own (e.g. a decoder reference frame still in use). See #144.
    if(!td->srcMalloced) {
      vsFrameAllocate(&td->src,&td->fiSrc);
      td->srcMalloced = 1;
    }
    if (vsFrameIsNull(&td->src)) {
      vs_log_error(td->conf.modName, "vs_malloc failed\n");
      return VS_ERROR;
    }
    vsFrameCopy(&td->src, src, &td->fiSrc);
  }else{ // otherwise no copy needed
    td->src=*src;
  }
  if (td->conf.crop == VSKeepBorder) {
    if(vsFrameIsNull(&td->destbuf)) {
      // if we keep the borders, we need a second buffer to store
      //  the previous stabilized frame, so we use destbuf
      vsFrameAllocate(&td->destbuf,&td->fiDest);
      if (vsFrameIsNull(&td->destbuf)) {
        vs_log_error(td->conf.modName, "vs_malloc failed\n");
        return VS_ERROR;
      }
      // if we keep borders, save first frame into the background buffer (destbuf)
      vsFrameCopy(&td->destbuf, src, &td->fiSrc); // here we have to take care
    }
  }else{ // otherwise we directly operate on the destination
    td->destbuf = *dest;
  }
  return VS_OK;
}

int vsDoTransform(VSTransformData* td, VSTransform t){
  if (td->fiSrc.pFormat < PF_PACKED)
    return transformPlanar(td, t);
  else
    return transformPacked(td, t);
}


int vsTransformFinish(VSTransformData* td){
  if(td->conf.crop == VSKeepBorder){
    // we have to store our result to video buffer
    // note: destbuf stores stabilized frame to be the default for next frame
    vsFrameCopy(&td->dest, &td->destbuf, &td->fiSrc);
  }
  return VS_OK;
}


VSTransform vsGetNextTransform(const VSTransformData* td, VSTransformations* trans){
  if(trans->len <=0 ) return null_transform();
  if (trans->current >= trans->len) {
    trans->current = trans->len;
    if(!trans->warned_end)
      vs_log_warn(td->conf.modName, "not enough transforms found, use last transformation!\n");
    trans->warned_end = 1;
  }else{
    trans->current++;
  }
  return trans->ts[trans->current-1];
}

void vsTransformationsInit(VSTransformations* trans){
  trans->ts = 0;
  trans->len = 0;
  trans->current = 0;
  trans->warned_end = 0;
}

void vsTransformationsCleanup(VSTransformations* trans){
  if (trans->ts) {
    vs_free(trans->ts);
    trans->ts = NULL;
  }
  trans->len=0;
}

/*
 *  This is actually the core algorithm for canceling the jiggle in the
 *  movie. We have different implementations which are patched here.
 */
int cameraPathOptimization(VSTransformData* td, VSTransformations* trans){
  switch(td->conf.camPathAlgo){
   case VSAvg: return cameraPathAvg(td,trans);
   case VSOptimalL1:
#ifdef VS_HAVE_LPSOLVER
    /* L1 optimizes a camera path built by composing relative transforms, so
       absolute ones put it out of scope by construction rather than by
       failure.  That is the tripod configuration (relative=0:smoothing=0),
       where the stored transforms are already the final per-frame corrections
       and the gaussian filter leaves them untouched -- exactly what tripod
       wants.  Go there directly, so a correct setup stays quiet. */
    if(td->conf.relative){
      /* cameraPathOptimalL1 leaves trans untouched unless it succeeds, so
         falling back to the gaussian filter is safe.  It fails for sequences
         shorter than 4 frames, without a zoom budget, and if the LP turns out
         to be infeasible. */
      if(cameraPathOptimalL1(td,trans)==VS_OK) return VS_OK;
      vs_log_msg(td->conf.modName,
                 "L1 camera path optimization unavailable, using gaussian filter");
    }
#endif // without an LP solver we always use the gaussian filter
    /* fall through */
   case VSGaussian: return cameraPathGaussian(td,trans);
  }
  return VS_ERROR;
}

/*
 *  We perform a low-pass filter on the camera path.
 *  This supports slow camera movemen, but in a smooth fasion.
 *  Here we use gaussian filter (gaussian kernel) lowpass filter
 */
int cameraPathGaussian(VSTransformData* td, VSTransformations* trans){
  VSTransform* ts = trans->ts;
  if (trans->len < 1)
    return VS_ERROR;
  if (td->conf.verbose & VS_DEBUG) {
    vs_log_msg(td->conf.modName, "Preprocess transforms:");
  }

  /* relative to absolute (integrate transformations) */
  if (td->conf.relative) {
    VSTransform t = ts[0];
    for (int i = 1; i < trans->len; i++) {
      ts[i] = add_transforms(&ts[i], &t);
      t = ts[i];
    }
  }

  if (td->conf.smoothing>0) {
    VSTransform* ts2 = vs_malloc(sizeof(VSTransform) * trans->len);
    memcpy(ts2, ts, sizeof(VSTransform) * trans->len);
    int s = td->conf.smoothing * 2 + 1;
    VSArray kernel = vs_array_new(s);
    // initialize gaussian kernel
    int mu        = td->conf.smoothing;
    double sigma2 = sqr(mu/2.0);
    for(int i=0; i<=mu; i++){
      kernel.dat[i] = kernel.dat[s-i-1] = exp(-sqr(i-mu)/sigma2);
    }
    // vs_array_print(kernel, stdout);

    for (int i = 0; i < trans->len; i++) {
      // make a convolution:
      double weightsum=0;
      VSTransform avg = null_transform();
      for(int k=0; k<s; k++){
        int idx = i+k-mu;
        if(idx>=0 && idx<trans->len){
          if(unlikely(0 && ts2[idx].extra==1)){ // deal with scene cuts or bad frames
            if(k<mu) { // in the past of our frame: ignore everthing before
              avg=null_transform();
              weightsum=0;
              continue;
            }else{           //current frame or in future: stop here
              if(k==mu)      //for current frame: ignore completely
                weightsum=0;
              break;
            }
          }
          weightsum+=kernel.dat[k];
          avg=add_transforms_(avg, mult_transform(&ts2[idx], kernel.dat[k]));
        }
      }
      if(weightsum>0){
        avg = mult_transform(&avg, 1.0/weightsum);

        // high frequency must be transformed away
        ts[i] = sub_transforms(&ts[i], &avg);
      }
      if (td->conf.verbose & VS_DEBUG) {
        vs_log_msg(td->conf.modName,
                   " avg: %5lf, %5lf, %5lf extra: %i weightsum %5lf",
                   avg.x, avg.y, avg.alpha, ts[i].extra, weightsum
                  );
      }
    }
    vs_array_free(kernel);
    vs_free(ts2);
  }
  return VS_OK;
}

/*
 *  We perform a low-pass filter in terms of transformations.
 *  This supports slow camera movement (low frequency), but in a smooth fasion.
 *  Here a simple average based filter
 */
int cameraPathAvg(VSTransformData* td, VSTransformations* trans){
  VSTransform* ts = trans->ts;

  if (trans->len < 1)
    return VS_ERROR;
  if (td->conf.verbose & VS_DEBUG) {
   vs_log_msg(td->conf.modName, "Preprocess transforms:");
  }
  if (td->conf.smoothing>0) {
    /* smoothing */
    VSTransform* ts2 = vs_malloc(sizeof(VSTransform) * trans->len);
    memcpy(ts2, ts, sizeof(VSTransform) * trans->len);

    /*  we will do a sliding average with minimal update
     *   \hat x_{n/2} = x_1+x_2 + .. + x_n
     *   \hat x_{n/2+1} = x_2+x_3 + .. + x_{n+1} = x_{n/2} - x_1 + x_{n+1}
     *   avg = \hat x / n
     */
    int s = td->conf.smoothing * 2 + 1;
    VSTransform null = null_transform();
    /* avg is the average over [-smoothing, smoothing] transforms
       around the current point */
    VSTransform avg;
    /* avg2 is a sliding average over the filtered signal! (only to past)
     *  with smoothing * 2 horizon to kill offsets */
    VSTransform avg2 = null_transform();
    double tau = 1.0/(2 * s);
    /* initialise sliding sum with hypothetic sum centered around
     * -1st element. We have two choices:
     * a) assume the camera is not moving at the beginning
     * b) assume that the camera moves and we use the first transforms
     */
    VSTransform s_sum = null;
    for (int i = 0; i < td->conf.smoothing; i++){
      s_sum = add_transforms(&s_sum, i < trans->len ? &ts2[i]:&null);
    }
    mult_transform(&s_sum, 2); // choice b (comment out for choice a)

    for (int i = 0; i < trans->len; i++) {
      VSTransform* old = ((i - td->conf.smoothing - 1) < 0)
        ? &null : &ts2[(i - td->conf.smoothing - 1)];
      VSTransform* new = ((i + td->conf.smoothing) >= trans->len)
        ? &null : &ts2[(i + td->conf.smoothing)];
      s_sum = sub_transforms(&s_sum, old);
      s_sum = add_transforms(&s_sum, new);

      avg = mult_transform(&s_sum, 1.0/s);

      /* lowpass filter:
       * meaning high frequency must be transformed away
       */
      ts[i] = sub_transforms(&ts2[i], &avg);
      /* kill accumulating offset in the filtered signal*/
      avg2 = add_transforms_(mult_transform(&avg2, 1 - tau),
                             mult_transform(&ts[i], tau));
      ts[i] = sub_transforms(&ts[i], &avg2);

      if (td->conf.verbose & VS_DEBUG) {
        vs_log_msg(td->conf.modName,
                   "s_sum: %5lf %5lf %5lf, ts: %5lf, %5lf, %5lf\n",
                   s_sum.x, s_sum.y, s_sum.alpha,
                   ts[i].x, ts[i].y, ts[i].alpha);
        vs_log_msg(td->conf.modName,
                   "  avg: %5lf, %5lf, %5lf avg2: %5lf, %5lf, %5lf",
                   avg.x, avg.y, avg.alpha,
                   avg2.x, avg2.y, avg2.alpha);
      }
    }
    vs_free(ts2);
  }
  /* relative to absolute */
  if (td->conf.relative) {
    VSTransform t = ts[0];
    for (int i = 1; i < trans->len; i++) {
      ts[i] = add_transforms(&ts[i], &t);
      t = ts[i];
    }
  }
  return VS_OK;
}


/* Boundary samples per edge used to decide whether a zoom fits.  MUST be
   dense, not just corners and edge midpoints: the extremum of a similarity
   composed with the radial lens map is NOT confined to those eight points.
   The radial map is monotone in distance from the lens centre, which bounds
   the *radius* extremum to a corner, but containment here is tested per axis
   against a rectangle, not against a radius -- composed with a rotation, the
   radial expansion can push a point strictly BETWEEN two corners outside the
   source in x (or y) while both corners stay inside.  Measured counter-
   example: VSLensCorrectFull, k=-0.10, t={x=-25, y=18, alpha=0.03} on a
   640x360 frame -- the eight-point check reports a fit at zoom=7.14%, but
   the true worst destination-boundary point (on an edge, not a corner or
   midpoint) maps 2.47 px outside the source.  64 samples/edge (256 total)
   keeps every excursion of that width or wider inside the sampling
   resolution (~10 px spacing on the long edge of a 640-wide frame; the
   measured counterexample's violating stretch spans only ~5 px, which is
   why the eight-point version missed it but 64/edge does not -- verified
   against test_lensmap_required_zoom's independent, much finer sweep). */
#define VS_LENS_ZOOM_SAMPLES_PER_EDGE 64

/* Does every destination boundary sample land inside the source at this zoom?
   The boundary is walked in DESTINATION dimensions (dw, dh) because that is
   the frame vsLensMapBackward's caller is filling in, pixel by destination
   pixel; the resulting source coordinate is then tested for containment
   against the SOURCE dimensions (sw, sh), because that is the buffer the
   sample is actually read from. The two are the same value today -- the
   transform pass never resizes -- but they are conceptually distinct axes of
   the same map, and conflating them would silently stop being correct the
   day that changes. */
static int lensFitsAtZoom(const VSLensPlaneMap* lm, const VSTransform* t0,
                          double zoom, int dw, int dh, int sw, int sh){
  const int n = VS_LENS_ZOOM_SAMPLES_PER_EDGE;
  VSTransform t = *t0;
  int e, j;
  t.zoom = zoom;
  for(e=0; e<4; e++){
    for(j=0; j<n; j++){
      double u = (double)j/(n-1);   /* 0..1 along this edge, corners included */
      double xd, yd, xs, ys;
      switch(e){
       case 0: xd = u*(dw-1);       yd = 0;              break;
       case 1: xd = dw-1;           yd = u*(dh-1);        break;
       case 2: xd = (1-u)*(dw-1);   yd = dh-1;            break;
       default:xd = 0;              yd = (1-u)*(dh-1);    break;
      }
      if(vsLensMapBackward(lm, &t, xd, yd, &xs, &ys) != VS_OK) return 0;
      if(xs < -0.5 || xs > sw-0.5 || ys < -0.5 || ys > sh-0.5) return 0;
    }
  }
  return 1;
}

double vsTransformRequiredZoom(VSTransformData* td, const VSTransform* t){
  int dw = td->fiDest.width, dh = td->fiDest.height;
  int sw = td->fiSrc.width,  sh = td->fiSrc.height;
  double lo, hi;
  int i;
  lensEnsureMaps(td);
  if(!td->lensActive)
    return transform_get_required_zoom(t, td->fiSrc.width, td->fiSrc.height);
  /* Zoom sits inside M, so the required zoom is a fixed point rather than a
     formula.  Overshoot is monotone in zoom, so bisect. */
  lo = 0.0; hi = 60.0;
  if(lensFitsAtZoom(&td->lensMaps[0], t, lo, dw, dh, sw, sh)) return 0.0;
  if(!lensFitsAtZoom(&td->lensMaps[0], t, hi, dw, dh, sw, sh)) return hi;
  for(i=0; i<15; i++){
    double mid = 0.5*(lo+hi);
    if(lensFitsAtZoom(&td->lensMaps[0], t, mid, dw, dh, sw, sh)) hi = mid; else lo = mid;
  }
  return hi;
}


/**
 * vsPreprocessTransforms: camera path optimization, relative to absolute conversion,
 *  and cropping of too large transforms.
 *
 * Parameters:
 *            td: transform private data structure
 *         trans: list of transformations (changed)
 * Return value:
 *     1 for success and 0 for failure
 * Preconditions:
 *     None
 * Side effects:
 *     td->trans will be modified
 */
int vsPreprocessTransforms(VSTransformData* td, VSTransformations* trans)
{
  // works inplace on trans
  if(cameraPathOptimization(td, trans)!=VS_OK) return VS_ERROR;
  VSTransform* ts = trans->ts;
  /*  invert? */
  if (td->conf.invert) {
    for (int i = 0; i < trans->len; i++) {
      ts[i] = mult_transform(&ts[i], -1);
    }
  }

  /* crop at maximal shift */
  if (td->conf.maxShift != -1)
    for (int i = 0; i < trans->len; i++) {
      ts[i].x     = VS_CLAMP(ts[i].x, -td->conf.maxShift, td->conf.maxShift);
      ts[i].y     = VS_CLAMP(ts[i].y, -td->conf.maxShift, td->conf.maxShift);
    }
  if (td->conf.maxAngle != - 1.0)
    for (int i = 0; i < trans->len; i++)
      ts[i].alpha = VS_CLAMP(ts[i].alpha, -td->conf.maxAngle, td->conf.maxAngle);

  /* Calc optimal zoom (1)
   *  cheap algo is to only consider translations
   *  uses cleaned max and min to eliminate 99% of transforms
   */
  if (td->conf.optZoom == 1 && trans->len > 1){
    VSTransform min_t, max_t;
    cleanmaxmin_xy_transform(ts, trans->len, 1, &min_t, &max_t);  // 99% of all transformations
    // the zoom value only for x
    double zx = 2*VS_MAX(max_t.x,fabs(min_t.x))/td->fiSrc.width;
    // the zoom value only for y
    double zy = 2*VS_MAX(max_t.y,fabs(min_t.y))/td->fiSrc.height;
    td->conf.zoom += 100 * VS_MAX(zx,zy); // use maximum
    td->conf.zoom = VS_CLAMP(td->conf.zoom,-60,60);
    vs_log_info(td->conf.modName, "Final zoom: %lf\n", td->conf.zoom);
  }
  /* Calc optimal zoom (2)
   *  sliding average to zoom only as much as needed also using rotation angles
   *  the baseline zoom is the mean required zoom + global zoom
   *  in order to avoid too much zooming in and out
   */
  if (td->conf.optZoom == 2 && trans->len > 1){
    double* zooms=(double*)vs_zalloc(sizeof(double)*trans->len);
    double req;
    double meanzoom;
    for (int i = 0; i < trans->len; i++) {
      zooms[i] = vsTransformRequiredZoom(td, &ts[i]);
    }

    double prezoom = 0.;
    double postzoom = 0.;
    if(td->conf.zoom>0.){
      prezoom = td->conf.zoom;
    } else if(td->conf.zoom < 0.){
      postzoom = td->conf.zoom;
    }

    meanzoom = mean(zooms, trans->len) + prezoom; // add global zoom
    // forward - propagation (to make the zooming smooth)
    req = meanzoom;
    for (int i = 0; i < trans->len; i++) {
      req = VS_MAX(req, zooms[i]);
      ts[i].zoom=VS_MAX(ts[i].zoom,req);
      req= VS_MAX(meanzoom, req - td->conf.zoomSpeed); // zoom-out each frame
    }
    // backward - propagation
    req = meanzoom;
    for (int i = trans->len-1; i >= 0; i--) {
      req = VS_MAX(req, zooms[i]);
      ts[i].zoom=VS_MAX(ts[i].zoom,req) + postzoom;
      req= VS_MAX(meanzoom, req - td->conf.zoomSpeed);
    }
    vs_free(zooms);
  }else if (td->conf.zoom != 0){ /* apply global zoom */
    for (int i = 0; i < trans->len; i++)
      ts[i].zoom += td->conf.zoom;
  }

  return VS_OK;
}


/**
 * vsLowPassTransforms: single step smoothing of transforms, using only the past.
 *  see also vsPreprocessTransforms. Here only relative transformations are
 *  considered (produced by motiondetection). Also cropping of too large transforms.
 *
 * Parameters:
 *            td: transform private data structure
 *           mem: memory for sliding average transformation
 *         trans: current transform (from previous to current frame)
 * Return value:
 *         new transformation for current frame
 * Preconditions:
 *     None
 */
VSTransform vsLowPassTransforms(VSTransformData* td, VSSlidingAvgTrans* mem,
                                const VSTransform* trans)
{

  if (!mem->initialized){
    // use the first transformation as the average camera movement
    mem->avg=*trans;
    mem->initialized=1;
    mem->zoomavg=0.0;
    mem->accum = null_transform();
    return mem->accum;
  }else{
    double s = 1.0/(td->conf.smoothing + 1);
    double tau = 1.0/(3.0 * (td->conf.smoothing + 1));
    if(td->conf.smoothing>0){
      // otherwise do the sliding window
      mem->avg = add_transforms_(mult_transform(&mem->avg, 1 - s),
                                 mult_transform(trans, s));
    }else{
      mem->avg = *trans;
    }

    /* lowpass filter:
     * meaning high frequency must be transformed away
     */
    VSTransform newtrans = sub_transforms(trans, &mem->avg);

    /* relative to absolute */
    if (td->conf.relative) {
      newtrans = add_transforms(&newtrans, &mem->accum);
      mem->accum = newtrans;
      if(td->conf.smoothing>0){
        // kill accumulating effects
        mem->accum = mult_transform(&mem->accum, 1.0 - tau);
      }
    }

    /* crop at maximal shift */
    if (td->conf.maxShift != -1){
      newtrans.x     = VS_CLAMP(newtrans.x, -td->conf.maxShift, td->conf.maxShift);
      newtrans.y     = VS_CLAMP(newtrans.y, -td->conf.maxShift, td->conf.maxShift);
    }
    if (td->conf.maxAngle != - 1.0)
      newtrans.alpha = VS_CLAMP(newtrans.alpha, -td->conf.maxAngle, td->conf.maxAngle);

    /* Calc sliding optimal zoom
     *  cheap algo is to only consider translations and to sliding avg
     */
    if (td->conf.optZoom != 0 && td->conf.smoothing > 0){
      // the zoom value only for x
      double zx = 2*newtrans.x/td->fiSrc.width;
      // the zoom value only for y
      double zy = 2*newtrans.y/td->fiSrc.height;
      double reqzoom = 100* VS_MAX(fabs(zx),fabs(zy)); // maximum is requried zoom
      mem->zoomavg = (mem->zoomavg*(1-s) + reqzoom*s);
      // since we only use past it is good to aniticipate
      //  and zoom a little in any case (so set td->zoom to 2 or so)
      newtrans.zoom = mem->zoomavg;
    }
    if (td->conf.zoom != 0){
      newtrans.zoom += td->conf.zoom;
    }
    return newtrans;
  }
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
