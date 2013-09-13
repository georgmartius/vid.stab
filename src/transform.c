/*
 *  transform.c
 *
 *  Copyright (C) Georg Martius - June 2007 - 2011
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

#include "transform.h"
#include "transform_internal.h"
#include "transformtype_operations.h"

#include "transformfixedpoint.h"
#ifdef TESTING
#include "transformfloat.h"
#endif

#include <math.h>
#include <libgen.h>
#include <string.h>

const char* interpol_type_names[5] = {"No (0)", "Linear (1)", "Bi-Linear (2)",
                                      "Bi-Cubic (3)"};

const char* getInterpolationTypeName(VSInterpolType type){
  if (type >= VS_Zero && type < VS_NBInterPolTypes)
    return interpol_type_names[(int) type];
  else
    return "unknown";
}

VSTransformConfig vsTransformGetDefaulfConfig(const char* modName){
  VSTransformConfig conf;
  /* Options */
  conf.maxShift           = -1;
  conf.maxAngle           = -1;
  conf.crop               = VSKeepBorder;
  conf.relative           = 1;
  conf.invert             = 0;
  conf.smoothing          = 10;
  conf.zoom               = 0;
  conf.optZoom            = 1;
  conf.interpolType       = VS_BiLinear;
  conf.verbose            = 0;
  conf.modName            = modName;
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

  td->maxAngleVariation  = 1;
  td->rotationThreshhold = 0.25/(180/M_PI);

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
  return VS_OK;
}

void vsTransformDataCleanup(VSTransformData* td){
  if (td->srcMalloced && !vsFrameIsNull(&td->src)) {
    vsFrameFree(&td->src);
  }
  if (td->conf.crop == VSKeepBorder && !vsFrameIsNull(&td->destbuf)) {
    vsFrameFree(&td->destbuf);
  }
}

int vsTransformPrepare(VSTransformData* td, const VSFrame* src, VSFrame* dest){
  // we first copy the frame to td->src and then overwrite the destination
  // with the transformed version
  td->dest = *dest;
  if(src==dest || td->srcMalloced){ // in place operation: we have to copy the src first
    if(vsFrameIsNull(&td->src)) {
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


/**
 * vsPreprocessTransforms: does smoothing, relative to absolute conversion,
 *  and cropping of too large transforms.
 *  This is actually the core algorithm for canceling the jiggle in the
 *  movie. We perform a low-pass filter in terms of transformation size.
 *  This supports slow camera movement (low frequency), but in a smooth fasion.
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
  VSTransform* ts = trans->ts;
  int i;

  if (trans->len < 1)
    return 0;
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
     *  with smoothing * 10 horizont to kill offsets */
    VSTransform avg2 = null_transform();
    double tau = 1.0/(3 * s);
    /* initialise sliding sum with hypothetic sum centered around
     * -1st element. We have two choices:
     * a) assume the camera is not moving at the beginning
     * b) assume that the camera moves and we use the first transforms
     */
    VSTransform s_sum = null;
    for (i = 0; i < td->conf.smoothing; i++){
      s_sum = add_transforms(&s_sum, i < trans->len ? &ts2[i]:&null);
    }
    mult_transform(&s_sum, 2); // choice b (comment out for choice a)

    for (i = 0; i < trans->len; i++) {
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


  /*  invert? */
  if (td->conf.invert) {
    for (i = 0; i < trans->len; i++) {
      ts[i] = mult_transform(&ts[i], -1);
    }
  }

  /* relative to absolute */
  if (td->conf.relative) {
    VSTransform t = ts[0];
    for (i = 1; i < trans->len; i++) {
      if (td->conf.verbose  & VS_DEBUG) {
        vs_log_msg(td->conf.modName, "shift: %5lf   %5lf   %lf \n",
                   t.x, t.y, t.alpha *180/M_PI);
      }
      ts[i] = add_transforms(&ts[i], &t);
      t = ts[i];
    }
  }
  /* crop at maximal shift */
  if (td->conf.maxShift != -1)
    for (i = 0; i < trans->len; i++) {
      ts[i].x     = VS_CLAMP(ts[i].x, -td->conf.maxShift, td->conf.maxShift);
      ts[i].y     = VS_CLAMP(ts[i].y, -td->conf.maxShift, td->conf.maxShift);
    }
  if (td->conf.maxAngle != - 1.0)
    for (i = 0; i < trans->len; i++)
      ts[i].alpha = VS_CLAMP(ts[i].alpha, -td->conf.maxAngle, td->conf.maxAngle);

  /* Calc optimal zoom
   *  cheap algo is to only consider translations
   *  uses cleaned max and min
   * Todo: use sliding average to zoom only as much as needed.
   *       use also rotation angles (transform all four corners)
   *       optzoom=2?
   */
  if (td->conf.optZoom != 0 && trans->len > 1){
    VSTransform min_t, max_t;
    cleanmaxmin_xy_transform(ts, trans->len, 1, &min_t, &max_t);  // 99% of all transformations
    // the zoom value only for x
    double zx = 2*VS_MAX(max_t.x,fabs(min_t.x))/td->fiSrc.width;
    // the zoom value only for y
    double zy = 2*VS_MAX(max_t.y,fabs(min_t.y))/td->fiSrc.height;
    td->conf.zoom += 100* VS_MAX(zx,zy); // use maximum
    vs_log_info(td->conf.modName, "Final zoom: %lf\n", td->conf.zoom);
  }

  /* apply global zoom */
  if (td->conf.zoom != 0){
    for (i = 0; i < trans->len; i++)
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
