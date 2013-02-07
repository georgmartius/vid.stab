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
 *   WITH THE RESTRICTION for NONCOMMERICIAL USAGE see below,
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
 *  This work is licensed under the Creative Commons
 *  Attribution-NonCommercial-ShareAlike 2.5 License. To view a copy of
 *  this license, visit http://creativecommons.org/licenses/by-nc-sa/2.5/
 *  or send a letter to Creative Commons, 543 Howard Street, 5th Floor,
 *  San Francisco, California, 94105, USA.
 *  This EXCLUDES COMMERCIAL USAGE
 *
 */

#include "transform.h"

#include "transformfixedpoint.h"
#ifdef TESTING
#include "transformfloat.h"
#endif

#include <math.h>
#include <libgen.h>

const char* interpolTypes[5] = {"No (0)", "Linear (1)", "Bi-Linear (2)",
                                "Bi-Cubic (3)"};


int initTransformData(TransformData* td, const DSFrameInfo* fi_src,
                      const DSFrameInfo* fi_dest , const char* modName){
    td->modName = modName;

    td->fiSrc = *fi_src;
    td->fiDest = *fi_dest;

    td->src= NULL;
    td->srcMalloced = 0;

    td->destbuf = 0;
    td->dest = 0;

    /* Options */
    td->maxShift = -1;
    td->maxAngle = -1;
    td->maxAngleVariation = 1;


    td->crop = KeepBorder;
    td->relative = 1;
    td->invert = 0;
    td->smoothing = 10;

    td->rotationThreshhold = 0.25/(180/M_PI);

    td->zoom    = 0;
    td->optZoom = 1;
    td->interpolType = BiLinear;
    td->sharpen = 0.8;

    td->verbose = 0;
    return DS_OK;
}

int configureTransformData(TransformData* td){
    if (td->maxShift > td->fiDest.width/2)
        td->maxShift = td->fiDest.width/2;
    if (td->maxShift > td->fiDest.height/2)
        td->maxShift = td->fiDest.height/2;

    td->interpolType = DS_MAX(DS_MIN(td->interpolType,BiCubic),Zero);

    switch(td->interpolType){
      case Zero:     td->interpolate = &interpolateZero; break;
      case Linear:   td->interpolate = &interpolateLin; break;
      case BiLinear: td->interpolate = &interpolateBiLin; break;
      case BiCubic:  td->interpolate = &interpolateBiCub; break;
      default: td->interpolate = &interpolateBiLin;
    }
#ifdef TESTING
    switch(td->interpolType){
      case Zero:     td->_FLT(interpolate) = &_FLT(interpolateZero); break;
      case Linear:   td->_FLT(interpolate) = &_FLT(interpolateLin); break;
      case BiLinear: td->_FLT(interpolate) = &_FLT(interpolateBiLin); break;
      case BiCubic:  td->_FLT(interpolate) = &_FLT(interpolateBiCub); break;
      default: td->_FLT(interpolate)	   = &_FLT(interpolateBiLin);
    }

#endif
    return DS_OK;
}

void cleanupTransformData(TransformData* td){
    if (td->srcMalloced && td->src) {
        ds_free(td->src);
        td->src = NULL;
    }
    if (td->crop == KeepBorder && td->destbuf) {
        ds_free(td->destbuf);
        td->destbuf = NULL;
    }
}

int transformPrepare(TransformData* td, const unsigned char* src, unsigned char* dest){
    // we first copy the frame to the src and then overwrite the destination
    // with the transformed version
    td->dest = dest;
    if(src==dest || td->srcMalloced){ // in place operation: we have to copy the src first
        if(td->src == NULL) {
            td->src = ds_malloc(td->fiSrc.framesize);
            td->srcMalloced = 1;
        }
        if (td->src == NULL) {
            ds_log_error(td->modName, "ds_malloc failed\n");
            return DS_ERROR;
        }
        memcpy(td->src, src, td->fiSrc.framesize);
    }else{ // otherwise no copy needed
        td->src=(unsigned char *)src;
    }
    if (td->crop == KeepBorder) {
      if(td->destbuf == 0) {
          // if we keep the borders, we need a second buffer to store
          //  the previous stabilized frame, so we use destbuf
          td->destbuf = ds_malloc(td->fiDest.framesize);
          if (td->destbuf == NULL) {
              ds_log_error(td->modName, "ds_malloc failed\n");
              return DS_ERROR;
          }
          // if we keep borders, save first frame into the background buffer (destbuf)
          memcpy(td->destbuf, src, td->fiSrc.framesize);
      }
    }else{ // otherwise we directly operate on the destination
        td->destbuf = dest;
    }
    return DS_OK;
}

int transformFinish(TransformData* td){
    if(td->crop == KeepBorder){
        // we have to store our result to video buffer
        // note: destbuf stores stabilized frame to be the default for next frame
        memcpy(td->dest, td->destbuf, td->fiSrc.framesize);
    }
    return DS_OK;
}


Transform getNextTransform(const TransformData* td, Transformations* trans){
    if(trans->len <=0 ) return null_transform();
    if (trans->current >= trans->len) {
        trans->current = trans->len;
        if(!trans->warned_end)
            ds_log_warn(td->modName, "not enough transforms found, use last transformation!\n");
        trans->warned_end = 1;
    }else{
        trans->current++;
    }
    return trans->ts[trans->current-1];
}

void initTransformations(Transformations* trans){
    trans->ts = 0;
    trans->len = 0;
    trans->current = 0;
    trans->warned_end = 0;
}

void cleanupTransformations(Transformations* trans){
    if (trans->ts) {
        ds_free(trans->ts);
        trans->ts = NULL;
    }
    trans->len=0;
}


/**
 * preprocessTransforms: does smoothing, relative to absolute conversion,
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
int preprocessTransforms(TransformData* td, Transformations* trans)
{
    Transform* ts = trans->ts;
    int i;

    if (trans->len < 1)
        return 0;
    if (td->verbose & DS_DEBUG) {
        ds_log_msg(td->modName, "Preprocess transforms:");
    }
    if (td->smoothing>0) {
        /* smoothing */
        Transform* ts2 = ds_malloc(sizeof(Transform) * trans->len);
        memcpy(ts2, ts, sizeof(Transform) * trans->len);

        /*  we will do a sliding average with minimal update
         *   \hat x_{n/2} = x_1+x_2 + .. + x_n
         *   \hat x_{n/2+1} = x_2+x_3 + .. + x_{n+1} = x_{n/2} - x_1 + x_{n+1}
         *   avg = \hat x / n
         */
        int s = td->smoothing * 2 + 1;
        Transform null = null_transform();
        /* avg is the average over [-smoothing, smoothing] transforms
           around the current point */
        Transform avg;
        /* avg2 is a sliding average over the filtered signal! (only to past)
         *  with smoothing * 10 horizont to kill offsets */
        Transform avg2 = null_transform();
        double tau = 1.0/(3 * s);
        /* initialise sliding sum with hypothetic sum centered around
         * -1st element. We have two choices:
         * a) assume the camera is not moving at the beginning
         * b) assume that the camera moves and we use the first transforms
         */
        Transform s_sum = null;
        for (i = 0; i < td->smoothing; i++){
            s_sum = add_transforms(&s_sum, i < trans->len ? &ts2[i]:&null);
        }
        mult_transform(&s_sum, 2); // choice b (comment out for choice a)

        for (i = 0; i < trans->len; i++) {
            Transform* old = ((i - td->smoothing - 1) < 0)
                ? &null : &ts2[(i - td->smoothing - 1)];
            Transform* new = ((i + td->smoothing) >= trans->len)
                ? &null : &ts2[(i + td->smoothing)];
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

            if (td->verbose & DS_DEBUG) {
                ds_log_msg(td->modName,
                           "s_sum: %5lf %5lf %5lf, ts: %5lf, %5lf, %5lf\n",
                           s_sum.x, s_sum.y, s_sum.alpha,
                           ts[i].x, ts[i].y, ts[i].alpha);
                ds_log_msg(td->modName,
                           "  avg: %5lf, %5lf, %5lf avg2: %5lf, %5lf, %5lf",
                           avg.x, avg.y, avg.alpha,
                           avg2.x, avg2.y, avg2.alpha);
            }
        }
        ds_free(ts2);
    }


    /*  invert? */
    if (td->invert) {
        for (i = 0; i < trans->len; i++) {
            ts[i] = mult_transform(&ts[i], -1);
        }
    }

    /* relative to absolute */
    if (td->relative) {
        Transform t = ts[0];
        for (i = 1; i < trans->len; i++) {
            if (td->verbose  & DS_DEBUG) {
                ds_log_msg(td->modName, "shift: %5lf   %5lf   %lf \n",
                           t.x, t.y, t.alpha *180/M_PI);
            }
            ts[i] = add_transforms(&ts[i], &t);
            t = ts[i];
        }
    }
    /* crop at maximal shift */
    if (td->maxShift != -1)
        for (i = 0; i < trans->len; i++) {
            ts[i].x     = DS_CLAMP(ts[i].x, -td->maxShift, td->maxShift);
            ts[i].y     = DS_CLAMP(ts[i].y, -td->maxShift, td->maxShift);
        }
    if (td->maxAngle != - 1.0)
        for (i = 0; i < trans->len; i++)
            ts[i].alpha = DS_CLAMP(ts[i].alpha, -td->maxAngle, td->maxAngle);

    /* Calc optimal zoom
     *  cheap algo is to only consider translations
     *  uses cleaned max and min
     * Todo: use sliding average to zoom only as much as needed.
     *       use also rotation angles (transform all four corners)
     *       optzoom=2?
     */
    if (td->optZoom != 0 && trans->len > 1){
        Transform min_t, max_t;
        cleanmaxmin_xy_transform(ts, trans->len, 10, &min_t, &max_t);
        // the zoom value only for x
        double zx = 2*DS_MAX(max_t.x,fabs(min_t.x))/td->fiSrc.width;
        // the zoom value only for y
        double zy = 2*DS_MAX(max_t.y,fabs(min_t.y))/td->fiSrc.height;
        td->zoom += 100* DS_MAX(zx,zy); // use maximum
        ds_log_info(td->modName, "Final zoom: %lf\n", td->zoom);
    }

    /* apply global zoom */
    if (td->zoom != 0){
        for (i = 0; i < trans->len; i++)
            ts[i].zoom += td->zoom;
    }

    return DS_OK;
}


/**
 * lowPassTransforms: single step smoothing of transforms, using only the past.
 *  see also preprocessTransforms. Here only relative transformations are
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
Transform lowPassTransforms(TransformData* td, SlidingAvgTrans* mem,
                            const Transform* trans)
{

  if (!mem->initialized){
    // use the first transformation as the average camera movement
    mem->avg=*trans;
    mem->initialized=1;
    mem->zoomavg=0.0;
    mem->accum = null_transform();
    return mem->accum;
  }else{
    double s = 1.0/(td->smoothing + 1);
    double tau = 1.0/(3.0 * (td->smoothing + 1));
    if(td->smoothing>0){
      // otherwise do the sliding window
      mem->avg = add_transforms_(mult_transform(&mem->avg, 1 - s),
				 mult_transform(trans, s));
    }else{
      mem->avg = *trans;
    }

    /* lowpass filter:
     * meaning high frequency must be transformed away
     */
    Transform newtrans = sub_transforms(trans, &mem->avg);

    /* relative to absolute */
    if (td->relative) {
      newtrans = add_transforms(&newtrans, &mem->accum);
      mem->accum = newtrans;
      if(td->smoothing>0){
	// kill accumulating effects
	mem->accum = mult_transform(&mem->accum, 1.0 - tau);
      }
    }

    /* crop at maximal shift */
    if (td->maxShift != -1){
      newtrans.x     = DS_CLAMP(newtrans.x, -td->maxShift, td->maxShift);
      newtrans.y     = DS_CLAMP(newtrans.y, -td->maxShift, td->maxShift);
    }
    if (td->maxAngle != - 1.0)
      newtrans.alpha = DS_CLAMP(newtrans.alpha, -td->maxAngle, td->maxAngle);

    /* Calc sliding optimal zoom
     *  cheap algo is to only consider translations and to sliding avg
     */
    if (td->optZoom != 0 && td->smoothing > 0){
      // the zoom value only for x
      double zx = 2*newtrans.x/td->fiSrc.width;
      // the zoom value only for y
      double zy = 2*newtrans.y/td->fiSrc.height;
      double reqzoom = 100* DS_MAX(fabs(zx),fabs(zy)); // maximum is requried zoom
      mem->zoomavg = (mem->zoomavg*(1-s) + reqzoom*s);
      // since we only use past it is good to aniticipate
      //  and zoom a little in any case (so set td->zoom to 2 or so)
      newtrans.zoom = mem->zoomavg;
    }
    if (td->zoom != 0){
      newtrans.zoom += td->zoom;
    }
    return newtrans;
  }
}

/*
 * Local variables:
 *   c-file-style: "stroustrup"
 *   c-file-offsets: ((case-label . *) (statement-case-intro . *))
 *   indent-tabs-mode: nil
 * End:
 *
 * vim: expandtab shiftwidth=4:
 */
