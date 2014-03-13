/*
 *  campathoptimization.c
 *
 *  Copyright (C) Georg Martius - Jan - 2014
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

#include <math.h>
#include <string.h>

#include "campathoptimization.h"
#include "vidstabdefines.h"
#include "l1campathoptimization.h"

/*
 *  This is actually the core algorithm for canceling the jiggle in the
 *  movie. We have different implementations which are patched here.
 */
int cameraPathOptimization(VSTransformData* td, VSTransformations* trans){
  switch(td->conf.camPathAlgo){
   case VSAvg: return cameraPathAvg(td,trans);
   case VSOptimalL1:
#ifdef USE_GLPK
    return cameraPathOptimalL1(td,trans);
#endif // otherwise use gaussian
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

/***********************************************************************
 * helper functions to create and operate with transformsLS.
 * all functions are non-destructive
 */

/* create an initialized transform*/
VSTransformLS new_transformLS(double x, double y, double a, double b, double c, int extra) {
  VSTransformLS t;
  t.x        = x;
  t.y        = y;
  t.a        = a;
  t.b        = b;
  t.c        = c;
  t.extra    = extra;
  return t;
}

/* create an identity transformLS*/
VSTransformLS id_transformLS(void) {
  return new_transformLS(0, 0, 1.0, 0, 1.0, 0);
}

VSTransformLS sub_transformLS(const VSTransformLS* t1, const VSTransformLS* t2){
  VSTransformLS t;
  t.x     = t1->x - t2->x;
  t.y     = t1->y - t2->y;
  t.a     = t1->a - t2->a;
  t.b     = t1->b - t2->b;
  t.c     = t1->c - t2->c;
  t.extra = t1->extra || t2->extra;
  return t;
}

VSTransformLS sub_transformLS_(VSTransformLS t1, VSTransformLS t2){
  VSTransformLS t;
  t.x     = t1.x - t2.x;
  t.y     = t1.y - t2.y;
  t.a     = t1.a - t2.a;
  t.b     = t1.b - t2.b;
  t.c     = t1.c - t2.c;
  t.extra = t1.extra || t2.extra;
  return t;
}
VSTransformLS add_transformLS_(VSTransformLS t1, VSTransformLS t2){
  VSTransformLS t;
  t.x     = t1.x + t2.x;
  t.y     = t1.y + t2.y;
  t.a     = t1.a + t2.a;
  t.b     = t1.b + t2.b;
  t.c     = t1.c + t2.c;
  t.extra = t1.extra || t2.extra;
  return t;
}

/* sequentially apply two transformLS: t1 then t2*/
VSTransformLS concat_transformLS(const VSTransformLS* t1, const VSTransformLS* t2)
{
  VSTransformLS t;
  t.x        = t2->x*t1->c + t1->x*t2->a + t1->y*t2->b;
  t.y        = t2->y*t1->c + t1->y*t2->a - t1->x*t2->b;
  t.a        = t1->a*t2->a - t1->b*t2->b;
  t.b        = t2->a*t1->b + t1->a*t2->b;
  t.c        = t1->c * t2->c;
  t.extra    = t1->extra || t2->extra;
  return t;
}

// returns an inverted transformLS
VSTransformLS invert_transformLS(const VSTransformLS* t){
  VSTransformLS r;
  double z = t->a*t->a + t->b*t->b;
  r.x        = (-t->a*t->x + t->b*t->y)/(z*t->c);
  r.y        = -(t->b*t->x + t->a*t->y)/(z*t->c);
  r.a        = t->a/z;
  r.b        = -t->b/z;
  r.c        = 1/t->c;
  r.extra    = t->extra;
  return r;
}


void storeVSTransformLS(FILE* f, const VSTransformLS* t){
  fprintf(f,"TransLS %lf %lf %lf %lf %i\n", t->x, t->y, t->a, t->b, t->extra);
}

VSTransformLS transformAZtoLS(const VSTransform* t){
  VSTransformLS r;
  r.x      = t->x;
  r.y      = t->y;
  double z = zoom2z(t->zoom);
  r.a      = z*cos(t->alpha);
  r.b      = -z*sin(t->alpha);
  r.c      = 1;
  r.extra  = t->extra;
  return r;
}

VSTransform transformLStoAZ(const VSTransformLS* t){
  VSTransform r;
  if(t->c!=1.0) {
    fprintf(stderr,"bad conversion! c=%f!=1\n",t->c);
  }
  r.x     = t->x;
  r.y     = t->y;
  r.alpha = atan2(-t->b,t->a);
  r.zoom  = z2zoom(t->a/cos(r.alpha));
  r.extra = t->extra;
  return r;
}


VSTransformationsLS transformationsAZtoLS(const VSTransformations* trans){
  VSTransformationsLS ts;
  ts.ts=vs_malloc(sizeof(VSTransformLS)*trans->len);
  ts.len=trans->len;
  ts.current=trans->current;
  for(int i=0; i<trans->len; i++){
    ts.ts[i]=transformAZtoLS(&trans->ts[i]);
  }
  return ts;
}

PreparedTransform prepare_transformLS(const VSTransformLS* t, const VSFrameInfo* fi){
  PreparedTransform pt;
  pt.x      = t->x;
  pt.y      = t->y;
  pt.zcos_a = t->a;
  pt.zsin_a = -t->b;
  pt.c_x    = fi->width / 2;
  pt.c_y    = fi->height / 2;
  return pt;
}
