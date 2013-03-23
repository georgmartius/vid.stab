/*
 * localmotion2transform.c
 *
 *  Copyright (C) Georg Martius - January 2013
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

#include "localmotion2transform.h"
#include "transformtype_operations.h"
#include <assert.h>

int localmotions2TransformsSimple(TransformData* td,
																	const ManyLocalMotions* motions,
																	Transformations* trans ){
	int i;
	int len = vs_vector_size(motions);
	assert(trans->len==0 && trans->ts == 0);
	trans->ts = vs_malloc(sizeof(Transform)*len );
	for(i=0; i< vs_vector_size(motions); i++) {
		trans->ts[i]=simpleMotionsToTransform(td,MLMGet(motions,i));
		//    storeLocalmotions(stderr,MLMGet(motions,i));
		//		storeTransform(stderr,&trans->ts[i]);
	}
	trans->len=len;
	return VS_OK;
}


/* calculates rotation angle for the given transform and
 * field with respect to the given center-point
 */
double calcAngle(const LocalMotion* lm, int center_x, int center_y){
  // we better ignore fields that are to close to the rotation center
  if (abs(lm->f.x - center_x) + abs(lm->f.y - center_y) < lm->f.size*2) {
    return 0;
  } else {
    // double r = sqrt(lm->f.x*lm->f.x + lm->f.y*lm->f.y);
    double a1 = atan2(lm->f.y - center_y, lm->f.x - center_x);
    double a2 = atan2(lm->f.y - center_y + lm->v.y,
                      lm->f.x - center_x + lm->v.x);
    double diff = a2 - a1;
    return (diff > M_PI) ? diff - 2 * M_PI : ((diff < -M_PI) ? diff + 2
					      * M_PI : diff);
  }
}


Transform simpleMotionsToTransform(TransformData* td,
                                   const LocalMotions* motions){
  int center_x = 0;
  int center_y = 0;
	Transform t = null_transform();
  if(motions==0) return t;
  int num_motions=vs_vector_size(motions);
  double *angles = (double*) vs_malloc(sizeof(double) * num_motions);
  LocalMotion meanmotion;
  int i;
  if(num_motions < 1)
    return t;

  // calc center point of all remaining fields
  for (i = 0; i < num_motions; i++) {
    center_x += LMGet(motions,i)->f.x;
    center_y += LMGet(motions,i)->f.y;
  }
  center_x /= num_motions;
  center_y /= num_motions;

  // cleaned mean
  meanmotion = cleanmean_localmotions(motions);

  // figure out angle
  if (num_motions < 6) {
    // the angle calculation is inaccurate for 5 and less fields
    t.alpha = 0;
  } else {
    for (i = 0; i < num_motions; i++) {
      // substract avg and calc angle
      LocalMotion m = sub_localmotion(LMGet(motions,i),&meanmotion);
      angles[i] = calcAngle(&m, center_x, center_y);
    }
    double min, max;
    t.alpha = -cleanmean(angles, num_motions, &min, &max);
    if (max - min > td->maxAngleVariation) {
      t.alpha = 0;
      vs_log_info(td->modName, "too large variation in angle(%f)\n",
		  max-min);
    }
  }
  vs_free(angles);
  // compensate for off-center rotation
  double p_x = (center_x - td->fiSrc.width / 2);
  double p_y = (center_y - td->fiSrc.height / 2);
  t.x = meanmotion.v.x + (cos(t.alpha) - 1) * p_x - sin(t.alpha) * p_y;
  t.y = meanmotion.v.y + sin(t.alpha) * p_x + (cos(t.alpha) - 1) * p_y;

  return t;
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
