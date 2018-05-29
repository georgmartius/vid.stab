/*
 *  transformfixedpoint.h
 *
 *  Copyright (C) Georg Martius - June 2011
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
#ifndef __TRANSFORMFIXEDPOINT_H
#define __TRANSFORMFIXEDPOINT_H

#include "transformtype.h"
#include <stdint.h>

typedef int32_t fp8;
typedef int32_t fp16; // also ncot definition of interpolFun in transform.h

struct _VSTransformData;

/// does the actual transformation in Packed space
int transformPacked(struct _VSTransformData* td, VSTransform t);

/// does the actual transformation in Planar space
int transformPlanar(struct _VSTransformData* td, VSTransform t);

// testing
/// does the actual transformation in Planar space
int transformPlanar_orc(struct _VSTransformData* td, VSTransform t);


/* forward deklarations, please see .c file for documentation*/
void interpolateBiLinBorder(uint8_t *rv, fp16 x, fp16 y,
                            const uint8_t *img, int img_linesize,
                            int w, int h, uint8_t def);
void interpolateBiCub(uint8_t *rv, fp16 x, fp16 y,
                      const uint8_t *img, int img_linesize,
                      int width, int height, uint8_t def);
void interpolateBiLin(uint8_t *rv, fp16 x, fp16 y,
                      const uint8_t *img, int img_linesize,
                      int w, int h, uint8_t def);
void interpolateLin(uint8_t *rv, fp16 x, fp16 y,
                    const uint8_t *img, int img_linesize,
                    int w, int h, uint8_t def);
void interpolateZero(uint8_t *rv, fp16 x, fp16 y,
                     const uint8_t *img, int img_linesize,
                     int w, int h, uint8_t def);
void interpolateN(uint8_t *rv, fp16 x, fp16 y,
                  const uint8_t *img, int img_linesize,
                  int width, int height,
                  uint8_t N, uint8_t channel, uint8_t def);

#endif

/*
 * Local variables:
 *   c-file-style: "stroustrup"
 *   c-file-offsets: ((case-label . *) (statement-case-intro . *))
 *   indent-tabs-mode: nil
 * End:
 *
 * vim: expandtab shiftwidth=4:
 */
