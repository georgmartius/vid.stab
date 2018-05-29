/*
 *  transformfloat.h
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
 *
 */
#ifndef __TRANSFORMFLOAT_H
#define __TRANSFORMFLOAT_H

#include "transformtype.h"
#include <stdint.h>

#ifdef TESTING
#define _FLT(n) n ## _float
#else
#define _FLT(n) n
#endif

struct _VSTransformData;

/// does the actual transformation in Packed space
int _FLT(transformPacked)(struct _VSTransformData* td, VSTransform t);
/// does the actual transformation in Planar space
int _FLT(transformPlanar)(struct _VSTransformData* td, VSTransform t);

/**
 * interpolate: general interpolation function pointer for one channel image data
 *
 * Parameters:
 *             rv: destination pixel (call by reference)
 *            x,y: the source coordinates in the image img. Note this
 *                 are real-value coordinates, that's why we interpolate
 *            img: source image
 *   img_linesize: length of one line in bytes (>= width)
 *   width,height: dimension of image
 *            def: default value if coordinates are out of range
 * Return value:  None
 */
typedef void (*_FLT(vsInterpolateFun))(uint8_t *rv, float x, float y,
                                       const uint8_t *img, int img_linesize,
                                       int width, int height, uint8_t def);

/* forward deklarations, please look in the .c file for documentation*/
void _FLT(interpolateBiLinBorder)(uint8_t *rv, float x, float y,
                                  const uint8_t *img, int img_linesize,
                                  int w, int h, uint8_t def);
void _FLT(interpolateBiCub)(uint8_t *rv, float x, float y,
                            const uint8_t *img, int img_linesize,
                            int width, int height, uint8_t def);
void _FLT(interpolateBiLin)(uint8_t *rv, float x, float y,
                            const uint8_t *img, int img_linesize,
                            int w, int h, uint8_t def);
void _FLT(interpolateLin)(uint8_t *rv, float x, float y,
                          const uint8_t *img, int img_linesize,
                          int w, int h, uint8_t def);
void _FLT(interpolateZero)(uint8_t *rv, float x, float y,
                           const uint8_t *img, int img_linesize,
                           int w, int h, uint8_t def);
void _FLT(interpolateN)(uint8_t *rv, float x, float y,
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
