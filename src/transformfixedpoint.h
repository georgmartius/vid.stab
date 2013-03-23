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
 *  This work is licensed under the Creative Commons
 *  Attribution-NonCommercial-ShareAlike 2.5 License. To view a copy of
 *  this license, visit http://creativecommons.org/licenses/by-nc-sa/2.5/
 *  or send a letter to Creative Commons, 543 Howard Street, 5th Floor,
 *  San Francisco, California, 94105, USA.
 *  This EXCLUDES COMMERCIAL USAGE
 *
 */
#ifndef __TRANSFORMFIXEDPOINT_H
#define __TRANSFORMFIXEDPOINT_H

#include "transformtype.h"
#include <stdint.h>

typedef int32_t fp8;
typedef int32_t fp16; // also not definition of interpolFun in transform.h

struct _TransformData;

/// does the actual transformation in RGB space
int transformRGB(struct _TransformData* td, Transform t);
/// does the actual transformation in YUV space
int transformYUV(struct _TransformData* td, Transform t);

// testing
/// does the actual transformation in YUV space
int transformYUV_orc(struct _TransformData* td, Transform t);


/* forward deklarations, please see .c file for documentation*/
void interpolateBiLinBorder(unsigned char *rv, fp16 x, fp16 y,
                          unsigned char* img, int w, int h, unsigned char def);
void interpolateBiCub(unsigned char *rv, fp16 x, fp16 y,
                      unsigned char* img, int width, int height, unsigned char def);
void interpolateBiLin(unsigned char *rv, fp16 x, fp16 y,
                      unsigned char* img, int w, int h, unsigned char def);
void interpolateLin(unsigned char *rv, fp16 x, fp16 y,
                      unsigned char* img, int w, int h, unsigned char def);
void interpolateZero(unsigned char *rv, fp16 x, fp16 y,
                     unsigned char* img, int w, int h, unsigned char def);
void interpolateN(unsigned char *rv, fp16 x, fp16 y,
                  unsigned char* img, int width, int height,
                  unsigned char N, unsigned char channel, unsigned char def);

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
