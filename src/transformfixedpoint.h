/*
 *  transformfixedpoint.h
 *
 *  Copyright (C) Georg Martius - June 2011
 *
 *  This file is part of transcode, a video stream processing tool
 *      
 *  transcode is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2, or (at your option)
 *  any later version.
 *   
 *  transcode is distributed in the hope that it will be useful,
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

#include "transform.h"

typedef int32_t fp8;
typedef int32_t fp16;

/// does the actual transformation in RGB space
int transformRGB(TransformData* td, Transform t);
/// does the actual transformation in YUV space
int transformYUV(TransformData* td, Transform t);

// testing
/// does the actual transformation in YUV space
int transformYUV_orc(TransformData* td, Transform t);


/** 
 * interpolate: general interpolation function pointer for one channel image data
 *
 * Parameters:
 *             rv: destination pixel (call by reference)
 *            x,y: the source coordinates in the image img. Note this 
 *                 are real-value coordinates (in fixed point format 24.8), 
 *                 that's why we interpolate
 *            img: source image
 *   width,height: dimension of image
 *            def: default value if coordinates are out of range
 * Return value:  None
 */
typedef void (*interpolateFun)(unsigned char *rv, fp16 x, fp16 y, 
                               unsigned char* img, int width, int height, 
                               unsigned char def);

interpolateFun interpolate;

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
