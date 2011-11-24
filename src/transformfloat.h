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
#ifndef __TRANSFORMFLOAT_H
#define __TRANSFORMFLOAT_H

#include "transform.h"

#ifdef TESTING
#define _FLT(n) n ## _float 
#else
#define _FLT(n) n
#endif

/// does the actual transformation in RGB space
int _FLT(transformRGB)(TransformData* td, Transform t);
/// does the actual transformation in YUV space
int _FLT(transformYUV)(TransformData* td, Transform t);

/** 
 * interpolate: general interpolation function pointer for one channel image data
 *
 * Parameters:
 *             rv: destination pixel (call by reference)
 *            x,y: the source coordinates in the image img. Note this 
 *                 are real-value coordinates, that's why we interpolate
 *            img: source image
 *   width,height: dimension of image
 *            def: default value if coordinates are out of range
 * Return value:  None
 */
typedef void (*_FLT(interpolateFun))(unsigned char *rv, float x, float y, 
                               unsigned char* img, int width, int height, 
                               unsigned char def);

extern _FLT(interpolateFun) _FLT(interpolate);

/* forward deklarations, please look in the .c file for documentation*/
void _FLT(interpolateBiLinBorder)(unsigned char *rv, float x, float y, 
                          unsigned char* img, int w, int h, unsigned char def);
void _FLT(interpolateBiCub)(unsigned char *rv, float x, float y, 
                      unsigned char* img, int width, int height, unsigned char def);
void _FLT(interpolateBiLin)(unsigned char *rv, float x, float y, 
                      unsigned char* img, int w, int h, unsigned char def);
void _FLT(interpolateLin)(unsigned char *rv, float x, float y, 
                      unsigned char* img, int w, int h, unsigned char def);
void _FLT(interpolateZero)(unsigned char *rv, float x, float y, 
                     unsigned char* img, int w, int h, unsigned char def);
void _FLT(interpolateN)(unsigned char *rv, float x, float y, 
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
