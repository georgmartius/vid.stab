/*
 *  transformfloat.c
 *
 *  Floating point image transformations
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
#include "transformfloat.h"
#include "transform.h"


/** interpolateBiLinBorder: bi-linear interpolation function that also works at the border.
    This is used by many other interpolation methods at and outsize the border, see interpolate */
void _FLT(interpolateBiLinBorder)(unsigned char *rv, float x, float y,
                                  unsigned char* img, int width, int height,
                                  unsigned char def)
{
  int x_f = myfloor(x);
  int x_c = x_f+1;
  int y_f = myfloor(y);
  int y_c = y_f+1;
  short v1 = PIXEL(img, x_c, y_c, width, height, def);
  short v2 = PIXEL(img, x_c, y_f, width, height, def);
  short v3 = PIXEL(img, x_f, y_c, width, height, def);
  short v4 = PIXEL(img, x_f, y_f, width, height, def);
  float s  = (v1*(x - x_f)+v3*(x_c - x))*(y - y_f) +
    (v2*(x - x_f) + v4*(x_c - x))*(y_c - y);
  *rv = (unsigned char)s;
}

/** taken from http://en.wikipedia.org/wiki/Bicubic_interpolation for alpha=-0.5
    in matrix notation:
    a0-a3 are the neigthboring points where the target point is between a1 and a2
    t is the point of interpolation (position between a1 and a2) value between 0 and 1
    | 0, 2, 0, 0 |  |a0|
    |-1, 0, 1, 0 |  |a1|
    (1,t,t^2,t^3) | 2,-5, 4,-1 |  |a2|
    |-1, 3,-3, 1 |  |a3|
*/
static short _FLT(bicub_kernel)(float t, short a0, short a1, short a2, short a3){
  return (2*a1 + t*((-a0+a2) + t*((2*a0-5*a1+4*a2-a3) + t*(-a0+3*a1-3*a2+a3) )) ) / 2;
}

/** interpolateBiCub: bi-cubic interpolation function using 4x4 pixel, see interpolate */
void _FLT(interpolateBiCub)(unsigned char *rv, float x, float y,
                            unsigned char* img, int width, int height, unsigned char def)
{
  // do a simple linear interpolation at the border
  if (x < 1 || x > width - 2 || y < 1 || y > height - 2) {
    _FLT(interpolateBiLinBorder)(rv, x,y,img,width,height,def);
  } else {
    int x_f = myfloor(x);
    int y_f = myfloor(y);
    float tx = x-x_f;
    short v1 = _FLT(bicub_kernel)(tx,
                                  PIX(img, x_f-1, y_f-1, width, height),
                                  PIX(img, x_f,   y_f-1, width, height),
                                  PIX(img, x_f+1, y_f-1, width, height),
                                  PIX(img, x_f+2, y_f-1, width, height));
    short v2 = _FLT(bicub_kernel)(tx,
                                  PIX(img, x_f-1, y_f, width, height),
                                  PIX(img, x_f,   y_f, width, height),
                                  PIX(img, x_f+1, y_f, width, height),
                                  PIX(img, x_f+2, y_f, width, height));
    short v3 = _FLT(bicub_kernel)(tx,
                                  PIX(img, x_f-1, y_f+1, width, height),
                                  PIX(img, x_f,   y_f+1, width, height),
                                  PIX(img, x_f+1, y_f+1, width, height),
                                  PIX(img, x_f+2, y_f+1, width, height));
    short v4 = _FLT(bicub_kernel)(tx,
                                  PIX(img, x_f-1, y_f+2, width, height),
                                  PIX(img, x_f,   y_f+2, width, height),
                                  PIX(img, x_f+1, y_f+2, width, height),
                                  PIX(img, x_f+2, y_f+2, width, height));
    *rv = (unsigned char)_FLT(bicub_kernel)(y-y_f, v1, v2, v3, v4);
  }
}


/** interpolateBiLin: bi-linear interpolation function, see interpolate */
void _FLT(interpolateBiLin)(unsigned char *rv, float x, float y,
                            unsigned char* img, int width, int height,
                            unsigned char def)
{
  if (x < 0 || x > width - 1 || y < 0 || y > height - 1) {
    _FLT(interpolateBiLinBorder)(rv, x, y, img, width, height, def);
  } else {
    int x_f = myfloor(x);
    int x_c = x_f+1;
    int y_f = myfloor(y);
    int y_c = y_f+1;
    short v1 = PIX(img, x_c, y_c, width, height);
    short v2 = PIX(img, x_c, y_f, width, height);
    short v3 = PIX(img, x_f, y_c, width, height);
    short v4 = PIX(img, x_f, y_f, width, height);
    float s  = (v1*(x - x_f)+v3*(x_c - x))*(y - y_f) +
      (v2*(x - x_f) + v4*(x_c - x))*(y_c - y);
    *rv = (unsigned char)s;
  }
}


/** interpolateLin: linear (only x) interpolation function, see interpolate */
void _FLT(interpolateLin)(unsigned char *rv, float x, float y,
                          unsigned char* img, int width, int height,
                          unsigned char def)
{
  int x_f = myfloor(x);
  int x_c = x_f+1;
  int y_n = myround(y);
  float v1 = PIXEL(img, x_c, y_n, width, height, def);
  float v2 = PIXEL(img, x_f, y_n, width, height, def);
  float s  = v1*(x - x_f) + v2*(x_c - x);
  *rv = (unsigned char)s;
}

/** interpolateZero: nearest neighbor interpolation function, see interpolate */
void _FLT(interpolateZero)(unsigned char *rv, float x, float y,
                           unsigned char* img, int width, int height, unsigned char def)
{
  int x_n = myround(x);
  int y_n = myround(y);
  *rv = (unsigned char) PIXEL(img, x_n, y_n, width, height, def);
}


/**
 * interpolateN: Bi-linear interpolation function for N channel image.
 *
 * Parameters:
 *             rv: destination pixel (call by reference)
 *            x,y: the source coordinates in the image img. Note this
 *                 are real-value coordinates, that's why we interpolate
 *            img: source image
 *   width,height: dimension of image
 *              N: number of channels
 *        channel: channel number (0..N-1)
 *            def: default value if coordinates are out of range
 * Return value:  None
 */
void _FLT(interpolateN)(unsigned char *rv, float x, float y,
                        unsigned char* img, int width, int height,
                        unsigned char N, unsigned char channel,
                        unsigned char def)
{
  if (x < - 1 || x > width || y < -1 || y > height) {
    *rv = def;
  } else {
    int x_f = myfloor(x);
    int x_c = x_f+1;
    int y_f = myfloor(y);
    int y_c = y_f+1;
    short v1 = PIXELN(img, x_c, y_c, width, height, N, channel, def);
    short v2 = PIXELN(img, x_c, y_f, width, height, N, channel, def);
    short v3 = PIXELN(img, x_f, y_c, width, height, N, channel, def);
    short v4 = PIXELN(img, x_f, y_f, width, height, N, channel, def);
    float s  = (v1*(x - x_f)+v3*(x_c - x))*(y - y_f) +
      (v2*(x - x_f) + v4*(x_c - x))*(y_c - y);
    *rv = (unsigned char)s;
  }
}


/**
 * transformRGB: applies current transformation to frame
 * Parameters:
 *         td: private data structure of this filter
 * Return value:
 *         0 for failture, 1 for success
 * Preconditions:
 *  The frame must be in RGB format
 /// TODO Add zoom!
 */
int _FLT(transformRGB)(TransformData* td, Transform t)
{
  int x = 0, y = 0, z = 0;
  unsigned char *D_1, *D_2;

  D_1  = td->src;
  D_2  = td->destbuf;
  float c_s_x = td->fiSrc.width/2.0;
  float c_s_y = td->fiSrc.height/2.0;
  float c_d_x = td->fiDest.width/2.0;
  float c_d_y = td->fiDest.height/2.0;

  /* for each pixel in the destination image we calc the source
   * coordinate and make an interpolation:
   *      p_d = c_d + M(p_s - c_s) + t
   * where p are the points, c the center coordinate,
   *  _s source and _d destination,
   *  t the translation, and M the rotation matrix
   *      p_s = M^{-1}(p_d - c_d - t) + c_s
   */
  /* All 3 channels */
  if (fabs(t.alpha) > td->rotationThreshhold) {
    for (x = 0; x < td->fiDest.width; x++) {
      for (y = 0; y < td->fiDest.height; y++) {
        float x_d1 = (x - c_d_x);
        float y_d1 = (y - c_d_y);
        float x_s  =  cos(-t.alpha) * x_d1
          + sin(-t.alpha) * y_d1 + c_s_x -t.x;
        float y_s  = -sin(-t.alpha) * x_d1
          + cos(-t.alpha) * y_d1 + c_s_y -t.y;
        for (z = 0; z < 3; z++) { // iterate over colors
          unsigned char* dest = &D_2[(x + y * td->fiDest.width)*3+z];
          _FLT(interpolateN)(dest, x_s, y_s, D_1,
                             td->fiSrc.width, td->fiSrc.height,
                             3, z, td->crop ? 16 : *dest);
        }
      }
    }
  }else {
    /* no rotation, just translation
     *(also no interpolation, since no size change (so far)
     */
    int round_tx = myround(t.x);
    int round_ty = myround(t.y);
    for (x = 0; x < td->fiDest.width; x++) {
      for (y = 0; y < td->fiDest.height; y++) {
        for (z = 0; z < 3; z++) { // iterate over colors
          short p = PIXELN(D_1, x - round_tx, y - round_ty,
                           td->fiSrc.width, td->fiSrc.height, 3, z, -1);
          if (p == -1) {
            if (td->crop == 1)
              D_2[(x + y * td->fiDest.width)*3+z] = 16;
          } else {
            D_2[(x + y * td->fiDest.width)*3+z] = (unsigned char)p;
          }
        }
      }
    }
  }
  return 1;
}

/**
 * transformYUV: applies current transformation to frame
 *
 * Parameters:
 *         td: private data structure of this filter
 * Return value:
 *         0 for failture, 1 for success
 * Preconditions:
 *  The frame must be in YUV format
 */
int _FLT(transformYUV)(TransformData* td, Transform t)
{
  int x = 0, y = 0;
  unsigned char *Y_1, *Y_2, *Cb_1, *Cb_2, *Cr_1, *Cr_2;

  if (t.alpha==0 && t.x==0 && t.y==0 && t.zoom == 0){
    if(td->src==td->destbuf)
      return DS_OK; // noop
    else {
      // FIXME: if framesizes differ this does not work
      memcpy(td->destbuf, td->src, td->fiSrc.framesize);
      return DS_OK;
    }
  }


  Y_1  = td->src;
  Y_2  = td->destbuf;
  Cb_1 = td->src     + td->fiSrc.width * td->fiSrc.height;
  Cb_2 = td->destbuf + td->fiDest.width * td->fiDest.height;
  Cr_1 = td->src     + 5*td->fiSrc.width * td->fiSrc.height/4;
  Cr_2 = td->destbuf + 5*td->fiDest.width * td->fiDest.height/4;
  float c_s_x = td->fiSrc.width/2.0;
  float c_s_y = td->fiSrc.height/2.0;
  float c_d_x = td->fiDest.width/2.0;
  float c_d_y = td->fiDest.height/2.0;

  float z = 1.0-t.zoom/100;
  float zcos_a = z*cos(-t.alpha); // scaled cos
  float zsin_a = z*sin(-t.alpha); // scaled sin

  /* for each pixel in the destination image we calc the source
   * coordinate and make an interpolation:
   *      p_d = c_d + M(p_s - c_s) + t
   * where p are the points, c the center coordinate,
   *  _s source and _d destination,
   *  t the translation, and M the rotation and scaling matrix
   *      p_s = M^{-1}(p_d - c_d - t) + c_s
   */
  /* Luminance channel */
  for (x = 0; x < td->fiDest.width; x++) {
    for (y = 0; y < td->fiDest.height; y++) {
      float x_d1 = (x - c_d_x);
      float y_d1 = (y - c_d_y);
      float x_s  =  zcos_a * x_d1
        + zsin_a * y_d1 + c_s_x -t.x;
      float y_s  = -zsin_a * x_d1
        + zcos_a * y_d1 + c_s_y -t.y;
      unsigned char* dest = &Y_2[x + y * td->fiDest.width];
      td->_FLT(interpolate)(dest, x_s, y_s, Y_1,
                            td->fiSrc.width, td->fiSrc.height,
                            td->crop ? 16 : *dest);
    }
  }

  /* Color channels */
  int ws2 = td->fiSrc.width/2;
  int wd2 = td->fiDest.width/2;
  int hs2 = td->fiSrc.height/2;
  int hd2 = td->fiDest.height/2;
  for (x = 0; x < wd2; x++) {
    for (y = 0; y < hd2; y++) {
      float x_d1 = x - (c_d_x)/2;
      float y_d1 = y - (c_d_y)/2;
      float x_s  =  zcos_a * x_d1
        + zsin_a * y_d1 + (c_s_x -t.x)/2;
      float y_s  = -zsin_a * x_d1
        + zcos_a * y_d1 + (c_s_y -t.y)/2;
      unsigned char* dest = &Cr_2[x + y * wd2];
      td->_FLT(interpolate)(dest, x_s, y_s, Cr_1, ws2, hs2,
                            td->crop ? 128 : *dest);
      dest = &Cb_2[x + y * wd2];
      td->_FLT(interpolate)(dest, x_s, y_s, Cb_1, ws2, hs2,
                            td->crop ? 128 : *dest);
    }
  }

  return DS_OK;
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
