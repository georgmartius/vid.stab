/*
 *  transformfixedpoint.c
 *
 *  Fixed point implementation of image transformations (see also transformfloat.c/h)
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
#include "transformfixedpoint.h"
#include "transform.h"
#include "transformtype_operations.h"

// the orc code does not work at the moment (BUG in ORC?)
// #include "orc/transformorc.h"

//#include <math.h>
//#include <libgen.h>

#define iToFp8(v)  ((v)<<8)
#define fToFp8(v)  ((int32_t)((v)*((float)0xFF)))
#define iToFp16(v) ((v)<<16)
#define fToFp16(v) ((int32_t)((v)*((double)0xFFFF)))
#define fp16To8(v) ((v)>>8)
//#define fp16To8(v) ( (v) && 0x80 == 1 ? ((v)>>8 + 1) : ((v)>>8) )
#define fp24To8(v) ((v)>>16)

#define fp8ToI(v)  ((v)>>8)
#define fp16ToI(v) ((v)>>16)
#define fp8ToF(v)  ((v)/((double)(1<<8)))
#define fp16ToF(v) ((v)/((double)(1<<16)))

// #define fp8ToIRound(v) ( (((v)>>7) & 0x1) == 0 ? ((v)>>8) : ((v)>>8)+1 )
#define fp8_0_5 (1<<7)
#define fp8ToIRound(v) (((v) + fp8_0_5) >> 7)
//#define fp16ToIRound(v) ( (((v)>>15) & 0x1) == 0 ? ((v)>>16) : ((v)>>16)+1 )
#define fp16_0_5 (1<<15)
#define fp16ToIRound(v) (((v) + fp16_0_5) >> 16)

/** interpolateBiLinBorder: bi-linear interpolation function that also works at the border.
    This is used by many other interpolation methods at and outsize the border, see interpolate */
inline void interpolateBiLinBorder(uint8_t *rv, fp16 x, fp16 y,
                                   const uint8_t *img, int img_linesize,
                                   int32_t width, int32_t height, uint8_t def)
{
  int32_t ix_f = fp16ToI(x);
  int32_t iy_f = fp16ToI(y);
  int32_t ix_c = ix_f + 1;
  int32_t iy_c = iy_f + 1;
  if (ix_f < 0 || ix_c >= width || iy_f < 0 || iy_c >= height) {
    int32_t w  = 10; // number of pixels to blur out the border pixel outwards
    int32_t xl = - w - ix_f;
    int32_t yl = - w - iy_f;
    int32_t xh = ix_c - w - width;
    int32_t yh = iy_c - w - height;
    int32_t c = VS_MAX(VS_MIN(VS_MAX(xl, VS_MAX(yl, VS_MAX(xh, yh))),w),0);
    // pixel at border of source image
    short val_border = PIX(img, img_linesize, VS_MAX(VS_MIN(ix_f, width-1),0),
                           VS_MAX(VS_MIN(iy_f, height-1),0));
    int32_t res = (def * c + val_border * (w - c)) / w;
    *rv = (res >= 0) ? ((res < 255) ? res : 255) : 0;
  }else{
    short v1 = PIXEL(img, img_linesize, ix_c, iy_c, width, height, def);
    short v2 = PIXEL(img, img_linesize, ix_c, iy_f, width, height, def);
    short v3 = PIXEL(img, img_linesize, ix_f, iy_c, width, height, def);
    short v4 = PIXEL(img, img_linesize, ix_f, iy_f, width, height, def);
    fp16 x_f = iToFp16(ix_f);
    fp16 x_c = iToFp16(ix_c);
    fp16 y_f = iToFp16(iy_f);
    fp16 y_c = iToFp16(iy_c);
    fp16 s   = fp16To8(v1*(x - x_f)+v3*(x_c - x))*fp16To8(y - y_f) +
      fp16To8(v2*(x - x_f) + v4*(x_c - x))*fp16To8(y_c - y) + 1;
    int32_t res = fp16ToIRound(s);
    *rv = (res >= 0) ? ((res < 255) ? res : 255) : 0;
  }
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
/* inline static short bicub_kernel(fp16 t, short a0, short a1, short a2, short a3){ */
/*   // (2*a1 + t*((-a0+a2) + t*((2*a0-5*a1+4*a2-a3) + t*(-a0+3*a1-3*a2+a3) )) ) / 2; */
/*   return ((iToFp16(2*a1) + t*(-a0+a2 */
/*             + fp16ToI(t*((2*a0-5*a1+4*a2-a3) */
/*              + fp16ToI(t*(-a0+3*a1-3*a2+a3)) )) ) */
/*      ) ) >> 17; */
/* } */

inline static short bicub_kernel(fp16 t, short a0, short a1, short a2, short a3){
  // (2*a1 + t*((-a0+a2) + t*((2*a0-5*a1+4*a2-a3) + t*(-a0+3*a1-3*a2+a3) )) ) / 2;
  // we add 1/2 because of truncation errors
  return fp16ToIRound((iToFp16(2*a1) + t*(-a0+a2
                                          + fp16ToIRound(t*((2*a0-5*a1+4*a2-a3)
                                                            + fp16ToIRound(t*(-a0+3*a1-3*a2+a3)) )) )
                       ) >> 1);
}

/** interpolateBiCub: bi-cubic interpolation function using 4x4 pixel, see interpolate */
inline void interpolateBiCub(uint8_t *rv, fp16 x, fp16 y,
                             const uint8_t *img, int img_linesize,
                             int width, int height, uint8_t def)
{
  // do a simple linear interpolation at the border
  int32_t ix_f = fp16ToI(x);
  int32_t iy_f = fp16ToI(y);
  if (unlikely(ix_f < 1 || ix_f > width - 3 || iy_f < 1 || iy_f > height - 3)) {
    interpolateBiLinBorder(rv, x, y, img, img_linesize, width, height, def);
  } else {
    fp16 x_f = iToFp16(ix_f);
    fp16 y_f = iToFp16(iy_f);
    fp16 tx  = x-x_f;
    short v1 = bicub_kernel(tx,
                            PIX(img, img_linesize, ix_f-1, iy_f-1),
                            PIX(img, img_linesize, ix_f,   iy_f-1),
                            PIX(img, img_linesize, ix_f+1, iy_f-1),
                            PIX(img, img_linesize, ix_f+2, iy_f-1));
    short v2 = bicub_kernel(tx,
                            PIX(img, img_linesize, ix_f-1, iy_f),
                            PIX(img, img_linesize, ix_f,   iy_f),
                            PIX(img, img_linesize, ix_f+1, iy_f),
                            PIX(img, img_linesize, ix_f+2, iy_f));
    short v3 = bicub_kernel(tx,
                            PIX(img, img_linesize, ix_f-1, iy_f+1),
                            PIX(img, img_linesize, ix_f,   iy_f+1),
                            PIX(img, img_linesize, ix_f+1, iy_f+1),
                            PIX(img, img_linesize, ix_f+2, iy_f+1));
    short v4 = bicub_kernel(tx,
                            PIX(img, img_linesize, ix_f-1, iy_f+2),
                            PIX(img, img_linesize, ix_f,   iy_f+2),
                            PIX(img, img_linesize, ix_f+1, iy_f+2),
                            PIX(img, img_linesize, ix_f+2, iy_f+2));
    short res = bicub_kernel(y-y_f, v1, v2, v3, v4);
    *rv = (res >= 0) ? ((res < 255) ? res : 255) : 0;
  }
}


/** interpolateBiLin: bi-linear interpolation function, see interpolate */
inline void interpolateBiLin(uint8_t *rv, fp16 x, fp16 y,
                             const uint8_t *img, int img_linesize,
                             int32_t width, int32_t height, uint8_t def)
{
  int32_t ix_f = fp16ToI(x);
  int32_t iy_f = fp16ToI(y);
  if (unlikely(ix_f < 0 || ix_f > width - 2 || iy_f < 0 || iy_f > height - 2)) {
    interpolateBiLinBorder(rv, x, y, img, img_linesize, width, height, def);
  } else {
    int32_t ix_c = ix_f + 1;
    int32_t iy_c = iy_f + 1;
    short v1 = PIX(img, img_linesize, ix_c, iy_c);
    short v2 = PIX(img, img_linesize, ix_c, iy_f);
    short v3 = PIX(img, img_linesize, ix_f, iy_c);
    short v4 = PIX(img, img_linesize, ix_f, iy_f);
    fp16 x_f = iToFp16(ix_f);
    fp16 x_c = iToFp16(ix_c);
    fp16 y_f = iToFp16(iy_f);
    fp16 y_c = iToFp16(iy_c);
    fp16 s  = fp16To8(v1*(x - x_f) + v3*(x_c - x))*fp16To8(y - y_f) +
      fp16To8(v2*(x - x_f) + v4*(x_c - x))*fp16To8(y_c - y);
    // it is underestimated due to truncation, so we add one
    short res = fp16ToI(s);
    *rv = (res >= 0) ? ((res < 255) ? res+1 : 255) : 0;
  }
}

/** interpolateLin: linear (only x) interpolation function, see interpolate */
inline void interpolateLin(uint8_t *rv, fp16 x, fp16 y,
                           const uint8_t *img, int img_linesize,
                           int width, int height, uint8_t def)
{
  int32_t ix_f = fp16ToI(x);
  int32_t ix_c = ix_f + 1;
  fp16    x_c  = iToFp16(ix_c);
  fp16    x_f  = iToFp16(ix_f);
  int     y_n  = fp16ToIRound(y);

  short v1 = PIXEL(img, img_linesize, ix_c, y_n, width, height, def);
  short v2 = PIXEL(img, img_linesize, ix_f, y_n, width, height, def);
  fp16 s   = v1*(x - x_f) + v2*(x_c - x);
  short res = fp16ToI(s);
  *rv = (res >= 0) ? ((res < 255) ? res : 255) : 0;
}

/** interpolateZero: nearest neighbor interpolation function, see interpolate */
inline void interpolateZero(uint8_t *rv, fp16 x, fp16 y,
                            const uint8_t *img, int img_linesize,
                            int width, int height, uint8_t def)
{
  int32_t ix_n = fp16ToIRound(x);
  int32_t iy_n = fp16ToIRound(y);
  int32_t res = PIXEL(img, img_linesize, ix_n, iy_n, width, height, def);
  *rv = (res >= 0) ? ((res < 255) ? res : 255) : 0;
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
inline void interpolateN(uint8_t *rv, fp16 x, fp16 y,
                         const uint8_t *img, int img_linesize,
                         int width, int height,
                         uint8_t N, uint8_t channel,
                         uint8_t def)
{
  int32_t ix_f = fp16ToI(x);
  int32_t iy_f = fp16ToI(y);
  int32_t ix_c = ix_f + 1;
  int32_t iy_c = iy_f + 1;
  if (ix_f < 0 || ix_f > width-1 || iy_f < 0 || iy_f > height - 1
          || ix_c < 0 || ix_c > width-1 || iy_c < 0 || iy_c > height - 1) {
    *rv = def;
  } else {
    short v1 = PIXN(img, img_linesize, ix_c, iy_c, N, channel);
    short v2 = PIXN(img, img_linesize, ix_c, iy_f, N, channel);
    short v3 = PIXN(img, img_linesize, ix_f, iy_c, N, channel);
    short v4 = PIXN(img, img_linesize, ix_f, iy_f, N, channel);
    fp16 x_f = iToFp16(ix_f);
    fp16 x_c = iToFp16(ix_c);
    fp16 y_f = iToFp16(iy_f);
    fp16 y_c = iToFp16(iy_c);
    fp16 s  = fp16To8(v1*(x - x_f)+v3*(x_c - x))*fp16To8(y - y_f) +
      fp16To8(v2*(x - x_f) + v4*(x_c - x))*fp16To8(y_c - y);
    int32_t res = fp16ToIRound(s);
    *rv = (res >= 0) ? ((res < 255) ? res : 255) : 0;
  }
}


/**
 * transformPacked: applies current transformation to frame
 * Parameters:
 *         td: private data structure of this filter
 * Return value:
 *         0 for failture, 1 for success
 * Preconditions:
 *  The frame must be in Packed format
 */
int transformPacked(VSTransformData* td, VSTransform t)
{
  int x = 0, y = 0, k = 0;
  uint8_t *D_1, *D_2;

  D_1  = td->src.data[0];
  D_2  = td->destbuf.data[0];
  fp16 c_s_x = iToFp16(td->fiSrc.width/2);
  fp16 c_s_y = iToFp16(td->fiSrc.height/2);
  int32_t c_d_x = td->fiDest.width/2;
  int32_t c_d_y = td->fiDest.height/2;

  /* for each pixel in the destination image we calc the source
   * coordinate and make an interpolation:
   *      p_d = c_d + M(p_s - c_s) + t
   * where p are the points, c the center coordinate,
   *  _s source and _d destination,
   *  t the translation, and M the rotation matrix
   *      p_s = M^{-1}(p_d - c_d - t) + c_s
   */
  float z     = 1.0-t.zoom/100.0;
  fp16 zcos_a = fToFp16(z*cos(-t.alpha)); // scaled cos
  fp16 zsin_a = fToFp16(z*sin(-t.alpha)); // scaled sin
  fp16  c_tx    = c_s_x - fToFp16(t.x);
  fp16  c_ty    = c_s_y - fToFp16(t.y);
  int channels = td->fiSrc.bytesPerPixel;
  /* All channels */
  for (y = 0; y < td->fiDest.height; y++) {
    int32_t y_d1 = (y - c_d_y);
    for (x = 0; x < td->fiDest.width; x++) {
      int32_t x_d1 = (x - c_d_x);
      fp16 x_s  =  zcos_a * x_d1 + zsin_a * y_d1 + c_tx;
      fp16 y_s  = -zsin_a * x_d1 + zcos_a * y_d1 + c_ty;

      for (k = 0; k < channels; k++) { // iterate over colors
        uint8_t *dest = &D_2[x * channels + y * td->destbuf.linesize[0] + k];
        interpolateN(dest, x_s, y_s, D_1, td->src.linesize[0],
                     td->fiSrc.width, td->fiSrc.height,
                     channels, k, td->conf.crop ? 16 : *dest);
      }
    }
  }
  return VS_OK;
}

/**
 * transformPlanar: applies current transformation to frame
 *
 * Parameters:
 *         td: private data structure of this filter
 * Return value:
 *         0 for failture, 1 for success
 * Preconditions:
 *  The frame must be in Planar format
 *
 * Fixed-point format 32 bit integer:
 *  for image coords we use val<<8
 *  for angle and zoom we use val<<16
 *
 */
int transformPlanar(VSTransformData* td, VSTransform t)
{
  int32_t x = 0, y = 0;
  uint8_t *dat_1, *dat_2;

  if (t.alpha==0 && t.x==0 && t.y==0 && t.zoom == 0){
    if(vsFramesEqual(&td->src,&td->destbuf))
      return VS_OK; // noop
    else {
      vsFrameCopy(&td->destbuf, &td->src, &td->fiSrc);
      return VS_OK;
    }
  }

  int plane;
  for(plane=0; plane< td->fiSrc.planes; plane++){
    dat_1  = td->src.data[plane];
    dat_2  = td->destbuf.data[plane];
    int wsub = vsGetPlaneWidthSubS(&td->fiSrc,plane);
    int hsub = vsGetPlaneHeightSubS(&td->fiSrc,plane);
    int dw = CHROMA_SIZE(td->fiDest.width , wsub);
    int dh = CHROMA_SIZE(td->fiDest.height, hsub);
    int sw = CHROMA_SIZE(td->fiSrc.width  , wsub);
    int sh = CHROMA_SIZE(td->fiSrc.height , hsub);
    uint8_t black = plane==0 ? 0 : 0x80;

    fp16 c_s_x = iToFp16(sw / 2);
    fp16 c_s_y = iToFp16(sh / 2);
    int32_t c_d_x = dw / 2;
    int32_t c_d_y = dh / 2;

    float z     = 1.0-t.zoom/100.0;
    fp16 zcos_a = fToFp16(z*cos(-t.alpha)); // scaled cos
    fp16 zsin_a = fToFp16(z*sin(-t.alpha)); // scaled sin
    fp16  c_tx    = c_s_x - (fToFp16(t.x) >> wsub);
    fp16  c_ty    = c_s_y - (fToFp16(t.y) >> hsub);

    /* for each pixel in the destination image we calc the source
     * coordinate and make an interpolation:
     *      p_d = c_d + M(p_s - c_s) + t
     * where p are the points, c the center coordinate,
     *  _s source and _d destination,
     *  t the translation, and M the rotation and scaling matrix
     *      p_s = M^{-1}(p_d - c_d - t) + c_s
     */
    for (y = 0; y < dh; y++) {
      // swapping of the loops brought 15% performace gain
      int32_t y_d1 = (y - c_d_y);
      for (x = 0; x < dw; x++) {
        int32_t x_d1 = (x - c_d_x);
        fp16 x_s  =  zcos_a * x_d1 + zsin_a * y_d1 + c_tx;
        fp16 y_s  = -zsin_a * x_d1 + zcos_a * y_d1 + c_ty;
        uint8_t *dest = &dat_2[x + y * td->destbuf.linesize[plane]];
        // inlining the interpolation function would bring 10%
        //  (but then we cannot use the function pointer anymore...)
        td->interpolate(dest, x_s, y_s, dat_1,
                        td->src.linesize[plane], sw, sh,
                        td->conf.crop ? black : *dest);
      }
    }
  }

  return VS_OK;
}



/* /\** TESTING */
/*  * transformPlanar_orc: applies current transformation to frame */
/*  * */
/*  * Parameters: */
/*  *         td: private data structure of this filter */
/*  * Return value:  */
/*  *         0 for failture, 1 for success */
/*  * Preconditions: */
/*  *  The frame must be in Planar format */
/*  * */
/*  * Fixed-point format 32 bit integer: */
/*  *  for image coords we use val<<8 */
/*  *  for angle and zoom we use val<<16 */
/*  * */
/*  *\/ */
/* int transformPlanar_orc(VSTransformData* td, VSTransform t) */
/* { */
/*     int32_t x = 0, y = 0; */
/*     uint8_t *Y_1, *Y_2, *Cb_1, *Cb_2, *Cr_1, *Cr_2; */

/*     if (t.alpha==0 && t.x==0 && t.y==0 && t.zoom == 0) return VS_OK; // noop */

/*     Y_1  = td->src;   */
/*     Y_2  = td->destbuf;   */
/*     Cb_1 = td->src + td->fiSrc.width * td->fiSrc.height; */
/*     Cb_2 = td->destbuf + td->fiDest.width * td->fiDest.height; */
/*     Cr_1 = td->src + 5*td->fiSrc.width * td->fiSrc.height/4; */
/*     Cr_2 = td->destbuf + 5*td->fiDest.width * td->fiDest.height/4; */
/*     fp16 c_s_x = iToFp16(td->fiSrc.width / 2); */
/*     fp16 c_s_y = iToFp16(td->fiSrc.height / 2); */
/*     int32_t c_d_x = td->fiDest.width / 2; */
/*     int32_t c_d_y = td->fiDest.height / 2;     */

/*     float z     = 1.0-t.zoom/100.0; */
/*     fp16 zcos_a = fToFp16(z*cos(-t.alpha)); // scaled cos */
/*     fp16 zsin_a = fToFp16(z*sin(-t.alpha)); // scaled sin */
/*     fp16  c_tx    = c_s_x - fToFp16(t.x); */
/*     fp16  c_ty    = c_s_y - fToFp16(t.y); */

/*     /\* for each pixel in the destination image we calc the source */
/*      * coordinate and make an interpolation:  */
/*      *      p_d = c_d + M(p_s - c_s) + t  */
/*      * where p are the points, c the center coordinate,  */
/*      *  _s source and _d destination,  */
/*      *  t the translation, and M the rotation and scaling matrix */
/*      *      p_s = M^{-1}(p_d - c_d - t) + c_s */
/*      *\/ */
/*     /\* Luminance channel *\/ */
/*     fp16* x_ss = (fp16*)malloc(sizeof(fp16)*td->fiDest.width); */
/*     fp16* y_ss = (fp16*)malloc(sizeof(fp16)*td->fiDest.width);     */
/*     int32_t* xs = (int32_t*)malloc(sizeof(int32_t)*td->fiDest.width);         */
/*     for (x = 0; x < td->fiDest.width; x++) { // this can go to td */
/*       xs[x]=x; */
/*     } */

/*     for (y = 0; y < td->fiDest.height; y++) { */
/*       int32_t y_d1 = (y - c_d_y);   */
/*       fp16 sin_y   = zsin_a * y_d1; */
/*       fp16 cos_y   = zcos_a * y_d1; */
/*       for (x = 0; x < td->fiDest.width; x++) { */
/*         int32_t x_d1 = (xs[x] - c_d_x); */
/*         //x_ss[x]  =  zcos_a * x_d1 + zsin_a * y_d1 + c_tx; */
/*   y_ss[x]  = -zsin_a * x_d1 + zcos_a * y_d1 + c_ty; */
/*       } */
/*       transform_one_line_optimized1 (x_ss, y_ss, xs, y_d1, c_d_x,  */
/*              c_tx, c_ty, zcos_a, zsin_a, sin_y, cos_y,  */
/*              td->fiDest.width); */
/*       // transform_one_line_optimized (x_ss, y_ss, xs, y_d1, c_d_x,  */
/*       //             c_tx, c_ty, zcos_a, zsin_a, td->fiDest.width); */

/*       for (x = 0; x < td->fiDest.width; x++) { */
/*   uint8_t *dest = &Y_2[x + y * td->fiDest.width]; */
/*   td->interpolate(dest, x_ss[x], y_ss[x], Y_1,  */
/*         td->fiSrc.width, td->fiSrc.height,  */
/*         td->crop ? 16 : *dest); */
/*       } */
/*     } */

/*     /\* Color channels *\/ */
/*     int32_t ws2 = td->fiSrc.width/2; */
/*     int32_t wd2 = td->fiDest.width/2; */
/*     int32_t hs2 = td->fiSrc.height/2; */
/*     int32_t hd2 = td->fiDest.height/2; */
/*     fp16 c_tx2   = c_tx/2; */
/*     fp16 c_ty2   = c_ty/2; */

/*     for (y = 0; y < hd2; y++) { */
/*       int32_t y_d1 = y - (c_d_y)/2; */
/*       for (x = 0; x < wd2; x++) { */
/*   int32_t x_d1 = x - (c_d_x)/2; */
/*   fp16 x_s  =  zcos_a * x_d1 + zsin_a * y_d1 + c_tx2; */
/*   fp16 y_s  = -zsin_a * x_d1 + zcos_a * y_d1 + c_ty2;  */
/*   uint8_t *dest = &Cr_2[x + y * wd2]; */
/*   td->interpolate(dest, x_s, y_s, Cr_1, ws2, hs2,  */
/*         td->crop ? 128 : *dest); */
/*   dest = &Cb_2[x + y * wd2]; */
/*   td->interpolate(dest, x_s, y_s, Cb_1, ws2, hs2,  */
/*         td->crop ? 128 : *dest); */
/*       } */
/*     } */

/*     return VS_OK; */
/* } */

/*
  some debugging stuff
  FILE* f1 = fopen("transFP.pos","w");
  fprintf(f1,"%i,%i:\t %f,%f\n", x, y, x_s / (float)(1<<16), y_s / (float)(1<<16));
  fclose(f1);

*/



/*
 * Local variables:
 *   c-file-style: "stroustrup"
 *   c-file-offsets: ((case-label . *) (statement-case-intro . *))
 *   indent-tabs-mode: nil
 *   c-basic-offset: 2 t
 *
 * End:
 *
 * vim: expandtab shiftwidth=2:
 */
