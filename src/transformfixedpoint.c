/*
 *  transformfixedpoint.c
 *
 *  Fixed point implementation of image transformations (see also transformfloat.c/h)
 *
 *  Copyright (C) Georg Martius - June 2011
 *   georg dot martius at web dot de
 *
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  This file is part of vid.stab video stabilization library
 *
 *  vid.stab is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published
 *  by the Free Software Foundation; either version 2.1 of the License, or
 *  (at your option) any later version.
 *
 *  vid.stab is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with vid.stab; see the file COPYING.LESSER.  If not, see
 *  <https://www.gnu.org/licenses/>.
 *
 *
 */
#include "transformfixedpoint.h"
#include "transform.h"
#include "transformtype_operations.h"

//#include <math.h>
//#include <libgen.h>

/* Left shifting a negative signed value is undefined behaviour, and these are
   reached with negative coordinates whenever a sample falls outside the frame
   (interpolateLin, interpolateBiLinBorder, ...). Shift as unsigned and convert
   back: identical bit pattern on two's complement, but well defined. */
#define iToFp8(v)  ((int32_t)((uint32_t)(v)<<8))
#define fToFp8(v)  ((int32_t)((v)*((float)0xFF)))
#define iToFp16(v) ((int32_t)((uint32_t)(v)<<16))
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
void interpolateBiLinBorder(uint8_t *rv, fp16 x, fp16 y,
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
void interpolateBiCub(uint8_t *rv, fp16 x, fp16 y,
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
void interpolateBiLin(uint8_t *rv, fp16 x, fp16 y,
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
void interpolateLin(uint8_t *rv, fp16 x, fp16 y,
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
void interpolateZero(uint8_t *rv, fp16 x, fp16 y,
                            const uint8_t *img, int img_linesize,
                            int width, int height, uint8_t def)
{
  int32_t ix_n = fp16ToIRound(x);
  int32_t iy_n = fp16ToIRound(y);
  int32_t res = PIXEL(img, img_linesize, ix_n, iy_n, width, height, def);
  *rv = (res >= 0) ? ((res < 255) ? res : 255) : 0;
}


/**
 * blendBiLinN: the bi-linear blend of four already fetched samples.
 *
 * Factored out so that interpolateN() (one channel, range checked) and
 * interpolateNall() (all channels of one pixel, indices hoisted) share one
 * copy of the arithmetic and cannot drift apart.
 *   v1 = (x_c,y_c)  v2 = (x_c,y_f)  v3 = (x_f,y_c)  v4 = (x_f,y_f)
 */
static uint8_t blendBiLinN(short v1, short v2, short v3, short v4,
                           fp16 x, fp16 x_f, fp16 x_c,
                           fp16 y, fp16 y_f, fp16 y_c)
{
  fp16 s  = fp16To8(v1*(x - x_f)+v3*(x_c - x))*fp16To8(y - y_f) +
    fp16To8(v2*(x - x_f) + v4*(x_c - x))*fp16To8(y_c - y);
  int32_t res = fp16ToIRound(s);
  return (res >= 0) ? ((res < 255) ? res : 255) : 0;
}

/**
 * interpolateN: Bi-linear interpolation function for N channel image.
 *
 * Parameters:
 *             rv: destination pixel (call by reference)
 *            x,y: the source coordinates in the image img. Note this
 *                 are real-value coordinates, that's why we interpolate
 *            img: source image
 *   img_linesize: distance between two rows in BYTES (see vidstabdefines.h)
 *   width,height: dimension of image in PIXELS
 *              N: number of channels (bytes per pixel)
 *        channel: channel number (0..N-1)
 *            def: default value if coordinates are out of range
 * Return value:  None
 */
void interpolateN(uint8_t *rv, fp16 x, fp16 y,
                         const uint8_t *img, int img_linesize,
                         int width, int height,
                         uint8_t N, uint8_t channel,
                         uint8_t def)
{
  int32_t ix_f = fp16ToI(x);
  int32_t iy_f = fp16ToI(y);
  // fp16ToI is an arithmetic shift, so it floors: negative x give negative ix_f
  if (ix_f < 0 || ix_f > width-1 || iy_f < 0 || iy_f > height - 1) {
    *rv = def;
  } else {
    int32_t ix_c = ix_f + 1;
    int32_t iy_c = iy_f + 1;
    short v1, v2, v3, v4;
    /* ix_f/iy_f are in range by the test above, so only the "ceil" neighbours
       can be one past the last row/column.  Away from that border no bound can
       be violated and the unchecked accessor is used; on it the range checked
       one supplies def, exactly as before. */
    if (ix_c < width && iy_c < height) {
      v1 = PIXN(img, img_linesize, ix_c, iy_c, N, channel);
      v2 = PIXN(img, img_linesize, ix_c, iy_f, N, channel);
      v3 = PIXN(img, img_linesize, ix_f, iy_c, N, channel);
      v4 = PIXN(img, img_linesize, ix_f, iy_f, N, channel);
    } else {
      v1 = PIXELN(img, img_linesize, ix_c, iy_c, width, height, N, channel, def);
      v2 = PIXELN(img, img_linesize, ix_c, iy_f, width, height, N, channel, def);
      v3 = PIXELN(img, img_linesize, ix_f, iy_c, width, height, N, channel, def);
      v4 = PIXELN(img, img_linesize, ix_f, iy_f, width, height, N, channel, def);
    }
    *rv = blendBiLinN(v1, v2, v3, v4, x, iToFp16(ix_f), iToFp16(ix_c),
                      y, iToFp16(iy_f), iToFp16(iy_c));
  }
}

/**
 * interpolateNall: interpolateN() for every channel of one destination pixel.
 *
 * transformPacked() used to call interpolateN() once per channel, which
 * recomputed the source indices and the interpolation weights N times for the
 * same pixel.  Here they are computed once and only the four sample loads and
 * the blend happen per channel.
 *
 * Parameters: as interpolateN(), except
 *           dest: the N consecutive destination channels of one pixel
 *           crop: if nonzero, out of range pixels become 16 (black), otherwise
 *                 the destination keeps the value it already had
 */
static void interpolateNall(uint8_t *dest, fp16 x, fp16 y,
                            const uint8_t *img, int img_linesize,
                            int width, int height, uint8_t N, char crop)
{
  int32_t ix_f = fp16ToI(x);
  int32_t iy_f = fp16ToI(y);
  uint8_t k;
  if (ix_f < 0 || ix_f > width-1 || iy_f < 0 || iy_f > height - 1) {
    if (crop)
      for (k = 0; k < N; k++)
        dest[k] = 16;
    /* else: leave the destination untouched, like interpolateN(.., def=*dest) */
  } else if (ix_f + 1 < width && iy_f + 1 < height) {
    /* interior: no sample can be out of range, so no per sample bound check */
    fp16 x_f = iToFp16(ix_f), x_c = iToFp16(ix_f + 1);
    fp16 y_f = iToFp16(iy_f), y_c = iToFp16(iy_f + 1);
    const uint8_t *rowf = img + ix_f*N + iy_f*img_linesize;
    const uint8_t *rowc = rowf + img_linesize;
    for (k = 0; k < N; k++)
      dest[k] = blendBiLinN(rowc[N+k], rowf[N+k], rowc[k], rowf[k],
                            x, x_f, x_c, y, y_f, y_c);
  } else {
    /* last row or column: the checked path, one channel at a time */
    for (k = 0; k < N; k++)
      interpolateN(&dest[k], x, y, img, img_linesize, width, height,
                   N, k, crop ? 16 : dest[k]);
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
  int x = 0, y = 0;
  uint8_t *D_1, *D_2;

  if (t.alpha==0 && t.x==0 && t.y==0 && t.zoom == 0){
    // identity: like transformPlanar, do not run the interpolation at all
    if(vsFramesEqual(&td->src,&td->destbuf))
      return VS_OK; // noop
    else {
      vsFrameCopy(&td->destbuf, &td->src, &td->fiSrc);
      return VS_OK;
    }
  }

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

      // linesize is in bytes, only the column is scaled by the channel count
      interpolateNall(&D_2[x * channels + y * td->destbuf.linesize[0]],
                      x_s, y_s, D_1, td->src.linesize[0],
                      td->fiSrc.width, td->fiSrc.height,
                      channels, td->conf.crop);
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
    /* A chroma sample spans (1<<wsub) luma columns and (1<<hsub) luma rows, so
       for wsub!=hsub (4:2:2, 4:4:0, 4:1:1) the plane's two axes are not to the
       same scale. The rotation mixes the axes, so its cross terms have to be
       converted between them: rotate in luma units and come back, which turns
       the sine into sin*(ay/ax) for the x row and sin*(ax/ay) for the y row.
       Doing it in float here rather than by shifting the fp16 product per
       pixel keeps the full fixed point precision and costs nothing in the
       loop. The scaling (cosine) terms stay put: they are diagonal, so they
       never leave their own axis. With wsub==hsub both factors are 1 and this
       is bit-identical to the plain rotation -- which is why only the
       asymmetric formats were ever wrong (issue #79). */
    float ax = (float)(1 << wsub), ay = (float)(1 << hsub);
    float zsin  = z*sin(-t.alpha);
    fp16 zcos_a = fToFp16(z*cos(-t.alpha)); // scaled cos
    fp16 zsin_xy = fToFp16(zsin * (ay/ax)); // scaled sin, y_d1 -> x_s
    fp16 zsin_yx = fToFp16(zsin * (ax/ay)); // scaled sin, x_d1 -> y_s
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
        fp16 x_s  =  zcos_a  * x_d1 + zsin_xy * y_d1 + c_tx;
        fp16 y_s  = -zsin_yx * x_d1 + zcos_a  * y_d1 + c_ty;
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
