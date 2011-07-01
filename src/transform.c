/*
 *  transform.c
 *
 *  Copyright (C) Georg Martius - June 2007
 *   georg dot martius at web dot de  
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

#include "transform.h"

#include <math.h>
#include <libgen.h>

typedef int32_t fp8;
typedef int32_t fp16;
#define iToFp8(v)  ((v)<<8)
#define fToFp8(v)  ((int32_t)((v)*((float)0xFF)))
#define iToFp16(v) ((v)<<16)
#define fToFp16(v) ((int32_t)((v)*((double)0xFFFF)))
#define fp16To8(v) ((v)>>8)

#define fp8ToI(v) ((v)>>8)
#define fp16ToI(v) ((v)>>16)


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
typedef void (*interpolateFun)(unsigned char *rv, float x, float y, 
                               unsigned char* img, int width, int height, 
                               unsigned char def);

static interpolateFun interpolate;
// extern void (*interpolate)(unsigned char *rv, float x, float y, 
//                            unsigned char* img, int width, int height, 
//                            unsigned char def);

const char* interpolTypes[5] = {"No (0)", "Linear (1)", "Bi-Linear (2)", 
                                       "Bi-Quadratic (3)", "Bi-Cubic (4)"};

/* forward deklarations, please look below for documentation*/
void interpolateBiLinBorder(unsigned char *rv, float x, float y, 
                          unsigned char* img, int w, int h, unsigned char def);
void interpolateBiCub(unsigned char *rv, float x, float y, 
                      unsigned char* img, int width, int height, unsigned char def);
void interpolateSqr(unsigned char *rv, float x, float y, 
                    unsigned char* img, int w, int h, unsigned char def);
void interpolateBiLin(unsigned char *rv, float x, float y, 
                      unsigned char* img, int w, int h, unsigned char def);
void interpolateLin(unsigned char *rv, float x, float y, 
                      unsigned char* img, int w, int h, unsigned char def);
void interpolateZero(unsigned char *rv, float x, float y, 
                     unsigned char* img, int w, int h, unsigned char def);
void interpolateN(unsigned char *rv, float x, float y, 
                  unsigned char* img, int width, int height, 
                  unsigned char N, unsigned char channel, unsigned char def);



int initTransformData(TransformData* td, const DSFrameInfo* fi_src, 
                      const DSFrameInfo* fi_dest , const char* modName){
    td->modName = modName;
    
    td->fiSrc = *fi_src;
    td->fiDest = *fi_dest;   
    
    td->src = ds_zalloc(td->fiSrc.framesize); /* FIXME */
    if (td->src == NULL) {
        ds_log_error(td->modName, "tc_malloc failed\n");
        return DS_ERROR;
    }

    td->dest = 0;
    
    /* Options */
    td->maxShift = -1;
    td->maxAngle = -1;

    td->crop = 0;
    td->relative = 1;
    td->invert = 0;
    td->smoothing = 10;
  
    td->rotationThreshhold = 0.25/(180/M_PI);

    td->zoom    = 0;
    td->optZoom = 1;
    td->interpolType = BiLinear;
    td->sharpen = 0.8;  

    td->verbose = 0;
    return DS_OK;
}

int configureTransformData(TransformData* td){
    if (td->maxShift > td->fiDest.width/2) 
        td->maxShift = td->fiDest.width/2;
    if (td->maxShift > td->fiDest.height/2)
        td->maxShift = td->fiDest.height/2;
    
    td->interpolType = DS_MIN(td->interpolType,BiCubic);

    switch(td->interpolType){
      case Zero:     interpolate = &interpolateZero; break;
      case Linear:   interpolate = &interpolateLin; break;
      case BiLinear: interpolate = &interpolateBiLin; break;
      case BiQuad:   interpolate = &interpolateSqr; break;
      case BiCubic:  interpolate = &interpolateBiCub; break;
      default: interpolate = &interpolateBiLin;
    }
    return DS_OK;
}

void cleanupTransformData(TransformData* td){
    if (td->src) {
        ds_free(td->src);
        td->src = NULL;
    }
}

int transformPrepare(TransformData* td, unsigned char* frame_buf){
    td->dest = frame_buf;
    // we first copy the frame to the src and then overwrite the destination
    // with the transformed version.
    memcpy(td->src,  frame_buf, td->fiSrc.framesize);
    return DS_OK;   
}
  

Transform getNextTransform(const TransformData* td, Transformations* trans){
    if (trans->current >= trans->len) {        
        trans->current = trans->len-1;
        if(!trans->warned_end)
            ds_log_warn(td->modName, "not enough transforms found, use last transformation!\n");
        trans->warned_end = 1;                 
    }    
    trans->current++;
    return trans->ts[trans->current-1];
}

void initTransformations(Transformations* trans){
    trans->ts = 0;
    trans->len = 0;
    trans->current = 0;
    trans->warned_end = 0;  
}

void cleanupTransformations(Transformations* trans){
    if (trans->ts) {
        ds_free(trans->ts);
        trans->ts = NULL;        
    }
    trans->len=0;
}

/** interpolateBiLinBorder: bi-linear interpolation function that also works at the border.
    This is used by many other interpolation methods at and outsize the border, see interpolate */
void interpolateBiLinBorder(unsigned char *rv, float x, float y, 
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
static short bicub_kernel(float t, short a0, short a1, short a2, short a3){ 
    return (2*a1 + t*((-a0+a2) + t*((2*a0-5*a1+4*a2-a3) + t*(-a0+3*a1-3*a2+a3) )) ) / 2;
}

/** interpolateBiCub: bi-cubic interpolation function using 4x4 pixel, see interpolate */
void interpolateBiCub(unsigned char *rv, float x, float y, 
                      unsigned char* img, int width, int height, unsigned char def)
{
    // do a simple linear interpolation at the border
    if (x < 1 || x > width-2 || y < 1 || y > height - 2) { 
        interpolateBiLinBorder(rv, x,y,img,width,height,def);    
    } else {
        int x_f = myfloor(x);
        int y_f = myfloor(y);
        float tx = x-x_f;
        short v1 = bicub_kernel(tx,
                                PIX(img, x_f-1, y_f-1, width, height),
                                PIX(img, x_f,   y_f-1, width, height),
                                PIX(img, x_f+1, y_f-1, width, height),
                                PIX(img, x_f+2, y_f-1, width, height));
        short v2 = bicub_kernel(tx,
                                PIX(img, x_f-1, y_f, width, height),
                                PIX(img, x_f,   y_f, width, height),
                                PIX(img, x_f+1, y_f, width, height),
                                PIX(img, x_f+2, y_f, width, height));
        short v3 = bicub_kernel(tx,
                                PIX(img, x_f-1, y_f+1, width, height),
                                PIX(img, x_f,   y_f+1, width, height),
                                PIX(img, x_f+1, y_f+1, width, height),
                                PIX(img, x_f+2, y_f+1, width, height));
        short v4 = bicub_kernel(tx,
                                PIX(img, x_f-1, y_f+2, width, height),
                                PIX(img, x_f,   y_f+2, width, height),
                                PIX(img, x_f+1, y_f+2, width, height),
                                PIX(img, x_f+2, y_f+2, width, height));
        *rv = (unsigned char)bicub_kernel(y-y_f, v1, v2, v3, v4);
    }
}

/** interpolateSqr: bi-quatratic interpolation function, see interpolate */
void interpolateSqr(unsigned char *rv, float x, float y, 
                    unsigned char* img, int width, int height, unsigned char def)
{
    if (x < 0 || x > width-1 || y < 0 || y > height - 1) { 
        interpolateBiLinBorder(rv, x, y, img, width, height, def);    
    } else {
        int x_f = myfloor(x);
        int x_c = x_f+1;
        int y_f = myfloor(y);
        int y_c = y_f+1;
        short v1 = PIX(img, x_c, y_c, width, height);
        short v2 = PIX(img, x_c, y_f, width, height);
        short v3 = PIX(img, x_f, y_c, width, height);
        short v4 = PIX(img, x_f, y_f, width, height);
        float f1 = 1 - sqrt((x_c - x) * (y_c - y));
        float f2 = 1 - sqrt((x_c - x) * (y - y_f));
        float f3 = 1 - sqrt((x - x_f) * (y_c - y));
        float f4 = 1 - sqrt((x - x_f) * (y - y_f));
        float s  = (v1*f1 + v2*f2 + v3*f3+ v4*f4)/(f1 + f2 + f3 + f4);
        *rv = (unsigned char)s;   
    }
}

/** interpolateBiLin: bi-linear interpolation function, see interpolate */
void interpolateBiLin(unsigned char *rv, float x, float y, 
                      unsigned char* img, int width, int height, 
                      unsigned char def)
{
    if (x < 0 || x > width-1 || y < 0 || y > height - 1) { 
        interpolateBiLinBorder(rv, x, y, img, width, height, def);    
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
void interpolateLin(unsigned char *rv, float x, float y, 
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
void interpolateZero(unsigned char *rv, float x, float y, 
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
void interpolateN(unsigned char *rv, float x, float y, 
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
 */
int transformRGB(TransformData* td, Transform t)
{
    int x = 0, y = 0, z = 0;
    unsigned char *D_1, *D_2;
  
    D_1  = td->src;  
    D_2  = td->dest;  
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
                    interpolateN(dest, x_s, y_s, D_1, 
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
int transformYUV(TransformData* td, Transform t)
{
    int x = 0, y = 0;
    unsigned char *Y_1, *Y_2, *Cb_1, *Cb_2, *Cr_1, *Cr_2;
  
    Y_1  = td->src;  
    Y_2  = td->dest;  
    Cb_1 = td->src + td->fiSrc.width * td->fiSrc.height;
    Cb_2 = td->dest + td->fiDest.width * td->fiDest.height;
    Cr_1 = td->src + 5*td->fiSrc.width * td->fiSrc.height/4;
    Cr_2 = td->dest + 5*td->fiDest.width * td->fiDest.height/4;
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
    if (fabs(t.alpha) > td->rotationThreshhold || t.zoom != 0) {
        for (x = 0; x < td->fiDest.width; x++) {
            for (y = 0; y < td->fiDest.height; y++) {
                float x_d1 = (x - c_d_x);
                float y_d1 = (y - c_d_y);
                float x_s  =  zcos_a * x_d1 
                    + zsin_a * y_d1 + c_s_x -t.x;
                float y_s  = -zsin_a * x_d1 
                    + zcos_a * y_d1 + c_s_y -t.y;
                unsigned char* dest = &Y_2[x + y * td->fiDest.width];
                interpolate(dest, x_s, y_s, Y_1,  
                            td->fiSrc.width, td->fiSrc.height,  
                            td->crop ? 16 : *dest); 
            }
        }
     }else { 
        /* no rotation, no zooming, just translation 
         *(also no interpolation, since no size change) 
         */
        int round_tx = myround(t.x);
        int round_ty = myround(t.y);
        for (x = 0; x < td->fiDest.width; x++) {
            for (y = 0; y < td->fiDest.height; y++) {
                short p = PIXEL(Y_1, x - round_tx, y - round_ty, 
                                td->fiSrc.width, td->fiSrc.height, -1);
                if (p == -1) {
                    if (td->crop == 1)
                        Y_2[x + y * td->fiDest.width] = 16;
                } else {
                    Y_2[x + y * td->fiDest.width] = (unsigned char)p;
                }
            }
        }
    }

    /* Color channels */
    int ws2 = td->fiSrc.width/2;
    int wd2 = td->fiDest.width/2;
    int hs2 = td->fiSrc.height/2;
    int hd2 = td->fiDest.height/2;
    if (fabs(t.alpha) > td->rotationThreshhold || t.zoom != 0) {
        for (x = 0; x < wd2; x++) {
            for (y = 0; y < hd2; y++) {
                float x_d1 = x - (c_d_x)/2;
                float y_d1 = y - (c_d_y)/2;
                float x_s  =  zcos_a * x_d1 
                    + zsin_a * y_d1 + (c_s_x -t.x)/2;
                float y_s  = -zsin_a * x_d1 
                    + zcos_a * y_d1 + (c_s_y -t.y)/2;
                unsigned char* dest = &Cr_2[x + y * wd2];
                interpolate(dest, x_s, y_s, Cr_1, ws2, hs2, 
                            td->crop ? 128 : *dest);
                dest = &Cb_2[x + y * wd2];
                interpolate(dest, x_s, y_s, Cb_1, ws2, hs2, 
                            td->crop ? 128 : *dest);      	
            }
        }
    } else { // no rotation, no zoom, no interpolation, just translation 
        int round_tx2 = myround(t.x/2.0);
        int round_ty2 = myround(t.y/2.0);        
        for (x = 0; x < wd2; x++) {
            for (y = 0; y < hd2; y++) {
                short cr = PIXEL(Cr_1, x - round_tx2, y - round_ty2, 
                                 wd2, hd2, -1);
                short cb = PIXEL(Cb_1, x - round_tx2, y - round_ty2, 
                                 wd2, hd2, -1);
                if (cr == -1) {
                    if (td->crop==1) { 
                        Cr_2[x + y * wd2] = 128;
                        Cb_2[x + y * wd2] = 128;
                    }
                } else {
                    Cr_2[x + y * wd2] = (unsigned char)cr;
                    Cb_2[x + y * wd2] = (unsigned char)cb;
                }
            }
        }
    }
    return DS_OK;
}


/** interpolateBiLin with FP: bi-linear interpolation function, see interpolate */
void interpolateFP(unsigned char *rv, fp8 x, fp8 y, 
                   unsigned char* img, int32_t width, int32_t height, 
                   unsigned char def)
{
    int32_t ix_f = fp8ToI(x);
    int32_t iy_f = fp8ToI(y);
    if (ix_f < 0 || ix_f > width-1 || iy_f < 0 || iy_f > height - 1) { 
        *rv=def; // Todo interpolateBiLinBorder(rv, x, y, img, width, height, def);    
    } else {
        int32_t ix_c = ix_f + 1;
        int32_t iy_c = iy_f + 1;        
        short v1 = PIX(img, ix_c, iy_c, width, height);
        short v2 = PIX(img, ix_c, iy_f, width, height);
        short v3 = PIX(img, ix_f, iy_c, width, height);
        short v4 = PIX(img, ix_f, iy_f, width, height);        
        fp8 x_f = iToFp8(ix_f);
        fp8 x_c = iToFp8(ix_c);
        fp8 y_f = iToFp8(iy_f);
        fp8 y_c = iToFp8(iy_c);
        fp16 s  = (v1*(x - x_f)+v3*(x_c - x))*(y - y_f) +  
            (v2*(x - x_f) + v4*(x_c - x))*(y_c - y);
        *rv = fp16ToI(s);
    }
}


/** TEST with fixed-point arithmetic
 * transformYUV: applies current transformation to frame
 *
 * Parameters:
 *         td: private data structure of this filter
 * Return value: 
 *         0 for failture, 1 for success
 * Preconditions:
 *  The frame must be in YUV format
 *
 * Fixed-point format 32 bit integer:
 *  for image coords we use val<<8
 *  for angle and zoom we use val<<16
 *
 */
int transformYUVFP(TransformData* td, Transform t)
{
    int32_t x = 0, y = 0;
    unsigned char *Y_1, *Y_2, *Cb_1, *Cb_2, *Cr_1, *Cr_2;
  
    Y_1  = td->src;  
    Y_2  = td->dest;  
    Cb_1 = td->src + td->fiSrc.width * td->fiSrc.height;
    Cb_2 = td->dest + td->fiDest.width * td->fiDest.height;
    Cr_1 = td->src + 5*td->fiSrc.width * td->fiSrc.height/4;
    Cr_2 = td->dest + 5*td->fiDest.width * td->fiDest.height/4;
    fp8 c_s_x = iToFp8(td->fiSrc.width / 2);
    fp8 c_s_y = iToFp8(td->fiSrc.height / 2);
    int32_t c_d_x = td->fiDest.width / 2;
    int32_t c_d_y = td->fiDest.height / 2;    
    
    float z     = 1.0-t.zoom/100.0;
    fp16 zcos_a = fToFp16(z*cos(-t.alpha)); // scaled cos
    fp16 zsin_a = fToFp16(z*sin(-t.alpha)); // scaled sin
    fp8  c_tx    = c_s_x - fToFp8(t.x);
    fp8  c_ty    = c_s_y - fToFp8(t.y);

    /* for each pixel in the destination image we calc the source
     * coordinate and make an interpolation: 
     *      p_d = c_d + M(p_s - c_s) + t 
     * where p are the points, c the center coordinate, 
     *  _s source and _d destination, 
     *  t the translation, and M the rotation and scaling matrix
     *      p_s = M^{-1}(p_d - c_d - t) + c_s
     */
    /* Luminance channel */
    if (fabs(t.alpha) > td->rotationThreshhold || t.zoom != 0) {
        for (x = 0; x < td->fiDest.width; x++) {
            for (y = 0; y < td->fiDest.height; y++) {
                int32_t x_d1 = (x - c_d_x);
                int32_t y_d1 = (y - c_d_y);
                fp8 x_s  = fp16To8( zcos_a * x_d1 + zsin_a * y_d1) 
                    + c_tx;
                fp8 y_s  = fp16To8(-zsin_a * x_d1 + zcos_a * y_d1) 
                    + c_ty;
                unsigned char* dest = &Y_2[x + y * td->fiDest.width];
                interpolateFP(dest, x_s, y_s, Y_1, 
                              td->fiSrc.width, td->fiSrc.height, 
                              td->crop ? 16 : *dest);
            }
        }
     }else { 
        /* no rotation, no zooming, just translation 
         *(also no interpolation, since no size change) 
         */
        // TODO
    }

    /* Color channels */
    int32_t ws2 = td->fiSrc.width/2;
    int32_t wd2 = td->fiDest.width/2;
    int32_t hs2 = td->fiSrc.height/2;
    int32_t hd2 = td->fiDest.height/2;
    fp8 c_tx2   = c_tx/2;
    fp8 c_ty2   = c_ty/2;

    if (fabs(t.alpha) > td->rotationThreshhold || t.zoom != 0) {
        for (x = 0; x < wd2; x++) {
            for (y = 0; y < hd2; y++) {
                int32_t x_d1 = x - (c_d_x)/2;
                int32_t y_d1 = y - (c_d_y)/2;
                fp8 x_s  =  fp16To8(zcos_a * x_d1 + zsin_a * y_d1) 
                    + c_tx2;
                fp8 y_s  = fp16To8(-zsin_a * x_d1 + zcos_a * y_d1) 
                    + c_ty2; 
                unsigned char* dest = &Cr_2[x + y * wd2];
                interpolateFP(dest, x_s, y_s, Cr_1, ws2, hs2, 
                            td->crop ? 128 : *dest);
                dest = &Cb_2[x + y * wd2];
                interpolateFP(dest, x_s, y_s, Cb_1, ws2, hs2, 
                            td->crop ? 128 : *dest);      	
            }
        }
    } else { // no rotation, no zoom, no interpolation, just translation 
        // TODO
    }
    return DS_OK;
}


/** 
 * read_transforms: read transforms file
 *  The format is as follows:
 *   Lines with # at the beginning are comments and will be ignored
 *   Data lines have 5 columns seperated by space or tab containing
 *   time, x-translation, y-translation, alpha-rotation, extra
 *   where time and extra are integers 
 *   and the latter is unused at the moment
 *
 * Parameters:
 *         f:  file description
 *         trans: place to store the transforms
 * Return value: 
 *         number of transforms read
 * Preconditions: f is opened
 */
int readTransforms(const TransformData* td, FILE* f , Transformations* trans)
{
    char l[1024];
    int s = 0;
    int i = 0;
    int ti; // time (ignored)
    Transform t;
    
    while (fgets(l, sizeof(l), f)) {
        if (l[0] == '#')
            continue;    //  ignore comments
        if (strlen(l) == 0)
            continue; //  ignore empty lines
        // try new format
        if (sscanf(l, "%i %lf %lf %lf %lf %i", &ti, &t.x, &t.y, &t.alpha, 
                   &t.zoom, &t.extra) != 6) {
            if (sscanf(l, "%i %lf %lf %lf %i", &ti, &t.x, &t.y, &t.alpha, 
                       &t.extra) != 5) {                
                ds_log_error(td->modName, "Cannot parse line: %s", l);
                return 0;
            }
            t.zoom=0;
        }
    
        if (i>=s) { // resize transform array
            if (s == 0)
                s = 256;
            else
                s*=2;
            /* ds_log_info(td->modName, "resize: %i\n", s); */
            trans->ts = ds_realloc(trans->ts, sizeof(Transform)* s);
            if (!trans->ts) {
                ds_log_error(td->modName, "Cannot allocate memory"
                                       " for transformations: %i\n", s);
                return 0;
            }
        }
        trans->ts[i] = t;
        i++;
    }
    trans->len = i;

    return i;
}

/**
 * preprocess_transforms: does smoothing, relative to absolute conversion,
 *  and cropping of too large transforms.
 *  This is actually the core algorithm for canceling the jiggle in the 
 *  movie. We perform a low-pass filter in terms of transformation size.
 *  This enables still camera movement, but in a smooth fasion.
 *
 * Parameters:
 *            td: tranform private data structure
 * Return value:
 *     1 for success and 0 for failture
 * Preconditions:
 *     None
 * Side effects:
 *     td->trans will be modified
 */
int preprocessTransforms(TransformData* td, Transformations* trans)
{
    Transform* ts = trans->ts;
    int i;

    if (trans->len < 1)
        return 0;
    if (td->verbose & DS_DEBUG) {
        ds_log_msg(td->modName, "Preprocess transforms:");
    }
    if (td->smoothing>0) {
        /* smoothing */
        Transform* ts2 = ds_malloc(sizeof(Transform) * trans->len);
        memcpy(ts2, ts, sizeof(Transform) * trans->len);

        /*  we will do a sliding average with minimal update
         *   \hat x_{n/2} = x_1+x_2 + .. + x_n
         *   \hat x_{n/2+1} = x_2+x_3 + .. + x_{n+1} = x_{n/2} - x_1 + x_{n+1}
         *   avg = \hat x / n
         */
        int s = td->smoothing * 2 + 1;
        Transform null = null_transform();
        /* avg is the average over [-smoothing, smoothing] transforms 
           around the current point */
        Transform avg;
        /* avg2 is a sliding average over the filtered signal! (only to past) 
         *  with smoothing * 10 horizont to kill offsets */
        Transform avg2 = null_transform();
        double tau = 1.0/(3 * s);
        /* initialise sliding sum with hypothetic sum centered around
         * -1st element. We have two choices:
         * a) assume the camera is not moving at the beginning 
         * b) assume that the camera moves and we use the first transforms
         */
        Transform s_sum = null; 
        for (i = 0; i < td->smoothing; i++){
            s_sum = add_transforms(&s_sum, i < trans->len ? &ts2[i]:&null);
        }
        mult_transform(&s_sum, 2); // choice b (comment out for choice a)

        for (i = 0; i < trans->len; i++) {
            Transform* old = ((i - td->smoothing - 1) < 0) 
                ? &null : &ts2[(i - td->smoothing - 1)];
            Transform* new = ((i + td->smoothing) >= trans->len) 
                ? &null : &ts2[(i + td->smoothing)];
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

            if (td->verbose & DS_DEBUG) {
                ds_log_msg(td->modName, 
                           "s_sum: %5lf %5lf %5lf, ts: %5lf, %5lf, %5lf\n", 
                           s_sum.x, s_sum.y, s_sum.alpha, 
                           ts[i].x, ts[i].y, ts[i].alpha);
                ds_log_msg(td->modName, 
                           "  avg: %5lf, %5lf, %5lf avg2: %5lf, %5lf, %5lf", 
                           avg.x, avg.y, avg.alpha, 
                           avg2.x, avg2.y, avg2.alpha);      
            }
        }
        ds_free(ts2);
    }
  
  
    /*  invert? */
    if (td->invert) {
        for (i = 0; i < trans->len; i++) {
            ts[i] = mult_transform(&ts[i], -1);      
        }
    }
  
    /* relative to absolute */
    if (td->relative) {
        Transform t = ts[0];
        for (i = 1; i < trans->len; i++) {
            if (td->verbose  & DS_DEBUG) {
                ds_log_msg(td->modName, "shift: %5lf   %5lf   %lf \n", 
                           t.x, t.y, t.alpha *180/M_PI);
            }
            ts[i] = add_transforms(&ts[i], &t); 
            t = ts[i];
        }
    }
    /* crop at maximal shift */
    if (td->maxShift != -1)
        for (i = 0; i < trans->len; i++) {
            ts[i].x     = DS_CLAMP(ts[i].x, -td->maxShift, td->maxShift);
            ts[i].y     = DS_CLAMP(ts[i].y, -td->maxShift, td->maxShift);
        }
    if (td->maxAngle != - 1.0)
        for (i = 0; i < trans->len; i++)
            ts[i].alpha = DS_CLAMP(ts[i].alpha, -td->maxAngle, td->maxAngle);

    /* Calc optimal zoom 
     *  cheap algo is to only consider transformations
     *  uses cleaned max and min 
     */
    if (td->optZoom != 0 && trans->len > 1){    
        Transform min_t, max_t;
        cleanmaxmin_xy_transform(ts, trans->len, 10, &min_t, &max_t); 
        // the zoom value only for x
        double zx = 2*DS_MAX(max_t.x,fabs(min_t.x))/td->fiSrc.width;
        // the zoom value only for y
        double zy = 2*DS_MAX(max_t.y,fabs(min_t.y))/td->fiSrc.height;
        td->zoom += 100* DS_MAX(zx,zy); // use maximum
        ds_log_info(td->modName, "Final zoom: %lf\n", td->zoom);
    }
        
    /* apply global zoom */
    if (td->zoom != 0){
        for (i = 0; i < trans->len; i++)
            ts[i].zoom += td->zoom;       
    }

    return DS_OK;
}


/*
  TODO:
  - check for optimization, e.g. mmx stuff
*/

/*
 * Local variables:
 *   c-file-style: "stroustrup"
 *   c-file-offsets: ((case-label . *) (statement-case-intro . *))
 *   indent-tabs-mode: nil
 * End:
 *
 * vim: expandtab shiftwidth=4:
 */
