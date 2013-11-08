/*
 *  motiondetect_internal.h
 *
 *  Copyright (C) Georg Martius - February 2011
 *   georg dot martius at web dot de
 *  Copyright (C) Alexey Osipov - Jule 2011
 *   simba at lerlan dot ru
 *   speed optimizations (threshold, spiral, SSE, asm)
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

#ifndef MOTIONDETECT_INTERNAL_H
#define MOTIONDETECT_INTERNAL_H

#include "motiondetect.h"

/* type for a function that calculates the transformation of a certain field
 */
typedef LocalMotion (*calcFieldTransFunc)(VSMotionDetect*, VSMotionDetectFields*,
                                          const Field*, int);

/* type for a function that calculates the contrast of a certain field
 */
typedef double (*contrastSubImgFunc)(VSMotionDetect*, const Field*);


int initFields(VSMotionDetect* md, VSMotionDetectFields* fs,
               int fieldSize, int maxShift, int stepSize, short border, int spacing);

double contrastSubImgPlanar(VSMotionDetect* md, const Field* field);
double contrastSubImgPacked(VSMotionDetect* md, const Field* field);
double contrastSubImg(unsigned char* const I, const Field* field,
                      int width, int height, int bytesPerPixel);


int cmp_contrast_idx(const void *ci1, const void* ci2);
VSVector selectfields(VSMotionDetect* md, VSMotionDetectFields* fields,
                      contrastSubImgFunc contrastfunc);

LocalMotion calcFieldTransPlanar(VSMotionDetect* md, VSMotionDetectFields* fields,
                                 const Field* field, int fieldnum);
LocalMotion calcFieldTransPacked(VSMotionDetect* md, VSMotionDetectFields* fields,
                                 const Field* field, int fieldnum);
LocalMotions calcTransFields(VSMotionDetect* md, VSMotionDetectFields* fields,
                             calcFieldTransFunc fieldfunc,
                             contrastSubImgFunc contrastfunc);


void drawFieldScanArea(VSMotionDetect* md, const LocalMotion* motion, int maxShift);
void drawField(VSMotionDetect* md, const LocalMotion* motion);
void drawFieldTrans(VSMotionDetect* md, const LocalMotion* motion);
void drawBox(unsigned char* I, int width, int height, int bytesPerPixel,
             int x, int y, int sizex, int sizey, unsigned char color);
void drawRectangle(unsigned char* I, int width, int height, int bytesPerPixel,
                   int x, int y, int sizex, int sizey, unsigned char color);

void drawLine(unsigned char* I, int width, int height, int bytesPerPixel,
              Vec* a, Vec* b, int thickness, unsigned char color);

unsigned int compareSubImg_thr(unsigned char* const I1, unsigned char* const I2,
                               const Field* field, int width1, int width2, int height,
                               int bytesPerPixel,
                               int d_x, int d_y, unsigned int threshold);

#endif  /* MOTIONDETECT_INTERNAL_H */

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
