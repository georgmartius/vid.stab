/*
 *  motiondetect.h
 *
 *  Created on: Feb 21, 2011
 *  Copyright (C) Georg Martius - June 2007
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

#ifndef MOTIONDETECT_H
#define MOTIONDETECT_H

#include <stddef.h>
#include <stdlib.h>

#include "transformtype.h"
#include "deshakedefines.h"
#include "dslist.h"
#include "frameinfo.h"

/** stores x y and size of a measurement field */
typedef struct _field {
  int x;     // middle position x
  int y;     // middle position y
  int size;  // size of field
} Field;


/** data structure for motion detection part of deshaking*/
typedef struct motiondetect {
  DSFrameInfo fi;

  unsigned char* curr; // current frame buffer (only pointer)
  unsigned char* currcopy; // copy of the current frame needed for drawing
  unsigned char* prev; // frame buffer for last frame (copied)
  short hasSeenOneFrame; // true if we have a valid previous frame

  const char* modName;

  /* list of transforms*/
  DSList* transs;

  Field* fields;

  /* Options */
  /* maximum number of pixels we expect the shift of subsequent frames */
  int maxShift;
  int stepSize; // stepsize of field transformation detection
  int allowMax; // 1 if maximal shift is allowed
  int algo;     // algorithm to use
  int fieldNum;  // number of measurement fields
  int maxFields;  // maximum number of fields used (selected by contrast)
  int fieldSize; // size    = min(md->width, md->height)/10;
  int fieldRows; // number of rows
  /* if 1 and 2 then the fields and transforms are shown in the frames */
  int show;
  /* measurement fields with lower contrast are discarded */
  double contrastThreshold;
  /* maximal difference in angles of fields */
  double maxAngleVariation;
  /* meta parameter for maxshift and fieldsize between 1 and 15 */
  int shakiness;
  int accuracy;   // meta parameter for number of fields between 1 and 10

  int initialized; // 1 if initialized and 2 if configured

  int t;
} MotionDetect;

/* type for a function that calculates the transformation of a certain field
 */
typedef Transform (*calcFieldTransFunc)(MotionDetect*, const Field*, int);

/* type for a function that calculates the contrast of a certain field
 */
typedef double (*contrastSubImgFunc)(MotionDetect*, const Field*);

/** initialized the MotionDetect structure and allocates memory
 *  for the frames and stuff
 *  @return DS_OK on success otherwise DS_ERROR
 */
int initMotionDetect(MotionDetect* md, const DSFrameInfo* fi, const char* modName);

/** configures MotionDetect structure and checks ranges, initializes fields and so on.
 *  @return DS_OK on success otherwise DS_ERROR
 */
int configureMotionDetect(MotionDetect* md);

/**
 *  Performs a motion detection step
 *  and adds a transform to the list of transforms
 *  Only the new current frame is given. The last frame
 *  is stored internally
 * */
int motionDetection(MotionDetect* md, unsigned char *frame);

/** Deletes internal data structures.
 * In order to use the MotionDetect again, you have to call initMotionDetect
 */
void cleanupMotionDetection(MotionDetect* md);

int initFields(MotionDetect* md);
int compareImg(unsigned char* I1, unsigned char* I2,
               int width, int height,  int bytesPerPixel, int d_x, int d_y);
int compareSubImg(unsigned char* const I1, unsigned char* const I2,
                  const Field* field,
                  int width, int height, int bytesPerPixel,int d_x,int d_y);
double contrastSubImgYUV(MotionDetect* md, const Field* field);
double contrastSubImgRGB(MotionDetect* md, const Field* field);
double contrastSubImg_Michelson(unsigned char* const I, const Field* field,
                                int width, int height, int bytesPerPixel);
double contrastSubImg(unsigned char* const I, const Field* field,
                      int width, int height);
int cmp_contrast_idx(const void *ci1, const void* ci2);
DSList* selectfields(MotionDetect* md, contrastSubImgFunc contrastfunc);

Transform calcShiftRGBSimple(MotionDetect* md);
Transform calcShiftYUVSimple(MotionDetect* md);
double calcAngle(MotionDetect* md, Field* field, Transform* t,
                 int center_x, int center_y);
Transform calcFieldTransYUV(MotionDetect* md, const Field* field,
                            int fieldnum);
Transform calcFieldTransRGB(MotionDetect* md, const Field* field,
                            int fieldnum);
Transform calcTransFields(MotionDetect* md, calcFieldTransFunc fieldfunc,
                          contrastSubImgFunc contrastfunc);


void drawFieldScanArea(MotionDetect* md, const Field* field, const Transform* t);
void drawField(MotionDetect* md, const Field* field, const Transform* t);
void drawFieldTrans(MotionDetect* md, const Field* field, const Transform* t);
void drawBox(unsigned char* I, int width, int height, int bytesPerPixel,
             int x, int y, int sizex, int sizey, unsigned char color);
void addTrans(MotionDetect* md, Transform sl);


#ifdef TESTING
/// Functions for testing against optimized versions
int compareSubImg_C(unsigned char* const I1, unsigned char* const I2,
                    const Field* field, int width, int height, int bytesPerPixel, 
                    int d_x, int d_y);

double contrastSubImg_C(unsigned char* const I, const Field* field,
                      int width, int height);

#endif // TESTING

#endif  /* MOTIONDETECT_H */

/*
 * Local variables:
 *   c-file-style: "stroustrup"
 *   c-file-offsets: ((case-label . *) (statement-case-intro . *))
 *   indent-tabs-mode: nil
 * End:
 *
 * vim: expandtab shiftwidth=4:
 */
