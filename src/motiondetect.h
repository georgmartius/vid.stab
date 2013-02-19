/*
 *  motiondetect.h
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

#ifndef MOTIONDETECT_H
#define MOTIONDETECT_H

#include <stddef.h>
#include <stdlib.h>

#include "transformtype.h"
#include "deshakedefines.h"
#include "dsvector.h"
#include "frameinfo.h"

#define USE_SPIRAL_FIELD_CALC


/** data structure for motion detection part of deshaking*/
typedef struct motiondetect {
  DSFrameInfo fi;

  DSFrame curr;     // blurred version of current frame buffer
  DSFrame currorig; // current frame buffer (original) (only pointer)
  DSFrame currtmp;  // temporary buffer for blurring
  DSFrame prev;     // frame buffer for last frame (copied)
  short hasSeenOneFrame;   // true if we have a valid previous frame

  const char* modName;

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
  /* if >0 then all the frames are compared with the given frame (1 for first) */
  int virtualTripod;
  /* if 1 and 2 then the fields and transforms are shown in the frames */
  int show;
  /* measurement fields with lower contrast are discarded */
  double contrastThreshold;
  /* meta parameter for maxshift and fieldsize between 1 and 15 */
  int shakiness;
  int accuracy;   // meta parameter for number of fields between 1 and 10

  int initialized; // 1 if initialized and 2 if configured

  int frameNum;
} MotionDetect;

/* type for a function that calculates the transformation of a certain field
 */
typedef LocalMotion (*calcFieldTransFunc)(MotionDetect*, const Field*, int);

/* type for a function that calculates the contrast of a certain field
 */
typedef double (*contrastSubImgFunc)(MotionDetect*, const Field*);


static const char motiondetect_help[] = ""
    "Overview:\n"
    "    Generates a file with relative transform information\n"
    "     (translation, rotation) about subsequent frames."
    " See also transform.\n"
    "Options\n"
    "    'result'      path to the file used to write the transforms\n"
    "                  (def:inputfile.stab)\n"
    "    'shakiness'   how shaky is the video and how quick is the camera?\n"
    "                  1: little (fast) 10: very strong/quick (slow) (def: 4)\n"
    "    'accuracy'    accuracy of detection process (>=shakiness)\n"
    "                  1: low (fast) 15: high (slow) (def: 4)\n"
    "    'stepsize'    stepsize of search process, region around minimum \n"
    "                  is scanned with 1 pixel resolution (def: 6)\n"
    "    'algo'        0: brute force (translation only);\n"
    "                  1: small measurement fields (def)\n"
    "    'mincontrast' below this contrast a field is discarded (0-1) (def: 0.3)\n"
    "    'tripod'      virtual tripod mode (if >0): motion is compared to a \n"
    "                  reference frame (frame # is the value) (def: 0)\n"
    "    'show'        0: draw nothing (def); 1,2: show fields and transforms\n"
    "                  in the resulting frames. Consider the 'preview' filter\n"
    "    'help'        print this help message\n";


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
 *  Only the new current frame is given. The last frame
 *  is stored internally
 *  @param motions: calculated local motions. (must be deleted manually)
 * */
int motionDetection(MotionDetect* md, LocalMotions* motions, DSFrame *frame);

/** Deletes internal data structures.
 * In order to use the MotionDetect again, you have to call initMotionDetect
 */
void cleanupMotionDetection(MotionDetect* md);


int initFields(MotionDetect* md);
unsigned int compareImg(unsigned char* I1, unsigned char* I2, int width, int height,
                        int bytesPerPixel, int strive1, int strive2, int d_x, int d_y);

double contrastSubImgYUV(MotionDetect* md, const Field* field);
double contrastSubImgRGB(MotionDetect* md, const Field* field);
double contrastSubImg(unsigned char* const I, const Field* field,
                      int width, int height, int bytesPerPixel);


int cmp_contrast_idx(const void *ci1, const void* ci2);
DSVector selectfields(MotionDetect* md, contrastSubImgFunc contrastfunc);

LocalMotions calcShiftRGBSimple(MotionDetect* md);
LocalMotions calcShiftYUVSimple(MotionDetect* md);

LocalMotion calcFieldTransYUV(MotionDetect* md, const Field* field,
                            int fieldnum);
LocalMotion calcFieldTransRGB(MotionDetect* md, const Field* field,
                            int fieldnum);
LocalMotions calcTransFields(MotionDetect* md, calcFieldTransFunc fieldfunc,
                             contrastSubImgFunc contrastfunc);


void drawFieldScanArea(MotionDetect* md, const LocalMotion* motion);
void drawField(MotionDetect* md, const LocalMotion* motion);
void drawFieldTrans(MotionDetect* md, const LocalMotion* motion);
void drawBox(unsigned char* I, int width, int height, int bytesPerPixel,
             int x, int y, int sizex, int sizey, unsigned char color);


unsigned int compareSubImg_thr(unsigned char* const I1, unsigned char* const I2,
                               const Field* field, int width1, int width2, int height,
                               int bytesPerPixel,
                               int d_x, int d_y, unsigned int threshold);

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
