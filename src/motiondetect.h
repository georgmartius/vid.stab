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

#ifndef MOTIONDETECT_H
#define MOTIONDETECT_H

#include <stddef.h>
#include <stdlib.h>

#include "transformtype.h"
#include "vidstabdefines.h"
#include "vsvector.h"
#include "frameinfo.h"
#include "vidstab_api.h"

#define ASCII_SERIALIZATION_MODE 1
#define BINARY_SERIALIZATION_MODE 2

typedef struct VS_API _vsmotiondetectconfig {
  /* meta parameter for maxshift and fieldsize between 1 and 15 */
  int         shakiness;
  int         seek_range;       // user-selected maximum range in pixels for finding matches
  int         accuracy;         // meta parameter for number of fields between 1 and 10
  int         stepSize;         // stepsize of field transformation detection
  int         algo;             // deprecated
  int         virtualTripod;
  /* if 1 and 2 then the fields and transforms are shown in the frames */
  int         show;
  /* measurement fields with lower contrast are discarded */
  double      contrastThreshold;
  const char* modName;          // module name (used for logging)
  int         numThreads;       // number of threads to use (automatically set if 0)
} VSMotionDetectConfig;

/** structure for motion detection fields */
typedef struct VS_API _vsmotiondetectfields {
  /* maximum number of pixels we expect the shift of subsequent frames */
  int maxShift;
  int stepSize;                 // stepsize for detection
  int fieldNum;                 // number of measurement fields
  int maxFields;                // maximum number of fields used (selected by contrast)
  double contrastThreshold;     // fields with lower contrast are discarded
  int fieldSize;                // size = min(md->width, md->height)/10;
  int fieldRows;                // number of rows
  Field* fields;                // measurement fields
  short useOffset;              // if true then the offset us used
  VSTransform offset;           // offset for detection (e.g. known from coarse scan)
} VSMotionDetectFields;

/** data structure for motion detection part of deshaking*/
typedef struct VS_API _vsmotiondetect {
  VSFrameInfo fi;

  VSMotionDetectConfig conf;

  VSMotionDetectFields fieldscoarse;
  VSMotionDetectFields fieldsfine;

  VSFrame curr;                 // blurred version of current frame buffer
  VSFrame currorig;             // current frame buffer (original) (only pointer)
  VSFrame currtmp;              // temporary buffer for blurring
  VSFrame prev;                 // frame buffer for last frame (copied)
  short hasSeenOneFrame;        // true if we have a valid previous frame
  int initialized;              // 1 if initialized and 2 if configured
  int serializationMode;        // 1 if ascii and 2 if binary

  int frameNum;
} VSMotionDetect;

static const char vs_motiondetect_help[] = ""
    "Overview:\n"
    "    Generates a file with relative transform information\n"
    "     (translation, rotation) about subsequent frames."
    " See also transform.\n"
    "Options\n"
    "    'fileformat'  the type of file format used to write the transforms\n"
    "                  1: ascii (human readable) file format 2: binary (smaller) file format\n"
    "    'result'      path to the file used to write the transforms\n"
    "                  (def:inputfile.stab)\n"
    "    'shakiness'   how shaky is the video and how quick is the camera?\n"
    "                  1: little (fast) 10: very strong/quick (slow) (def: 5)\n"
    "    'seek_range'  How far (in pixels) can we search for local matches?\n"
    "                  (def: 1/7 of the smallest image dimension )\n"
    "    'accuracy'    accuracy of detection process (>=shakiness)\n"
    "                  1: low (fast) 15: high (slow) (def: 9)\n"
    "    'stepsize'    stepsize of search process, region around minimum \n"
    "                  is scanned with 1 pixel resolution (def: 6)\n"
    "    'mincontrast' below this contrast a field is discarded (0-1) (def: 0.3)\n"
    "    'tripod'      virtual tripod mode (if >0): motion is compared to a \n"
    "                  reference frame (frame # is the value) (def: 0)\n"
    "    'show'        0: draw nothing (def); 1,2: show fields and transforms\n"
    "                  in the resulting frames. Consider the 'preview' filter\n"
    "    'help'        print this help message\n";


/** returns the default config
 */
VS_API VSMotionDetectConfig vsMotionDetectGetDefaultConfig(const char* modName);

/** initialized the VSMotionDetect structure and allocates memory
 *  for the frames and stuff
 *  @return VS_OK on success otherwise VS_ERROR
 */
VS_API int vsMotionDetectInit(VSMotionDetect* md, const VSMotionDetectConfig* conf,
                       const VSFrameInfo* fi);

/**
 *  Performs a motion detection step
 *  Only the new current frame is given. The last frame
 *  is stored internally
 *  @param motions: calculated local motions. (must be deleted manually)
 * */
VS_API int vsMotionDetection(VSMotionDetect* md, LocalMotions* motions, VSFrame *frame);

/** Deletes internal data structures.
 * In order to use the VSMotionDetect again, you have to call vsMotionDetectInit
 */
VS_API void vsMotionDetectionCleanup(VSMotionDetect* md);

/// returns the current config
VS_API void vsMotionDetectGetConfig(VSMotionDetectConfig* conf, const VSMotionDetect* md);

/// returns the frame info
VS_API const VSFrameInfo* vsMotionDetectGetFrameInfo(const VSMotionDetect* md);

#endif  /* MOTIONDETECT_H */

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
