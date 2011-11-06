/*
 *  transformtype.h
 *
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
#ifndef __TRANSFORM_H
#define __TRANSFORM_H

#include <math.h>
#include <libgen.h>
#include "transformtype.h"
#include "frameinfo.h"
#include "deshakedefines.h"

typedef struct transformations {
    Transform* ts; // array of transformations
    int current;   // index to current transformation
    int len;       // length of trans array
    short warned_end; // whether we warned that there is no transform left
} Transformations;

typedef struct slidingavgtrans {
    Transform avg; // average transformation
    Transform accum; // accumulator for relative to absolute conversion
    double zoomavg;     // average zoom value
    short initialized; // whether it was initialized or not
} SlidingAvgTrans;


/// interpolation types
typedef enum { Zero, Linear, BiLinear, BiCubic} InterpolType;
/// name of the interpolation type
extern const char* interpolTypes[5];

typedef struct {
    DSFrameInfo fiSrc;
    DSFrameInfo fiDest;

    unsigned char* src;  // copy of the current frame buffer
    unsigned char* dest; // pointer to destination buffer or
                         // the current frame buffer (depending on crop)
    unsigned char* framebuf; // pointer to the current frame buffer

    const char* modName;
 
    /* Options */
    int maxShift;        // maximum number of pixels we will shift
    double maxAngle;     // maximum angle in rad

    /* whether to consider transforms as relative (to previous frame) 
     * or absolute transforms  
     */
    int relative;  
    /* number of frames (forward and backward) 
     * to use for smoothing transforms */
    int smoothing;  
    int crop;       // 1: black bg, 0: keep border from last frame(s)
    int invert;     // 1: invert transforms, 0: nothing
    /* constants */
    /* threshhold below which no rotation is performed */
    double rotationThreshhold; 
    double zoom;      // percentage to zoom: 0->no zooming 10:zoom in 10%
    int optZoom;      // 1: determine optimal zoom, 0: nothing
    InterpolType interpolType; // type of interpolation: 0->Zero,1->Lin,2->BiLin,3->Sqr
    double sharpen;   // amount of sharpening

    int verbose;     // level of logging

    int initialized; // 1 if initialized and 2 if configured
} TransformData;


static const char transform_help[] = ""
    "Overview\n"
    "    Reads a file with transform information for each frame\n"
    "     and applies them. See also filter stabilize.\n" 
    "Options\n"
    "    'input'     path to the file used to read the transforms\n"
    "                (def: inputfile.stab)\n"
    "    'smoothing' number of frames*2 + 1 used for lowpass filtering \n"
    "                used for stabilizing (def: 10)\n"
    "    'maxshift'  maximal number of pixels to translate image\n"
    "                (def: -1 no limit)\n"
    "    'maxangle'  maximal angle in rad to rotate image (def: -1 no limit)\n"
    "    'crop'      0: keep border (def), 1: black background\n"
    "    'invert'    1: invert transforms(def: 0)\n"
    "    'relative'  consider transforms as 0: absolute, 1: relative (def)\n"
    "    'zoom'      percentage to zoom >0: zoom in, <0 zoom out (def: 0)\n"
    "    'optzoom'   0: nothing, 1: determine optimal zoom (def)\n"
    "                i.e. no (or only little) border should be visible.\n"
    "                Note that the value given at 'zoom' is added to the \n"
    "                here calculated one\n"
    "    'interpol'  type of interpolation: 0: no interpolation, \n"
    "                1: linear (horizontal), 2: bi-linear (def), \n"
    "                3: bi-cubic\n"
    "    'sharpen'   amount of sharpening: 0: no sharpening (def: 0.8)\n"
    "                uses filter unsharp with 5x5 matrix\n"
    "    'help'      print this help message\n";

/** initialized the TransformData structure and allocates memory
 *  for the frames and stuff
 *  @return DS_OK on success otherwise DS_ERROR
 */
int initTransformData(TransformData* td, const DSFrameInfo* fi_src, 
                      const DSFrameInfo* fi_dest , const char* modName);

/** configures TransformData structure and checks ranges, initializes fields and so on.
 *  @return DS_OK on success otherwise DS_ERROR
 */
int configureTransformData(TransformData* td);

/** Deletes internal data structures.
 * In order to use the TransformData again, you have to call initTransformData
 */
void cleanupTransformData(TransformData* td);


/// initializes Transformations structure
void initTransformations(Transformations* trans);
/// deletes Transformations internal memory
void cleanupTransformations(Transformations* trans);

/// return next Transform and increases internal counter
Transform getNextTransform(const TransformData* td, Transformations* trans);

/// read the transformations from the given file
int readTransforms(const TransformData* td, FILE* f , Transformations* trans);

/** preprocesses the list of transforms all at once. Here the deshaking is calculated!
 */
int preprocessTransforms(TransformData* td, Transformations* trans);


/**
 * lowPassTransforms: single step smoothing of transforms, using only the past.
 *  see also preprocessTransforms. */
Transform lowPassTransforms(TransformData* td, SlidingAvgTrans* mem, 
                            const Transform* trans);

/** call this function to prepare for a next transformation (transformRGB/transformYUV)
    and supply the frame buffer for the frame to write
 */
int transformPrepare(TransformData* td, unsigned char* frame_buf);

/** call this function to finish the transformation of a frame (transformRGB/transformYUV)
 */
int transformFinish(TransformData* td);


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
