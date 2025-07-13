/*
 *  transform.h
 *
 *  Copyright (C) Georg Martius - June 2007 - 2011
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
 */
#ifndef __TRANSFORM_H
#define __TRANSFORM_H

#include <math.h>
#ifndef _MSC_VER
    #include <libgen.h>
#endif
#include <stdint.h>
#include "transformtype.h"
#include "frameinfo.h"
#include "vidstabdefines.h"
#include "vidstab_api.h"
#ifdef TESTING
#include "transformfloat.h"
#endif


typedef struct VS_API _vstransformations {
    VSTransform* ts; // array of transformations
    int current;   // index to current transformation
    int len;       // length of trans array
    short warned_end; // whether we warned that there is no transform left
} VSTransformations;

typedef struct VS_API _vsslidingavgtrans {
    VSTransform avg; // average transformation
    VSTransform accum; // accumulator for relative to absolute conversion
    double zoomavg;     // average zoom value
    short initialized; // whether it was initialized or not
} VSSlidingAvgTrans;


/// interpolation types
typedef enum { VS_Zero, VS_Linear, VS_BiLinear, VS_BiCubic, VS_NBInterPolTypes} VSInterpolType;

/// returns a name for the interpolation type
VS_API const char* getInterpolationTypeName(VSInterpolType type);

typedef enum { VSKeepBorder = 0, VSCropBorder } VSBorderType;
typedef enum { VSOptimalL1 = 0, VSGaussian, VSAvg } VSCamPathAlgo;

/**
 * interpolate: general interpolation function pointer for one channel image data
 *              for fixed point numbers/calculations
 * Parameters:
 *             rv: destination pixel (call by reference)
 *            x,y: the source coordinates in the image img. Note this
 *                 are real-value coordinates (in fixed point format 24.8),
 *                 that's why we interpolate
 *            img: source image
 *   width,height: dimension of image
 *            def: default value if coordinates are out of range
 * Return value:  None
 */
typedef void (*vsInterpolateFun)(uint8_t *rv, int32_t x, int32_t y,
                                 const uint8_t *img, int linesize,
                                 int width, int height, uint8_t def);

typedef struct VS_API _VSTransformConfig {

    /* whether to consider transforms as relative (to previous frame)
     * or absolute transforms
     */
    int            relative;
    /* number of frames (forward and backward)
     * to use for smoothing transforms */
    int            smoothing;
    VSBorderType   crop;        // 1: black bg, 0: keep border from last frame(s)
    int            invert;      // 1: invert transforms, 0: nothing
    double         zoom;        // percentage to zoom: 0->no zooming 10:zoom in 10%
    int            optZoom;     // 2: optimal adaptive zoom 1: optimal static zoom, 0: nothing
    double         zoomSpeed;   // for adaptive zoom: zoom per frame in percent
    VSInterpolType interpolType; // type of interpolation: 0->Zero,1->Lin,2->BiLin,3->Sqr
    int            maxShift;    // maximum number of pixels we will shift
    double         maxAngle;    // maximum angle in rad
    const char*    modName;     // module name (used for logging)
    int            verbose;     // level of logging
    // if 1 then the simple but fast method to termine the global motion is used
    int            simpleMotionCalculation;
    int            storeTransforms; // stores calculated transforms to file
    int            smoothZoom;   // if 1 the zooming is also smoothed. Typically not recommended.
    VSCamPathAlgo  camPathAlgo;  // algorithm to use for camera path optimization
} VSTransformConfig;

typedef struct VS_API _VSTransformData {
    VSFrameInfo fiSrc;
    VSFrameInfo fiDest;

    VSFrame src;         // copy of the current frame buffer
    VSFrame destbuf;     // pointer to an additional buffer or
                         // to the destination buffer (depending on crop)
    VSFrame dest;        // pointer to the destination buffer

    short srcMalloced;   // 1 if the source buffer was internally malloced

    vsInterpolateFun interpolate; // pointer to interpolation function
#ifdef TESTING
    _FLT(vsInterpolateFun) _FLT(interpolate);
#endif

    /* Options */
    VSTransformConfig conf;

    int initialized; // 1 if initialized and 2 if configured
} VSTransformData;


static const char vs_transform_help[] = ""
    "Overview\n"
    "    Reads a file with transform information for each frame\n"
    "     and applies them. See also filter stabilize.\n"
    "Options\n"
    "    'input'     path to the file used to read the transforms\n"
    "                (def: inputfile.trf)\n"
    "    'smoothing' number of frames*2 + 1 used for lowpass filtering \n"
    "                used for stabilizing (def: 10)\n"
    "    'maxshift'  maximal number of pixels to translate image\n"
    "                (def: -1 no limit)\n"
    "    'maxangle'  maximal angle in rad to rotate image (def: -1 no limit)\n"
    "    'crop'      0: keep border (def), 1: black background\n"
    "    'invert'    1: invert transforms(def: 0)\n"
    "    'relative'  consider transforms as 0: absolute, 1: relative (def)\n"
    "    'zoom'      percentage to zoom >0: zoom in, <0 zoom out (def: 0)\n"
    "    'optzoom'   0: nothing, 1: determine optimal static zoom (def)\n"
    "                i.e. no (or only little) border should be visible.\n"
    "                2: determine optimal adaptive zoom\n"
    "                Note that the value given at 'zoom' is added to the \n"
    "                here calculated one\n"
    "    'zoomspeed' for adaptive zoom: zoom per frame in percent \n"
    "    'interpol'  type of interpolation: 0: no interpolation, \n"
    "                1: linear (horizontal), 2: bi-linear (def), \n"
    "                3: bi-cubic\n"
    "    'sharpen'   amount of sharpening: 0: no sharpening (def: 0.8)\n"
    "                uses filter unsharp with 5x5 matrix\n"
    "    'tripod'    virtual tripod mode (=relative=0:smoothing=0)\n"
    "    'help'      print this help message\n";

/** returns the default config
 */
VS_API VSTransformConfig vsTransformGetDefaultConfig(const char* modName);

/** initialized the VSTransformData structure using the config and allocates memory
 *  for the frames and stuff
 *  @return VS_OK on success otherwise VS_ERROR
 */
VS_API int vsTransformDataInit(VSTransformData* td, const VSTransformConfig* conf,
                        const VSFrameInfo* fi_src, const VSFrameInfo* fi_dest);


/** Deletes internal data structures.
 * In order to use the VSTransformData again, you have to call vsTransformDataInit
 */
VS_API void vsTransformDataCleanup(VSTransformData* td);

/// returns the current config
VS_API void vsTransformGetConfig(VSTransformConfig* conf, const VSTransformData* td);

/// returns the frame info for the src
VS_API const VSFrameInfo* vsTransformGetSrcFrameInfo(const VSTransformData* td);
/// returns the frame info for the dest
VS_API const VSFrameInfo* vsTransformGetDestFrameInfo(const VSTransformData* td);


/// initializes VSTransformations structure
VS_API void vsTransformationsInit(VSTransformations* trans);
/// deletes VSTransformations internal memory
VS_API void vsTransformationsCleanup(VSTransformations* trans);

/// return next Transform and increases internal counter
VS_API VSTransform vsGetNextTransform(const VSTransformData* td, VSTransformations* trans);

/** preprocesses the list of transforms all at once. Here the deshaking is calculated!
 */
VS_API int vsPreprocessTransforms(VSTransformData* td, VSTransformations* trans);

/**
 * vsLowPassTransforms: single step smoothing of transforms, using only the past.
 *  see also vsPreprocessTransforms. */
VS_API VSTransform vsLowPassTransforms(VSTransformData* td, VSSlidingAvgTrans* mem,
                            const VSTransform* trans);

/** call this function to prepare for a next transformation (transformPacked/transformPlanar)
    and supply the src frame buffer and the frame to write to. These can be the same pointer
    for an inplace operation (working on framebuffer directly)
 */
VS_API int vsTransformPrepare(VSTransformData* td, const VSFrame* src, VSFrame* dest);

/// does the actual transformation
VS_API int vsDoTransform(VSTransformData* td, VSTransform t);


/** call this function to finish the transformation of a frame (transformPacked/transformPlanar)
 */
VS_API int vsTransformFinish(VSTransformData* td);


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
