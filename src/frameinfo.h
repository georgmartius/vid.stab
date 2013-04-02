/*
 *  frameinfo.h
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
#ifndef FRAMEINFO_H
#define FRAMEINFO_H

#include <stddef.h>
#include <stdlib.h>
#include <inttypes.h>

/// pixel formats
typedef enum {PF_NONE = -1,
              PF_GRAY8,     ///<        Y        ,  8bpp
              PF_YUV420P,   ///< planar YUV 4:2:0, 12bpp, (1 Cr & Cb sample per 2x2 Y samples)
              PF_YUV422P,   ///< planar YUV 4:2:2, 16bpp, (1 Cr & Cb sample per 2x1 Y samples)
              PF_YUV444P,   ///< planar YUV 4:4:4, 24bpp, (1 Cr & Cb sample per 1x1 Y samples)
              PF_YUV410P,   ///< planar YUV 4:1:0,  9bpp, (1 Cr & Cb sample per 4x4 Y samples)
              PF_YUV411P,   ///< planar YUV 4:1:1, 12bpp, (1 Cr & Cb sample per 4x1 Y samples)
              PF_YUV440P,   ///< planar YUV 4:4:0 (1 Cr & Cb sample per 1x2 Y samples)
              PF_YUVA420P,  ///< planar YUV 4:2:0, 20bpp, (1 Cr & Cb sample per 2x2 Y & A samples)
              PF_PACKED,    ///< dummy: packed formats start here
              PF_RGB24,     ///< packed RGB 8:8:8, 24bpp, RGBRGB...
              PF_BGR24,     ///< packed RGB 8:8:8, 24bpp, BGRBGR...
              PF_RGBA,      ///< packed RGBA 8:8:8:8, 32bpp, RGBARGBA...
              PF_NUMBER     ///< number of pixel formats
} PixelFormat;

/** frame information for deshaking lib
    This only works for planar image formats
 */
typedef struct vsframeinfo {
  int width, height;
  int planes;        // number of planes (1 luma, 2,3 chroma, 4 alpha)
  int log2ChromaW; // subsampling of width in chroma planes
  int log2ChromaH; // subsampling of height in chroma planes
  PixelFormat pFormat;
  int bytesPerPixel; // number of bytes per pixel (for packed formats)
} VSFrameInfo;

/** frame data according to frameinfo
 */
typedef struct vsframe {
  uint8_t* data[4]; // data in planes. For packed data everthing is in plane 0
  int linesize[4]; // line size of each line in a the planes
} VSFrame;

// use it to calculate the CHROMA sizes (rounding is correct)
#define CHROMA_SIZE(width,log2sub)  (-(-(width) >> (log2sub)))

/// initializes the frameinfo for the given format
int initFrameInfo(VSFrameInfo* fi, int width, int height, PixelFormat pFormat);


/// returns the subsampling shift amount, horizonatally for the given plane
int getPlaneWidthSubS(const VSFrameInfo* fi, int plane);

/// returns the subsampling shift amount, vertically for the given plane
int getPlaneHeightSubS(const VSFrameInfo* fi, int plane);

/// zero initialization
void nullFrame(VSFrame* frame);

/// returns true if frame is null (data[0]==0)
int isNullFrame(const VSFrame* frame);

/// compares two frames for identity (based in data[0])
int equalFrames(const VSFrame* frame1,const VSFrame* frame2);

/// allocates memory for a frame
void allocateFrame(VSFrame* frame, const VSFrameInfo* fi);


/// copies the given plane number from src to dest
void copyFramePlane(VSFrame* dest, const VSFrame* src,
										const VSFrameInfo* fi, int plane);

/// copies src to dest
void copyFrame(VSFrame* dest, const VSFrame* src, const VSFrameInfo* fi);

/** fills the data pointer so that it corresponds to the img saved in the linear buffer.
    No copying is performed.
    Do not call freeFrame() on it.
 */
void fillFrameFromBuffer(VSFrame* frame, uint8_t* img, const VSFrameInfo* fi);

/// frees memory
void freeFrame(VSFrame* frame);

#endif  /* FRAMEINFO_H */

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
