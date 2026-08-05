/*
 * vidstabdefines.h
 *
 *  Created on: Feb 23, 2011
 *      Author: georg
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
 *  the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 *  Boston, MA 02110-1301, USA.
 *
 *
*/

#ifndef VIDSTABDEFINES_H_
#define VIDSTABDEFINES_H_

#include <stddef.h>
#include <stdlib.h>

#include "vidstab_api.h"

#ifdef __GNUC__
#define likely(x)       __builtin_expect(!!(x), 1)
#define unlikely(x)     __builtin_expect(!!(x), 0)
#else
#define likely(x)        (x)
#define unlikely(x)      (x)
#endif

#define VS_MAX(a, b)    (((a) > (b)) ?(a) :(b))
#define VS_MIN(a, b)    (((a) < (b)) ?(a) :(b))
/* clamp x between a and b */
#define VS_CLAMP(x, a, b)  VS_MIN(VS_MAX((a), (x)), (b))

#define VS_DEBUG 2

/**** Pixel access ***********************************************************
 *
 * UNIT CONVENTION (holds for all PIX* macros below, for VSFrame::linesize and
 * for every function in this library that takes a linesize):
 *
 *   `linesize` is the distance between the start of two consecutive rows
 *   measured in BYTES -- exactly like FFmpeg's AVFrame::linesize. It may be
 *   larger than width*bytesPerPixel (padding).
 *
 * Consequently only the COLUMN index is scaled by the number of channels
 * (N == bytesPerPixel for the packed formats), never the row offset:
 *
 *   byte offset = x*N + y*linesize + channel
 *
 * Never pass a linesize that was pre-divided by the number of channels.
 *****************************************************************************/

/// pixel in single layer image
#define PIXEL(img, linesize, x, y, w, h, def) \
  (((x) < 0 || (y) < 0 || (x) >= (w) || (y) >= (h)) ? (def) : img[(x) + (y) * (linesize)])
/// pixel in single layer image without rangecheck
#define PIX(img, linesize, x, y) (img[(x) + (y) * (linesize)])
/// pixel in N-channel image. channel in {0..N-1}
#define PIXELN(img, linesize, x, y, w, h, N, channel, def)                     \
  (((x) < 0 || (y) < 0 || (x) >= (w) || (y) >= (h)                             \
    || (channel) < 0 || (channel) >= (N))                                      \
   ? (def) : img[(x)*(N) + (y) * (linesize) + (channel)])
/// pixel in N-channel image without rangecheck. channel in {0..N-1}
#define PIXN(img, linesize, x, y, N, channel) \
  (img[(x)*(N) + (y) * (linesize) + (channel)])

/**** Configurable memory and logging functions. Defined in libvidstab.c ****/

typedef void* (*vs_malloc_t) (size_t size);
typedef void* (*vs_realloc_t) (void* ptr, size_t size);
typedef void (*vs_free_t) (void* ptr);
typedef void* (*vs_zalloc_t) (size_t size);

typedef int (*vs_log_t) (int type, const char* tag, const char* format, ...);

typedef char* (*vs_strdup_t) (const char* s);

extern VS_API vs_log_t vs_log;
extern VS_API int vs_log_level;

extern VS_API vs_malloc_t vs_malloc;
extern VS_API vs_realloc_t vs_realloc;
extern VS_API vs_free_t vs_free;
extern VS_API vs_zalloc_t vs_zalloc;

extern VS_API vs_strdup_t vs_strdup;

extern VS_API int VS_ERROR_TYPE;
extern VS_API int VS_WARN_TYPE;
extern VS_API int VS_INFO_TYPE;
extern VS_API int VS_MSG_TYPE;

extern VS_API int VS_ERROR;
extern VS_API int VS_OK;
/* The message type goes in front of __VA_ARGS__, which leaves tag and format
   inside the variadic part.  The variadic part is therefore never empty, and
   these need neither the GNU ", ## args" extension nor C23's __VA_OPT__ to
   swallow the comma of an argument list that stops after the format. */
#define vs_log_error(...) vs_log(VS_ERROR_TYPE, __VA_ARGS__)
#define vs_log_warn(...)  vs_log(VS_WARN_TYPE,  __VA_ARGS__)
#define vs_log_info(...)  vs_log(VS_INFO_TYPE,  __VA_ARGS__)
#define vs_log_msg(...)   vs_log(VS_MSG_TYPE,   __VA_ARGS__)

#endif /* VIDSTABDEFINES_H_ */
