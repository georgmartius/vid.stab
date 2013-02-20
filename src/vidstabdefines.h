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
 *  the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *
*/

#ifndef VIDSTABDEFINES_H_
#define VIDSTABDEFINES_H_

#include <stddef.h>

#define VS_MAX(a, b)		(((a) > (b)) ?(a) :(b))
#define VS_MIN(a, b)		(((a) < (b)) ?(a) :(b))
/* clamp x between a and b */
#define VS_CLAMP(x, a, b)	VS_MIN(VS_MAX((a), (x)), (b))

#define VS_DEBUG 2

/// pixel in single layer image
#define PIXEL(img, x, y, w, h, def) ((x) < 0 || (y) < 0) ? (def)	\
  : (((x) >= (w) || (y) >= (h)) ? (def) : img[(x) + (y) * (w)])
/// pixel in single layer image without rangecheck
#define PIX(img, x, y, w, h) (img[(x) + (y) * (w)])
/// pixel in N-channel image. channel in {0..N-1}
#define PIXELN(img, x, y, w, h, N,channel , def) ((x) < 0 || (y) < 0) ? (def) \
  : (((x) >=(w) || (y) >= (h)) ? (def) : img[((x) + (y) * (w))*(N) + (channel)])
/// pixel in N-channel image without rangecheck. channel in {0..N-1}
#define PIXN(img, x, y, w, h, N,channel) (img[((x) + (y) * (w))*(N) + (channel)])


// define here the functions to be used in the framework we are in
// for transcode
#ifdef TRANSCODE
#include <transcode.h>
#define vs_malloc(size) tc_malloc(size)
#define vs_zalloc(size) tc_zalloc(size)
#define vs_realloc(ptr, size) tc_realloc(ptr, size)
#define vs_free(ptr) tc_free(ptr)

#define vs_log_error(tag, format, args...) \
    tc_log(TC_LOG_ERR, tag, format , ## args)
#define vs_log_info(tag, format, args...) \
    tc_log(TC_LOG_INFO, tag, format , ## args)
#define vs_log_warn(tag, format, args...) \
    tc_log(TC_LOG_WARN, tag, format , ## args)
#define vs_log_msg(tag, format, args...) \
    tc_log(TC_LOG_MSG, tag, format , ## args)

#define vs_strdup(s) tc_strdup(s)
#define vs_strndup(s, n) tc_strndup(s, n)

#define VS_ERROR TC_ERROR
#define VS_OK TC_OK


#else
#ifdef LIBAVFILTER // libavfilter library

#include <libavutil/log.h>
#include <libavutil/mem.h>
extern void* pctx;

#define vs_malloc(size) av_malloc(size)
#define vs_zalloc(size) av_mallocz(size)
#define vs_realloc(ptr, size) av_realloc(ptr, size)
#define vs_free(ptr) av_free(ptr)

#define vs_log_error(tag, format, args...) \
	av_log(pctx, AV_LOG_ERROR, tag, format , ## args)
#define vs_log_info(tag, format, args...) \
	av_log(pctx, AV_LOG_INFO, tag, format , ## args)
#define vs_log_warn(tag, format, args...) \
	av_log(pctx, AV_LOG_WARN, tag, format , ## args)
#define vs_log_msg(tag, format, args...) \
	av_log(pctx, AV_LOG_VERBOSE, tag, format , ## args)

#define vs_strdup(s) av_strdup(s)
#define vs_strndup(s, n) av_strndup(s, n)

#define VS_ERROR 0
#define VS_OK 1

#else
 // standard C framework
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#define vs_malloc(size) (void*)malloc(size)
#define vs_realloc(ptr, size) (void*)realloc(ptr, size)
#define vs_free(ptr) free(ptr)
#define vs_zalloc(size) memset(malloc(size),0,size)

#define vs_log(type, tag, format, args...) \
	{ fprintf(stderr,"%s (%s):", type, tag); fprintf(stderr,format, ## args); \
	  fprintf(stderr,"\n"); }

#define vs_log_error(tag, format, args...) \
    vs_log("Error:", tag, format , ## args)
#define vs_log_info(tag, format, args...) \
    vs_log("Info: ", tag, format , ## args)
#define vs_log_warn(tag, format, args...) \
    vs_log("Warn: ", tag, format , ## args)
#define vs_log_msg(tag, format, args...) \
    vs_log("Msg:  ", tag, format , ## args)

#define vs_strdup(s) strdup(s)
#define vs_strndup(s, n) strndup(s, n)

#define VS_ERROR -1
#define VS_OK 0
#endif // frameworks
#endif // frameworks


#endif /* VIDSTABDEFINES_H_ */
