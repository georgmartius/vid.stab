/*
 * deshakedefines.h
 *
 *  Created on: Feb 23, 2011
 *      Author: georg
 */

#ifndef DESHAKEDEFINES_H_
#define DESHAKEDEFINES_H_

#include <stddef.h>

#define DS_MAX(a, b)		(((a) > (b)) ?(a) :(b))
#define DS_MIN(a, b)		(((a) < (b)) ?(a) :(b))
/* clamp x between a and b */
#define DS_CLAMP(x, a, b)	DS_MIN(DS_MAX((a), (x)), (b))

#define DS_DEBUG 2

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
#define ds_malloc(size) tc_malloc(size)
#define ds_zalloc(size) tc_zalloc(size)
#define ds_realloc(ptr, size) tc_realloc(ptr, size)
#define ds_free(ptr) tc_free(ptr)

#define ds_log_error(tag, format, args...) \
    tc_log(TC_LOG_ERR, tag, format , ## args)
#define ds_log_info(tag, format, args...) \
    tc_log(TC_LOG_INFO, tag, format , ## args)
#define ds_log_warn(tag, format, args...) \
    tc_log(TC_LOG_WARN, tag, format , ## args)
#define ds_log_msg(tag, format, args...) \
    tc_log(TC_LOG_MSG, tag, format , ## args)

#define ds_strdup(s) tc_strdup(s)
#define ds_strndup(s, n) tc_strndup(s, n)

#define DS_ERROR TC_ERROR
#define DS_OK TC_OK
#else
// standard C framework
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#define ds_malloc(size) (void*)malloc(size)
#define ds_realloc(ptr, size) (void*)realloc(ptr, size)
#define ds_free(ptr) free(ptr)
#define ds_zalloc(size) memset(malloc(size),0,size)

#define ds_log(type, tag, format, args...) \
	fprintf(stderr,"%s (%s):", type, tag); fprintf(stderr,format, ## args); \
	fprintf(stderr,"\n");

#define ds_log_error(tag, format, args...) \
    ds_log("Error:", tag, format , ## args)
#define ds_log_info(tag, format, args...) \
    ds_log("Info: ", tag, format , ## args)
#define ds_log_warn(tag, format, args...) \
    ds_log("Warn: ", tag, format , ## args)
#define ds_log_msg(tag, format, args...) \
    ds_log("Msg:  ", tag, format , ## args)

#define ds_strdup(s) strdup(s)
#define ds_strndup(s, n) strndup(s, n)

#define DS_ERROR -1
#define DS_OK 0

#endif // frameworks


#endif /* DESHAKEDEFINES_H_ */
