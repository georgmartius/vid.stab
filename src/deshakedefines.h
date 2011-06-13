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
	printf("%s (%s):", type, tag); printf(format, ## args);

#define ds_log_error(tag, format, args...) \
    ds_log("\nError:", tag, format , ## args)
#define ds_log_info(tag, format, args...) \
    ds_log("\nInfo: ", tag, format , ## args)
#define ds_log_warn(tag, format, args...) \
    ds_log("\nWarn: ", tag, format , ## args)
#define ds_log_msg(tag, format, args...) \
    ds_log("\nMsg:  ", tag, format , ## args)

#define ds_strdup(s) strdup(s)
#define ds_strndup(s, n) strndup(s, n)

#define DS_ERROR -1
#define DS_OK 0

#endif // frameworks


#endif /* DESHAKEDEFINES_H_ */
