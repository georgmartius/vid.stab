/*
 * libvidstab.c
 *
 *  Created on: Feb 21, 2011
 *  Copyright (C) Georg Martius - February 2011
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

#include "libvidstab.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/**** default values for memory and logging ****/

/// memory allocation with zero initialization
void* _zalloc(size_t size){
    return memset(malloc(size),0,size);
}

/// logging function
int _vs_log(int type, const char* tag, const char* format, ...){
    if(vs_log_level >= type){
        fprintf(stderr,"%s (%s):",
                type == VS_ERROR_TYPE ? "Error: " :
                type == VS_WARN_TYPE  ? "Warn:  " :
                type == VS_INFO_TYPE  ? "Info:  " :
                type == VS_MSG_TYPE   ? "Msg:   " : "Unknown",
                tag);
        va_list ap;
        va_start (ap, format);
        vfprintf (stderr, format, ap);
        va_end (ap);
        fprintf(stderr,"\n");
    }
    return 0;
}


vs_malloc_t vs_malloc   = malloc;
vs_realloc_t vs_realloc = realloc;
vs_free_t vs_free       = free;
vs_zalloc_t vs_zalloc   = _zalloc;

vs_strdup_t vs_strdup   = strdup;

vs_log_t vs_log         = _vs_log;
int VS_ERROR_TYPE = 0;
int VS_WARN_TYPE  = 1;
int VS_INFO_TYPE  = 2;
int VS_MSG_TYPE   = 3;

int VS_ERROR     = -1;
int VS_OK        = 0;

int vs_log_level = 4;

/*
 * Local variables:
 *   c-file-style: "stroustrup"
 *   c-file-offsets: ((case-label . *) (statement-case-intro . *))
 *   indent-tabs-mode: nil
 * End:
 *
 * vim: expandtab shiftwidth=4:
 */
