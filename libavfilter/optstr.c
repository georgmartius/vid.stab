/*
 *  optstr.c
 *
 *  This is a shortened version of the optstr.c file from transcode
 *  Copyright (C) Tilmann Bitterberg 2003
 *
 *  Description: A general purpose option string parser
 *
 *  Usage: see optstr.h, please
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

#ifdef HAVE_CONFIG_H
# include "config.h"
#endif

/* for vsscanf */
#ifdef HAVE_VSSCANF
#  define _GNU_SOURCE
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include "optstr.h"


// take from transcode
const char* optstr_lookup(const char *haystack, const char *needle)
{
    const char *ch = haystack;
    int found = 0;
    size_t len = strlen(needle);

    while (!found) {
        ch = strstr(ch, needle);

        /* not in string */
        if (!ch) {
            break;
        }

        /* do we want this hit? ie is it exact? */
        if (ch[len] == '\0' || ch[len] == '=' || ch[len] == ARG_SEP) {
            found = 1;
        } else {
            /* go a little further */
            ch++;
        }
    }

    return ch;
}

int optstr_get(const char *options, const char *name, const char *fmt, ...)
{
    va_list ap;     /* points to each unnamed arg in turn */
    int num_args = 0, n = 0;
    size_t pos, fmt_len = strlen(fmt);
    const char *ch = NULL;

#ifndef HAVE_VSSCANF
    void *temp[ARG_MAXIMUM];
#endif

    ch = optstr_lookup(options, name);
    if (!ch) {
        return -1;
    }

    /* name IS in options */

    /* Find how many arguments we expect */
    for (pos = 0; pos < fmt_len; pos++) {
        if (fmt[pos] == '%') {
            ++num_args;
            /* is this one quoted  with '%%' */
            if (pos + 1 < fmt_len && fmt[pos + 1] == '%') {
                --num_args;
                ++pos;
            }
        }
    }

#ifndef HAVE_VSSCANF
    if (num_args > ARG_MAXIMUM) {
        fprintf (stderr,
            "(%s:%d) Internal Overflow; redefine ARG_MAXIMUM (%d) to something higher\n",
            __FILE__, __LINE__, ARG_MAXIMUM);
        return -2;
    }
#endif

    n = num_args;
    /* Bool argument */
    if (num_args <= 0) {
        return 0;
    }

    /* skip the `=' (if it is one) */
    ch += strlen( name );
    if( *ch == '=' )
        ch++;

    if( !*ch )
        return 0;

    va_start(ap, fmt);

#ifndef HAVE_VSSCANF
    while (--n >= 0) {
        temp[num_args - n - 1] = va_arg(ap, void *);
    }

    n = sscanf(ch, fmt,
            temp[0],  temp[1],  temp[2],  temp[3]);

#else
    /* this would be very nice instead of the above,
     * but it does not seem portable
     */
     n = vsscanf(ch, fmt, ap);
#endif

    va_end(ap);

    return n;
}
