/*
 *  transform.h
 *
 *  Copyright (C) Georg Martius - June 2007 - 2013
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
#ifndef __TRANSFORMTYPE_H
#define __TRANSFORMTYPE_H

#include <stdio.h>
#include "vsvector.h"

/* structure to hold information about frame transformations
   x,y are translations, alpha is a rotation around the center in RAD,
   zoom is a percentage to zoom in and
   extra is for additional information like scene cut (unused)
 */
typedef struct _transform {
    double x;
    double y;
    double alpha;
    double zoom;
    double barrel;
    double rshutter;
    int extra;    /* -1: ignore transform (only internal use);
                     0 for normal trans; 1 for inter scene cut (unused) */
} VSTransform;

/** stores x y and size of a measurement field */
typedef struct _field {
  int x;     // middle position x
  int y;     // middle position y
  int size;  // size of field
} Field;

/** stores x y coordinates (integer) */
typedef struct _vec {
  int x;     // middle position x
  int y;     // middle position y
} Vec;

/* structure to hold information about local motion.
 */
typedef struct _localmotion {
    Vec v;
    Field f;
    double contrast; // local contrast of the measurement field
    double match;    // quality of match
} LocalMotion;

typedef VSVector LocalMotions;

#endif

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
