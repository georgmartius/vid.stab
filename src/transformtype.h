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
#include "dsvector.h"

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
    int extra;    /* -1: ignore transform (only internal use);
                     0 for normal trans; 1 for inter scene cut (unused) */
} Transform;

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

typedef DSVector LocalMotions;
/// helper macro to access a localmotion in the DSVector
#define LMGet(localmotions,index) \
    ((LocalMotion*)ds_vector_get(localmotions,index))


/* helper functions to create and operate with transforms.
 * all functions are non-destructive
 * the "_" version uses non-pointer Transforms. This is slower
 * but useful when cascading calculations like
 * add_transforms_(mult_transform(&t1, 5.0), &t2)
 */
Transform null_transform(void);
Transform new_transform(double x, double y, double alpha,
                        double zoom, int extra);
Transform add_transforms(const Transform* t1, const Transform* t2);
Transform add_transforms_(const Transform t1, const Transform t2);
Transform sub_transforms(const Transform* t1, const Transform* t2);
Transform mult_transform(const Transform* t1, double f);
Transform mult_transform_(const Transform t1, double f);

void storeTransform(FILE* f, const Transform* t);

/* compares a transform with respect to x (for sort function) */
int cmp_trans_x(const void *t1, const void* t2);
/* compares a transform with respect to y (for sort function) */
int cmp_trans_y(const void *t1, const void* t2);
/* static int cmp_trans_alpha(const void *t1, const void* t2); */

/* compares two double values (for sort function)*/
int cmp_double(const void *t1, const void* t2);
/* compares two int values (for sort function)*/
int cmp_int(const void *t1, const void* t2);


/* calculates the median of an array of transforms,
 * considering only x and y
 */
Transform median_xy_transform(const Transform* transforms, int len);
/* median of a double array */
double median(double* ds, int len);
/* mean of a double array */
double mean(const double* ds, int len);
/* mean with cutted upper and lower pentile
 * (min and max are optionally returned)
 */
double cleanmean(double* ds, int len, double* minimum, double* maximum);
/* calulcates the cleaned mean of an array of transforms,
 * considerung only x and y
 */
Transform cleanmean_xy_transform(const Transform* transforms, int len);

/* calculates the cleaned (cutting of x-th percentil)
 * maximum and minimum of an array of transforms,
 * considerung only x and y
 */
void cleanmaxmin_xy_transform(const Transform* transforms, int len,
                              int percentil,
                              Transform* min, Transform* max);


/* helper function to work with local motions */

LocalMotion null_localmotion(void);
/// a new array of the v.x values is returned (ds_free has to be called)
int* localmotions_getx(const LocalMotions* localmotions);
/// a new array of the v.y values is returned (ds_free has to be called)
int* localmotions_gety(const LocalMotions* localmotions);
/// lm1 - lm2 only for the Vec (the remaining values are taken from lm1)
LocalMotion sub_localmotion(const LocalMotion* lm1, const LocalMotion* lm2);

/* calulcates the cleaned mean of the vector of localmotions
 * considerung only v.x and v.y
 */
LocalMotion cleanmean_localmotions(const LocalMotions* localmotions);



/* helper functions */

/* optimized round function */
inline static int myround(float x) {
    if(x>0)
        return x + 0.5;
    else
        return x - 0.5;
}


/* optimized floor function
   This does not give the correct value for negative integer values like -1.0. In this case
   it will produce -2.0.
*/
inline static int myfloor(float x) {
    if(x<0)
        return x - 1;
    else
        return x;
}


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
