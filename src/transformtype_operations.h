/*
 *  transformtype_operations.h
 *
 *  Copyright (C) Georg Martius - June 2007 - 2013
 *
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  This file is part of vid.stab video stabilization library
 *
 *  vid.stab is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published
 *  by the Free Software Foundation; either version 2.1 of the License, or
 *  (at your option) any later version.
 *
 *  vid.stab is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with vid.stab; see the file COPYING.LESSER.  If not, see
 *  <https://www.gnu.org/licenses/>.
 *
 */
#ifndef __TRANSFORMTYPE_OPERATIONS_H
#define __TRANSFORMTYPE_OPERATIONS_H

#include "transformtype.h"
#include "vidstabdefines.h"
#include "vsvector.h"
#include "frameinfo.h"
#include "vidstab_api.h"

/// helper macro to access a localmotion in the VSVector
#define LMGet(localmotions,index) \
    ((LocalMotion*)vs_vector_get(localmotions,index))

/* helper functions to create and operate with transforms.
 * all functions are non-destructive
 * the "_" version uses non-pointer Transforms. This is slower
 * but useful when cascading calculations like
 * add_transforms_(mult_transform(&t1, 5.0), &t2)
 */
VS_API VSTransform null_transform(void);
VS_API VSTransform new_transform(double x, double y, double alpha,
                          double zoom, double barrel, double rshutter, int extra);
VS_API VSTransform add_transforms(const VSTransform* t1, const VSTransform* t2);
VS_API VSTransform add_transforms_(const VSTransform t1, const VSTransform t2);
VS_API VSTransform sub_transforms(const VSTransform* t1, const VSTransform* t2);
VS_API VSTransform mult_transform(const VSTransform* t1, double f);
VS_API VSTransform mult_transform_(const VSTransform t1, double f);

VS_API void storeVSTransform(FILE* f, const VSTransform* t);


typedef struct _preparedtransform {
  const VSTransform* t;
  double zcos_a;
  double zsin_a;
  double c_x;
  double c_y;
  /* Rotational model (VSTransformConfig.fov).  f <= 0 selects the similarity
     model and leaves r untouched, which is the default and is what keeps the
     old behaviour bit-identical rather than merely equivalent. */
  double f;        /* focal length in pixels */
  double z;        /* 1 + zoom/100, which zcos_a/zsin_a fold in for f <= 0 */
  double r[9];     /* the FORWARD rotation; see rotation_matrix_backward */
} PreparedTransform;

/* Focal length in pixels from a horizontal field of view in degrees, over a
   frame of the given width.  Returns 0 for fovDeg <= 0, which every consumer
   reads as "similarity model". */
VS_API double focal_from_fov(double fovDeg, int width);

/* The rotation of the BACKWARD map: the one a warp loop wants, taking a
   destination point to the source point it shows, as p_s = K r K^-1 p_d.

   r = Ry(-yaw) . Rx(pitch) . Rz(roll), and both the order and that leading
   minus are load-bearing:

     - rightmost acts first, so roll is applied before the two terms that
       read as translation at long focal length.  That is rotate-then-
       translate, matching the affine the similarity path computes;
     - Ry takes -yaw because K Ry(w) K^-1 shifts a point by +w f while the
       affine shifts it by -t.x.

   Get either wrong and the model still looks plausible -- it degenerates to
   something translation-like -- which is why tests/test_fovmodel.c asserts on
   the SHAPE of the convergence rather than on one tolerance.

   The forward map is the same thing with r transposed, rotations being
   orthogonal; prepare_transform_fov stores that transpose. */
VS_API void rotation_matrix_backward(double yaw, double pitch, double roll,
                                     double r[9]);

// transforms vector
VS_API PreparedTransform prepare_transform(const VSTransform* t, const VSFrameInfo* fi);
/* As prepare_transform, but modelling the motion as a rotation about the
   optical centre at focal length f.  f <= 0 is exactly prepare_transform. */
VS_API PreparedTransform prepare_transform_fov(const VSTransform* t,
                                               const VSFrameInfo* fi, double f);
// transforms vector (attention, only integer)
VS_API Vec transform_vec(const PreparedTransform* t, const Vec* v);
VS_API void transform_vec_double(double *x, double* y, const PreparedTransform* t, const Vec* v);

// subtract two vectors
VS_API Vec sub_vec(Vec v1, Vec v2);
// adds two vectors
VS_API Vec add_vec(Vec v1, Vec v2);
VS_API Vec field_to_vec(Field f);

/* compares a transform with respect to x (for sort function) */
VS_API int cmp_trans_x(const void *t1, const void* t2);
/* compares a transform with respect to y (for sort function) */
VS_API int cmp_trans_y(const void *t1, const void* t2);
/* static int cmp_trans_alpha(const void *t1, const void* t2); */

/* compares two double values (for sort function)*/
VS_API int cmp_double(const void *t1, const void* t2);
/* compares two int values (for sort function)*/
VS_API int cmp_int(const void *t1, const void* t2);


/** square of a number */
VS_API double sqr(double x);

/* calculates the median of an array of transforms,
 * considering only x and y
 */
VS_API VSTransform median_xy_transform(const VSTransform* transforms, int len);
/* median of a double array */
VS_API double median(double* ds, int len);
/* mean of a double array */
VS_API double mean(const double* ds, int len);
/* standard deviation of a double array */
VS_API double stddev(const double* ds, int len, double mean);
/* mean with cutted upper and lower pentile
 * (min and max are optionally returned)
 */
VS_API double cleanmean(double* ds, int len, double* minimum, double* maximum);
/* calulcates the cleaned mean of an array of transforms,
 * considerung only x and y
 */
VS_API VSTransform cleanmean_xy_transform(const VSTransform* transforms, int len);

/* calculates the cleaned (cutting of x-th percentil)
 * maximum and minimum of an array of transforms,
 * considerung only x and y
 */
VS_API void cleanmaxmin_xy_transform(const VSTransform* transforms, int len,
                              int percentil,
                              VSTransform* min, VSTransform* max);

/* calculates the required zoom value to have no borders visible
 */
VS_API double transform_get_required_zoom(const VSTransform* transform, int width, int height);

/* helper function to work with local motions */

VS_API LocalMotion null_localmotion(void);
/// a new array of the v.x values is returned (vs_free has to be called)
VS_API int* localmotions_getx(const LocalMotions* localmotions);
/// a new array of the v.y values is returned (vs_free has to be called)
VS_API int* localmotions_gety(const LocalMotions* localmotions);
/// lm1 - lm2 only for the Vec (the remaining values are taken from lm1)
VS_API LocalMotion sub_localmotion(const LocalMotion* lm1, const LocalMotion* lm2);

/* calulcates the cleaned mean of the vector of localmotions
 * considerung only v.x and v.y
 */
VS_API LocalMotion cleanmean_localmotions(const LocalMotions* localmotions);

VS_API VSArray localmotionsGetMatch(const LocalMotions* localmotions);

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
 *   c-basic-offset: 2 t
 * End:
 *
 * vim: expandtab shiftwidth=2:
 */
