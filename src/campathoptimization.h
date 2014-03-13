/*
 *  campathoptimization.h
 *
 *  Copyright (C) Georg Martius - January 2014
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
#ifndef __CAMPATHOPTIMIZATION_H
#define __CAMPATHOPTIMIZATION_H

#include "transform.h"
#include "transformtype_operations.h"

// ---------------------
/** 4DOF linear similarity: corresponds to transformation matrix: ((a , b, x),(-b, a, y),(0,0,c))
    This is like VSTransform, but in a different parametrization, which is more suitable for smoothing. c is typically 1, but if we add transforms it can differ.
 */
typedef struct _vsTransformLS {
  double x;
  double y;
  double a;
  double b;
  double c;
  int extra;
} VSTransformLS;

typedef struct _vstransformationsLS {
    VSTransformLS* ts; // array of transformations
    int current;   // index to current transformation
    int len;       // length of trans array
} VSTransformationsLS;

/** performs the smoothing of the camera path and modifies the transforms
    to compensate for the jiggle
    */
int cameraPathOptimization(VSTransformData* td, VSTransformations* trans);

int cameraPathAvg(VSTransformData* td, VSTransformations* trans);
int cameraPathGaussian(VSTransformData* td, VSTransformations* trans);
int cameraPathOptimalL1(VSTransformData* td, VSTransformations* trans);

/* ****** operations on VSTransformLS ************** */
/// converts the transformations list
VSTransformationsLS transformationsAZtoLS(const VSTransformations* trans);

/* helper functions to create and operate with transformLSs.
 *  (same as VSTransform, but differently parametrized)
 */
VSTransformLS id_transformLS(void);
VSTransformLS new_transformLS(double x, double y, double a, double b, double c, int extra);

VSTransformLS add_transformLS_(VSTransformLS t1, VSTransformLS t2);
VSTransformLS sub_transformLS(const VSTransformLS* t1, const VSTransformLS* t2);
VSTransformLS sub_transformLS_(VSTransformLS t1, VSTransformLS t2);
VSTransformLS invert_transformLS(const VSTransformLS* t1);
VSTransformLS concat_transformLS(const VSTransformLS* t1, const VSTransformLS* t2);

void storeVSTransformLS(FILE* f, const VSTransformLS* t);

VSTransformLS transformAZtoLS(const VSTransform* t);
VSTransform transformLStoAZ(const VSTransformLS* t);

PreparedTransform prepare_transformLS(const VSTransformLS* t, const VSFrameInfo* fi);


#endif  /* __CAMPATHOPTIMIZATION_H */

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
