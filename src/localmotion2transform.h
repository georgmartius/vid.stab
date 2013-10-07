/*
 * localmotion2transform.h
 *
 *  Copyright (C) Georg Martius - January 2013
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

#ifndef __LOCALMOTION2TRANSFORM_H
#define __LOCALMOTION2TRANSFORM_H


#include "transform.h"
#include "transformtype.h"
#include "serialize.h"


/** converts for each frame the localmotions into a transform
    @deprecated!
 */
int vsLocalmotions2TransformsSimple(VSTransformData* td,
                                    const VSManyLocalMotions* motions,
                                    VSTransformations* trans );

/** converts for each frame the localmotions into a transform
 */
int vsLocalmotions2Transforms(VSTransformData* td,
                              const VSManyLocalMotions* motions,
                              VSTransformations* trans );

/** calculates rotation angle for the given transform and
 * field with respect to the given center-point
 */
double vsCalcAngle(const LocalMotion* lm, int center_x, int center_y);

/** calculates the transformation that caused the observed motions.
    Using a simple cleaned-means approach to eliminate outliers.
    translation and rotation is calculated.
*/
VSTransform vsSimpleMotionsToTransform(VSTransformData* td,
                                   const LocalMotions* motions);


/** calculates the transformation that caused the observed motions.
    Using a gradient descent algorithm. Outliers are removed by repeated cross-validation.
*/
VSTransform vsMotionsToTransform(VSTransformData* td,
                                 const LocalMotions* motions);



/** general purpose gradient descent algorithm

 * Parameters:
 *       eval: evaluation function (value/energy to be minimized)
 *     params: initial starting parameters
 *        dat: custom data for eval function
 *          N: number of iterations (100)
 *  stepsizes: stepsizes for each dimension of the gradient {0.1,0.1...} (will be deleted)
 *  threshold: value below which the value/energy is considered to be minimized (0)
 *   residual: residual value (call by reference) (can be NULL)
 * Return Value:
 *     Optimized parameters
 */
VSArray vsGradientDescent(double (*eval)(VSArray, void*),
                         VSArray params, void* dat,
                         int N, VSArray stepsizes, double threshold, double* residual);

#endif
