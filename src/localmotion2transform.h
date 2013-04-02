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
 */
int localmotions2TransformsSimple(TransformData* td,
																	const ManyLocalMotions* motions,
																	Transformations* trans );


/** calculates rotation angle for the given transform and
 * field with respect to the given center-point
 */
double calcAngle(const LocalMotion* lm, int center_x, int center_y);

/** calculates the transformation that caused the observed motions.
    Using a simple cleaned-means approach to eliminate outliers.
    translation and rotation is calculated.
*/
Transform simpleMotionsToTransform(TransformData* td,
                                   const LocalMotions* motions);

#endif
