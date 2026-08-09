/*
 *  transform_internal.h
 *
 *  Copyright (C) Georg Martius - June 2007 - 2011
 *   georg dot martius at web dot de
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
#ifndef __TRANSFORM_INTERNAL_H
#define __TRANSFORM_INTERNAL_H

#include "transform.h"
#include "vidstab_api.h"
#include "transformfixedpoint.h"
#ifdef TESTING
#include "transformfloat.h"
#endif

/// name of the interpolation type
const char* getInterpolationTypeName(VSInterpolType type);

/** performs the smoothing of the camera path and modifies the transforms
    to compensate for the jiggle
    */
VS_API int cameraPathOptimization(VSTransformData* td, VSTransformations* trans);

VS_API int cameraPathAvg(VSTransformData* td, VSTransformations* trans);
VS_API int cameraPathGaussian(VSTransformData* td, VSTransformations* trans);
VS_API int cameraPathOptimalL1(VSTransformData* td, VSTransformations* trans);

/** Builds (or rebuilds) td->lensMaps for the current td->lensK, if needed. */
void lensEnsureMaps(VSTransformData* td);

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
