/*
 *  libvidstab.h
 *
 *  Created on: Feb 21, 2011
 *  Copyright (C) Georg Martius - June 2007
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

#ifndef LIBVIDSTAB_H
#define LIBVIDSTAB_H

/* Keep in sync with MAJOR/MINOR/PATCH_VERSION in the top-level CMakeLists.txt */
#define LIBVIDSTAB_VERSION "v1.2.0"

#include "frameinfo.h"
#include "motiondetect.h"
#include "transform.h"
#include "vsvector.h"
#include "serialize.h"
#include "localmotion2transform.h"

#endif  /* LIBVIDSTAB_H_ */

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
