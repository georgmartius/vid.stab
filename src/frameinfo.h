/*
 *  frameinfo.h
 *
 *  Created on: Feb 21, 2011
 *  Copyright (C) Georg Martius - June 2007
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

#ifndef FRAMEINFO_H
#define FRAMEINFO_H

#include <stddef.h>
#include <stdlib.h>

typedef enum {PF_RGB=1, PF_YUV = 2 /*,PF_YUY2=8, PF_YUV422=256*/} PixelFormat;

/** initialization data structure for motion detection part of deshaking*/
typedef struct dsframeinfo {
  size_t framesize;  // size of frame buffer in bytes (prev)
  int width, height, strive;
  PixelFormat pFormat;
} DSFrameInfo;



#endif  /* FRAMEINFO_H */

/*
 * Local variables:
 *   c-file-style: "stroustrup"
 *   c-file-offsets: ((case-label . *) (statement-case-intro . *))
 *   indent-tabs-mode: nil
 * End:
 *
 * vim: expandtab shiftwidth=4:
 */
