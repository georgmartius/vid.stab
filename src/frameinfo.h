/*
 *  frameinfo.h
 *
 *  Copyright (C) Georg Martius - June 2007 - 2011
 *   georg dot martius at web dot de  
 *
 *  This file is part of vid.stab video stabilization library
 *      
 *  vid.stab is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License,
 *   WITH THE RESTRICTION for NONCOMMERICIAL USAGE see below, 
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
 *  This work is licensed under the Creative Commons         
 *  Attribution-NonCommercial-ShareAlike 2.5 License. To view a copy of   
 *  this license, visit http://creativecommons.org/licenses/by-nc-sa/2.5/ 
 *  or send a letter to Creative Commons, 543 Howard Street, 5th Floor,   
 *  San Francisco, California, 94105, USA.                                
 *  This EXCLUDES COMMERCIAL USAGE
 *
 */
#ifndef FRAMEINFO_H
#define FRAMEINFO_H

#include <stddef.h>
#include <stdlib.h>

typedef enum {PF_RGB=1, PF_YUV = 2 /*,PF_YUY2=8, PF_YUV422=256*/} PixelFormat;

/** frame information for deshaking lib*/
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
