/*
 *  transformtype.h
 *
 *  Copyright (C) Georg Martius - July 2011
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
#ifndef __BOXBLUR_H
#define __BOXBLUR_H

#include "frameinfo.h"

/** BoxBlurColor     - blur also color channels,
    BoxBlurKeepColor - copy original color channels
    BoxBlurNoColor   - do not touch color channels in dest
*/
typedef enum _BoxBlurColorMode { BoxBlurColor, BoxBlurKeepColor, BoxBlurNoColor} BoxBlurColorMode ;

/** performs a boxblur operation on src and stores results in dest.
 * It uses an accumulator method and separate horizontal and vertical runs
 * @param buffer may be given for intermediate results.
 *            If 0 then it is locally malloced
 * @param size of bluring kernel, (min 3 and it is made odd)
 * @param onlyLumincance if true color planes stay untouched
 */
void boxblurYUV(DSFrame* dest, const DSFrame* src,
		DSFrame* buffer, const DSFrameInfo* fi,
		unsigned int size, BoxBlurColorMode colormode);

#endif
