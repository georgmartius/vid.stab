/*
 *  pix_formats.h
 *
 *  Copyright (C) Georg Martius - February 2013
 *   georg dot martius at web dot de
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

#ifndef __PIX_FORMATS_H
#define __PIX_FORMATS_H

#include "frameinfo.h"
#include "transcode.h"

static PixelFormat transcode2ourPF(int tc_img_codec){
	switch(tc_img_codec){
	case CODEC_RGB:
		return PF_RGB24;
	case CODEC_YUV:
		return PF_YUV420P;
	case CODEC_YUV422:
		return PF_YUV422P;
	default:
		tc_log_error(MOD_NAME, "cannot deal with img format %i!\n", tc_img_codec);
		return PF_NONE;
	}
}

#endif
