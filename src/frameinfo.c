/*
 *  frameinfo.c
 *
 *  Copyright (C) Georg Martius - Feb - 2013
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

#include "frameinfo.h"
#include "vidstabdefines.h"
#include <assert.h>
#include <string.h>

int vsFrameInfoInit(VSFrameInfo* fi, int width, int height, VSPixelFormat pFormat){
  fi->pFormat=pFormat;
  fi->width = width;
  fi->height = height;
  fi->planes=3;
  fi->log2ChromaW = 0;
  fi->log2ChromaH = 0;
  fi->bytesPerPixel=1;
  assert(width%2==0 && height%2==0);
  switch(pFormat){
   case PF_GRAY8:
    fi->planes=1;
    break;
   case PF_YUV420P:
    fi->log2ChromaW = 1;
    fi->log2ChromaH = 1;
    break;
   case PF_YUV422P:
    fi->log2ChromaW = 1;
    fi->log2ChromaH = 0;
    break;
   case PF_YUV444P:
    break;
   case PF_YUV410P:
    fi->log2ChromaW = 2;
    fi->log2ChromaH = 2;
    break;
   case PF_YUV411P:
    fi->log2ChromaW = 2;
    fi->log2ChromaH = 0;
    break;
   case PF_YUV440P:
    fi->log2ChromaW = 0;
    fi->log2ChromaH = 1;
    break;
   case PF_YUVA420P:
    fi->log2ChromaW = 1;
    fi->log2ChromaH = 1;
    fi->planes = 4;
    break;
   case PF_RGB24:
   case PF_BGR24:
    fi->bytesPerPixel=3;
    fi->planes = 0;
    break;
   case PF_RGBA:
    fi->bytesPerPixel=4;
    fi->planes = 0;
    break;
   default:
    fi->pFormat=0;
    return 0;
  }
  return 1;
}

int vsGetPlaneWidthSubS(const VSFrameInfo* fi, int plane){
  return plane == 1 || plane == 2 ? fi->log2ChromaW : 0;
}

int vsGetPlaneHeightSubS(const VSFrameInfo* fi, int plane){
  return  plane == 1 || plane == 2 ? fi->log2ChromaH : 0;
}

int vsFrameIsNull(const VSFrame* frame) {
  return frame==0 || frame->data[0]==0;
}


int vsFramesEqual(const VSFrame* frame1, const VSFrame* frame2){
  return frame1 && frame2 && ( (frame1==frame2) || (frame1->data[0] == frame2->data[0]) );
}

void vsFrameNull(VSFrame* frame){
  memset(frame->data,0,sizeof(uint8_t*)*4);
  memset(frame->linesize,0,sizeof(int)*4);
}

void vsFrameAllocate(VSFrame* frame, const VSFrameInfo* fi){
  vsFrameNull(frame);
  if(fi->pFormat<PF_PACKED){
    int i;
    assert(fi->planes > 0 && fi->planes <= 4);
    for (i=0; i< fi->planes; i++){
      int w = fi->width  >> vsGetPlaneWidthSubS(fi, i);
      int h = fi->height >> vsGetPlaneHeightSubS(fi, i);
      frame->data[i] = vs_zalloc(w * h * sizeof(uint8_t));
      frame->linesize[i] = w;
      if(frame->data[i]==0)
        vs_log_error("vid.stab","out of memory: cannot allocated buffer");
    }
  }else{
    assert(fi->planes==1);
    int w = fi->width;
    int h = fi->height;
    frame->data[0] = vs_zalloc(w * h * sizeof(uint8_t)*fi->bytesPerPixel);
    frame->linesize[0] = w * fi->bytesPerPixel;
    if(frame->data[0]==0)
      vs_log_error("vid.stab","out of memory: cannot allocated buffer");
  }
}

void vsFrameCopyPlane(VSFrame* const dest, const VSFrame* src,
                      const VSFrameInfo* fi, int plane){
  assert(src->data[plane]);
  int h = fi->height >> vsGetPlaneHeightSubS(fi, plane);
  if (src->linesize[plane] == dest->linesize[plane]) {
    const int32_t dSize = src->linesize[plane] *  h * sizeof(uint8_t);
    memcpy(dest->data[plane], src->data[plane], dSize);
  } else {
    uint8_t* d = dest->data[plane];
    const uint8_t* s = src->data[plane];
    int w = fi->width  >> vsGetPlaneWidthSubS(fi, plane);
    for (; h>0; h--) {
      memcpy(d, s, sizeof(uint8_t) * w);
      d += dest->linesize[plane];
      s += src ->linesize[plane];
    }
  }
}

void vsFrameCopy(VSFrame* const dest, const VSFrame* src, const VSFrameInfo* fi) {
  assert( (fi->planes > 0) && (fi->planes <= 4) );
  for (int plane = 0; plane < fi->planes; plane++){
    vsFrameCopyPlane(dest, src, fi, plane);
  }
}

void vsFrameFillFromBuffer(VSFrame* frame, uint8_t* img, const VSFrameInfo* fi){
  assert(fi->planes > 0 && fi->planes <= 4);
  vsFrameNull(frame);
  long int offset = 0;
  int i;
  for (i=0; i< fi->planes; i++){
    int w = fi->width  >> vsGetPlaneWidthSubS(fi, i);
    int h = fi->height >> vsGetPlaneHeightSubS(fi, i);
    frame->data[i] = img + offset;
    frame->linesize[i] = w*fi->bytesPerPixel;
    offset += h * w*fi->bytesPerPixel;
  }
}

void vsFrameFree(VSFrame* frame){
  int plane;
  for (plane=0; plane< 4; plane++){
    if(frame->data[plane]) vs_free(frame->data[plane]);
    frame->data[plane]=0;
    frame->linesize[plane]=0;
  }
}


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
