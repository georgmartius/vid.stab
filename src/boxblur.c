/*
 *  boxblur.c
 *
 *  Copyright (C) Georg Martius - July 2010
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

#include "boxblur.h"
#include "vidstabdefines.h"


void boxblur_hori_C(unsigned char* dest, const unsigned char* src,
										int width, int height, int dest_strive, int src_strive, int size);
void boxblur_vert_C(unsigned char* dest, const unsigned char* src,
										int width, int height, int dest_strive, int src_strive, int size);

/*
  The algorithm:
  box filter: kernel has only 1's
  a good blur is obtained for multiple runs of boxblur
  - 2 runs: tent kernel,  infinity -> gaussian
  but for our purposes is the tent kernel enough.

  horizontal and vertical 1D boxfilters can be used

  accumulator: acc = acc + new - old, pixel = acc/size
*/

void boxblurYUV(VSFrame* dest, const VSFrame* src,
		VSFrame* buffer, const VSFrameInfo* fi,
		unsigned int size, BoxBlurColorMode colormode){
  int localbuffer=0;
  int size2;
  if(size<2){
    if(dest!=src)
      copyFrame(dest,src,fi);
    return;
  }
	VSFrame buf;
  if(buffer==0){
		allocateFrame(&buf,fi);
		localbuffer=1;
  }else{
		buf = *buffer;
	}
  // odd and larger than 2 and maximally half of smaller image dimension
  size  = VS_CLAMP((size/2)*2+1,3,VS_MIN(fi->height/2,fi->width/2));
  //printf("%i\n",size);

  // luminance
  boxblur_hori_C(buf.data[0],  src->data[0],
								 fi->width, fi->height, buf.linesize[0],src->linesize[0], size);
  boxblur_vert_C(dest->data[0], buf.data[0],
								 fi->width, fi->height, dest->linesize[0], buf.linesize[0], size);

	size2 = size/2+1; 	// odd and larger than 0
	int plane;
	switch (colormode){
	case BoxBlurColor:
		// color
		if(size2>1){
			for(plane=1; plane<fi->planes; plane++){
				boxblur_hori_C(buf.data[plane], src->data[plane],
											 fi->width  >> getPlaneWidthSubS(fi,plane),
											 fi->height >> getPlaneHeightSubS(fi,plane),
											 buf.linesize[plane], src->linesize[plane], size2);
				boxblur_vert_C(dest->data[plane], buf.data[plane],
											 fi->width  >> getPlaneWidthSubS(fi,plane),
											 fi->height >> getPlaneHeightSubS(fi,plane),
											 dest->linesize[plane], buf.linesize[plane], size2);
			}
		}
		break;
	case BoxBlurKeepColor:
		// copy both color channels
		for(plane=1; plane<fi->planes; plane++){
			copyFramePlane(dest, src, fi, plane);
		}
	case BoxBlurNoColor: // do nothing
	default:
		break;
	}

  if(localbuffer)
    freeFrame(&buf);
}

/* /\* */
/*   The algorithm: */
/*   see boxblurYUV but here we for RGB */

/*   we add the 3 bytes of one pixel as if they where one number */
/* *\/ */
/* void boxblurRGB(const unsigned char* src, unsigned char* dest,  */
/* 		unsigned char* buffer, const VSFrameInfo* fi,  */
/* 		unsigned int size){ */
/*   int localbuffer=0; */
/*   if(buffer==0){ */
/*     buffer=(unsigned char*) vs_malloc(fi->framesize); */
/*     localbuffer=1; */
/*   } */
/*   // odd and larger than 2 and maximal half of smaller image dimension  */
/*   //  (and not larger than 256, because otherwise we can get an overflow) */
/*   size  = VS_CLAMP((size/2)*2+1,3,VS_MIN(256,VS_MIN(fi->height/2,fi->width/2)));  */

/*   // we need a different version of these functions for RGB */
/*   boxblur_hori_C(src, buffer, fi->width, fi->height, fi->strive, size);   */
/*   boxblur_vert_C(buffer, dest, fi->width, fi->height, fi->strive, size); */

/*   if(localbuffer) */
/*     vs_free(buffer); */
/* } */


void boxblur_hori_C(unsigned char* dest, const unsigned char* src,
										int width, int height, int dest_strive, int src_strive, int size){

  int i,j,k;
  unsigned int acc;
  const unsigned char *start, *end; // start and end of kernel
  unsigned char *current;     // current destination pixel
  int size2 = size/2; // size of one side of the kernel without center
  // #pragma omp parallel for private(acc),schedule(guided,2) (no speedup)
  for(j=0; j< height; j++){
    //  for(j=100; j< 101; j++){
    start = end = src + j*src_strive;
    current = dest + j*dest_strive;
    // initialize accumulator
    acc= (*start)*(size2+1); // left half of kernel with first pixel
    for(k=0; k<size2; k++){  // right half of kernel
      acc+=(*end);
      end++;
    }
    // go through the image
    for(i=0; i< width; i++){
      acc = acc + (*end) - (*start);
      if(i > size2) start++;
      if(i < width - size2 - 1) end++;
      (*current) = acc/size;
      current++;
    }
  }
}

void boxblur_vert_C(unsigned char* dest, const unsigned char* src,
		    int width, int height, int dest_strive, int src_strive, int size){

  int i,j,k;
  int acc;
  const unsigned char *start, *end; // start and end of kernel
  unsigned char *current;     // current destination pixel
  int size2 = size/2; // size of one side of the kernel without center
  for(i=0; i< width; i++){
    start = end = src + i;
    current = dest + i;
    // initialize accumulator
    acc= (*start)*(size2+1); // left half of kernel with first pixel
    for(k=0; k<size2; k++){  // right half of kernel
      acc+=(*end);
      end+=src_strive;
    }
    // go through the image
    for(j=0; j< height; j++){
      acc = acc - (*start) + (*end);
      if(j > size2) start+=src_strive;
      if(j < height - size2 - 1) end+=src_strive;
      *current = acc/size;
      current+=dest_strive;
    }
  }
}
