/*
 *  boxblur.c
 *
 *  Copyright (C) Georg Martius - July 2010
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

#include "boxblur.h"
#include "deshakedefines.h"


void boxblur_hori_C(const unsigned char* src, unsigned char* dest, 
		    int width, int height, int strive, int size);
void boxblur_vert_C(const unsigned char* src, unsigned char* dest, 
		    int width, int height, int strive, int size);

/*
  The algorithm:
  box filter: kernel has only 1's
  a good blur is obtained for multiple runs of boxblur 
  - 2 runs: tent kernel,  infinity -> gaussian 
  but for our purposes is the tent kernel enough.
  
  horizontal and vertical 1D boxfilters can be used
  
  accumulator: acc = acc + new - old, pixel = acc/size
*/

void boxblurYUV(const unsigned char* src, unsigned char* dest, 
		unsigned char* buffer, const DSFrameInfo* fi, 
		unsigned int size){
  int localbuffer=0;
  int size2;
  int offset = fi->width * fi->height;
  if(buffer==0){
    buffer=(unsigned char*) ds_malloc(fi->framesize);
    localbuffer=1;
  }
  // odd and larger than 2 and maximal half of smaller image dimension
  size  = DS_CLAMP((size/2)*2+1,3,DS_MIN(fi->height/2,fi->width/2)); 
  size2 = size/2+1;                        // odd and larger than 0
  
  // luminance  
  boxblur_hori_C(src, buffer, fi->width, fi->height, fi->strive, size);  
  // color
  if(size2>1){
    boxblur_hori_C(src+offset, buffer+offset, 
		   fi->width/2, fi->height/2, fi->strive/2, size2); 
    boxblur_hori_C(src+5*offset/4, buffer+5*offset/4, 
		   fi->width/2, fi->height/2, fi->strive/2, size2); 
  }

  boxblur_vert_C(buffer, dest, fi->width, fi->height, fi->strive, size);
  // color
  if(size2>1){
    boxblur_vert_C(buffer+offset, dest+offset, 
		   fi->width/2, fi->height/2, fi->strive/2, size2); 
    boxblur_vert_C(buffer+5*offset/4, dest+5*offset/4, 
		   fi->width/2, fi->height/2, fi->strive/2, size2); 
  }

  if(localbuffer)
    ds_free(buffer);
}

void boxblur_hori_C(const unsigned char* src, unsigned char* dest, 
		    int width, int height, int strive, int size){
  
  int i,j,k;
  unsigned int acc;
  const unsigned char *start, *end; // start and end of kernel
  unsigned char *current;     // current destination pixel
  int size2 = size/2; // size one side of the kernel without center
  for(j=0; j< height; j++){    
    //  for(j=100; j< 101; j++){    
    start = end = src + j*strive;
    current = dest + j*strive;
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

// 0 1 2 3 4
// 1 1 

void boxblur_vert_C(const unsigned char* src, unsigned char* dest, 
		    int width, int height, int strive, int size){
  
  int i,j,k;
  int acc;
  const unsigned char *start, *end; // start and end of kernel
  unsigned char *current;     // current destination pixel
  int size2 = size/2; // size one side of the kernel without center
  for(i=0; i< width; i++){    
    start = end = src + i;
    current = dest + i;
    // initialize accumulator
    acc= (*start)*(size2+1); // left half of kernel with first pixel
    for(k=0; k<size2; k++){  // right half of kernel
      acc+=(*end);
      end+=strive;
    }
    // go through the image
    for(j=0; j< height; j++){
      acc = acc - (*start) + (*end);
      if(j > size2) start+=strive;
      if(j < height - size2 - 1) end+=strive;      
      *current = acc/size;
      current+=strive;
    }
  }
}
