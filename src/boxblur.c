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
 *  the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 *  Boston, MA 02110-1301, USA.
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

void boxblurPlanar(VSFrame* dest, const VSFrame* src,
    VSFrame* buffer, const VSFrameInfo* fi,
    unsigned int size, BoxBlurColorMode colormode){
  int localbuffer=0;
  int size2;
  if(size<2){
    if(dest!=src)
      vsFrameCopy(dest,src,fi);
    return;
  }
  VSFrame buf;
  if(buffer==0){
    vsFrameAllocate(&buf,fi);
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

  size2 = size/2+1;   // odd and larger than 0
  int plane;
  switch (colormode){
  case BoxBlurColor:
    // color
    if(size2>1){
      for(plane=1; plane<fi->planes; plane++){
        boxblur_hori_C(buf.data[plane], src->data[plane],
                       fi->width  >> vsGetPlaneWidthSubS(fi,plane),
                       fi->height >> vsGetPlaneHeightSubS(fi,plane),
                       buf.linesize[plane], src->linesize[plane], size2);
        boxblur_vert_C(dest->data[plane], buf.data[plane],
                       fi->width  >> vsGetPlaneWidthSubS(fi,plane),
                       fi->height >> vsGetPlaneHeightSubS(fi,plane),
                       dest->linesize[plane], buf.linesize[plane], size2);
      }
    }
    break;
  case BoxBlurKeepColor:
    // copy both color channels
    for(plane=1; plane<fi->planes; plane++){
      vsFrameCopyPlane(dest, src, fi, plane);
    }
  case BoxBlurNoColor: // do nothing
  default:
    break;
  }

  if(localbuffer)
    vsFrameFree(&buf);
}

/* /\* */
/*   The algorithm: */
/*   see boxblurPlanar but here we for Packed */

/*   we add the 3 bytes of one pixel as if they where one number */
/* *\/ */
/* void boxblurPacked(const unsigned char* src, unsigned char* dest,  */
/*     unsigned char* buffer, const VSFrameInfo* fi,  */
/*     unsigned int size){ */
/*   int localbuffer=0; */
/*   if(buffer==0){ */
/*     buffer=(unsigned char*) vs_malloc(fi->framesize); */
/*     localbuffer=1; */
/*   } */
/*   // odd and larger than 2 and maximal half of smaller image dimension  */
/*   //  (and not larger than 256, because otherwise we can get an overflow) */
/*   size  = VS_CLAMP((size/2)*2+1,3,VS_MIN(256,VS_MIN(fi->height/2,fi->width/2)));  */

/*   // we need a different version of these functions for Packed */
/*   boxblur_hori_C(src, buffer, fi->width, fi->height, fi->strive, size);   */
/*   boxblur_vert_C(buffer, dest, fi->width, fi->height, fi->strive, size); */

/*   if(localbuffer) */
/*     vs_free(buffer); */
/* } */


void boxblur_hori_C(unsigned char* dest, const unsigned char* src,
                    int width, int height, int dest_strive, int src_strive, int size){

  int j;
  int size2 = size/2; // size of one side of the kernel without center
  /* Every row is an independent accumulator chain, so this parallelises with
     no sharing at all.  (It used to be commented out as "no speedup"; that no
     longer holds -- at 1080p the two boxblur passes are several ms of purely
     serial work per frame, which is a large part of the frame time once the
     motion search itself runs on all cores.) */
#ifdef USE_OMP
#pragma omp parallel for schedule(static)
#endif
  for(j=0; j< height; j++){
    int i,k;
    unsigned int acc;
    const unsigned char *start, *end; // start and end of kernel
    unsigned char *current;     // current destination pixel
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

/* The vertical pass used to run one whole column at a time, which touches a
   new cache line for every single pixel and leaves the compiler nothing to
   vectorize (each column is one serial accumulator chain).

   Turning the loops inside out fixes both: the accumulators for *all* columns
   are held in one array and the image is walked row by row, so memory is read
   and written sequentially and the per-row inner loops are independent across
   columns and auto-vectorize.

   The recurrence is exactly the one the column-at-a-time version implemented,
   including its edge behaviour, so the output is bit identical:
     acc(-1) = src[0][i]*(size2+1) + sum over rows 0..size2-1
     acc(j)  = acc(j-1) + src[endRow(j)][i] - src[startRow(j)][i]
   with  endRow(j)   = min(j + size2, height - 1)
         startRow(j) = max(0, j - size2 - 1)
   (in the original those are the positions `end` and `start` had *before* the
   conditional increments at the bottom of the loop body). */
void boxblur_vert_C(unsigned char* dest, const unsigned char* src,
        int width, int height, int dest_strive, int src_strive, int size){

  int i,j,k;
  int size2 = size/2; // size of one side of the kernel without center
  uint32_t* acc;

  if(width <= 0 || height <= 0)
    return;

  acc = (uint32_t*)vs_malloc((size_t)width * sizeof(uint32_t));
  if(acc == NULL){ // fall back to the in-place column walk rather than fail
    for(i=0; i< width; i++){
      const unsigned char *start, *end;
      unsigned char *current;
      int a;
      start = end = src + i;
      current = dest + i;
      a = (*start)*(size2+1);
      for(k=0; k<size2; k++){ a+=(*end); end+=src_strive; }
      for(j=0; j< height; j++){
        a = a - (*start) + (*end);
        if(j > size2) start+=src_strive;
        if(j < height - size2 - 1) end+=src_strive;
        *current = a/size;
        current+=dest_strive;
      }
    }
    return;
  }

  /* Columns are independent of each other, so the work is split into blocks of
     columns.  Threads touch disjoint ranges of acc[] and of every row, so no
     synchronisation and no per-thread buffer is needed.  The block width keeps
     one block's accumulators plus the two source rows comfortably in L1. */
  {
    const int BLOCK = 512;
    int nblocks = (width + BLOCK - 1) / BLOCK;
    int cb;
#ifdef USE_OMP
#pragma omp parallel for schedule(static)
#endif
    for(cb=0; cb<nblocks; cb++){
      int c0 = cb*BLOCK;
      int c1 = c0 + BLOCK; if(c1 > width) c1 = width;
      int ii, jj, kk;

      // initialize accumulators: left half of the kernel with the first row ...
      for(ii=c0; ii<c1; ii++)
        acc[ii] = (uint32_t)src[ii] * (uint32_t)(size2+1);
      // ... plus rows 0 .. size2-1 for the right half
      for(kk=0; kk<size2; kk++){
        const unsigned char* row = src + (kk < height ? kk : height-1)*src_strive;
        for(ii=c0; ii<c1; ii++)
          acc[ii] += row[ii];
      }

      for(jj=0; jj<height; jj++){
        int er = jj + size2;          if(er > height-1) er = height-1;
        int sr = jj - size2 - 1;      if(sr < 0)        sr = 0;
        const unsigned char* endRow   = src + er*src_strive;
        const unsigned char* startRow = src + sr*src_strive;
        unsigned char* out = dest + jj*dest_strive;

        for(ii=c0; ii<c1; ii++)
          acc[ii] += (uint32_t)endRow[ii] - (uint32_t)startRow[ii];
        for(ii=c0; ii<c1; ii++)
          out[ii] = (unsigned char)(acc[ii]/size);
      }
    }
  }

  vs_free(acc);
}
