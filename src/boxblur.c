/*
 *  boxblur.c
 *
 *  Copyright (C) Georg Martius - July 2010
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

#include "boxblur.h"
#include "vidstabdefines.h"


void boxblur_hori_C(unsigned char* dest, const unsigned char* src,
                    int width, int height, int dest_strive, int src_strive, int size);
void boxblur_vert_C(unsigned char* dest, const unsigned char* src,
                    int width, int height, int dest_strive, int src_strive, int size);

/* ---- magic-number reciprocal for the `acc/size` in the two passes ----------

   `size` is a runtime value, so `acc/size` compiles to a real integer division:
   ~20-30 cycles of latency in the serial horizontal chain, and -- worse -- it
   blocks the vertical pass's output loop from vectorizing at all, since no
   x86/NEON SIMD unit has an integer divide.

   Replaced by  (acc*mul) >> shift.  That is only worth doing if it is *exactly*
   equal to acc/size for every acc that can occur, so the constants are checked
   rather than assumed:

     mul   = ceil(2^shift / size),  e = mul*size - 2^shift  (0 <= e < size)
     acc*mul/2^shift = acc/size + acc*e/(size*2^shift)

   With acc = q*size + r the floor is q iff  r/size + acc*e/(size*2^shift) < 1,
   and the worst case r = size-1 leaves the condition

     acc_max * e < 2^shift                                   (exactness)

   plus, so the multiply stays a 32x32->32 one (vpmulld / vmulq_u32):

     acc_max * mul <= 2^32-1                                 (no overflow)

   The window holds 2*(size/2)+1 pixels, so acc_max = 255*(2*(size/2)+1), and
   the overflow bound alone caps shift at 24 (255*2^24 < 2^32).  Exactness then
   needs 255*size*(size-1) < 2^24, i.e. it holds for every odd size up to 265
   and fails first at 267 -- so this is *not* usable for arbitrary sizes, and
   the caller-visible range is not restricted here.  Instead the search below
   returns valid=0 when no shift works and the callers keep the division.
   (boxblurPlanar only ever passes stepSize-derived sizes, far inside the range.)
*/
typedef struct { uint32_t mul; int shift; int valid; } VSReciprocal;

static VSReciprocal vs_reciprocal(int size, uint32_t acc_max){
  VSReciprocal r = { 0, 0, 0 };
  int shift;
  if(size <= 0) return r;
  /* larger shift is better for exactness and worse for overflow, so walk down
     from the top and take the first shift that satisfies both */
  for(shift=31; shift>=0; shift--){
    uint64_t pow2 = (uint64_t)1 << shift;
    uint64_t mul  = (pow2 + (uint64_t)size - 1) / (uint64_t)size;   /* ceil */
    uint64_t e    = mul*(uint64_t)size - pow2;
    if(mul*(uint64_t)acc_max > 0xFFFFFFFFu) continue;               /* overflow */
    if((uint64_t)acc_max*e >= pow2)         continue;               /* not exact */
    r.mul = (uint32_t)mul; r.shift = shift; r.valid = 1;
    return r;
  }
  return r;
}

/* upper bound on the accumulator: the window is 2*(size/2)+1 pixels wide */
static uint32_t vs_boxblur_accmax(int size){
  return 255u * (uint32_t)(2*(size/2)+1);
}

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
  /* one integer division per pixel sits in the middle of the serial running
     sum here; the exact reciprocal removes it (see vs_reciprocal) */
  const VSReciprocal rec = vs_reciprocal(size, vs_boxblur_accmax(size));
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
    if(rec.valid){
      for(i=0; i< width; i++){
        acc = acc + (*end) - (*start);
        if(i > size2) start++;
        if(i < width - size2 - 1) end++;
        (*current) = (unsigned char)((acc * rec.mul) >> rec.shift);
        current++;
      }
    }else{
      for(i=0; i< width; i++){
        acc = acc + (*end) - (*start);
        if(i > size2) start++;
        if(i < width - size2 - 1) end++;
        (*current) = acc/size;
        current++;
      }
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
  /* without this the output loop below cannot vectorize at all: `acc[i]/size`
     has a runtime divisor and there is no SIMD integer divide (see
     vs_reciprocal for why this is exact) */
  const VSReciprocal rec = vs_reciprocal(size, vs_boxblur_accmax(size));

  if(width <= 0 || height <= 0)
    return;

  acc = (uint32_t*)vs_malloc((size_t)width * sizeof(uint32_t));
  if(acc == NULL){ // fall back to the in-place column walk rather than fail
    for(i=0; i< width; i++){
      const unsigned char *start, *end;
      unsigned char *current;
      uint32_t a;
      start = end = src + i;
      current = dest + i;
      a = (uint32_t)(*start)*(uint32_t)(size2+1);
      for(k=0; k<size2; k++){ a+=(*end); end+=src_strive; }
      for(j=0; j< height; j++){
        a = a - (*start) + (*end);
        if(j > size2) start+=src_strive;
        if(j < height - size2 - 1) end+=src_strive;
        *current = rec.valid ? (unsigned char)((a * rec.mul) >> rec.shift)
                             : (unsigned char)(a/(uint32_t)size);
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
        if(rec.valid){
          const uint32_t mul = rec.mul;
          const int shift = rec.shift;
          for(ii=c0; ii<c1; ii++)
            out[ii] = (unsigned char)((acc[ii] * mul) >> shift);
        }else{
          for(ii=c0; ii<c1; ii++)
            out[ii] = (unsigned char)(acc[ii]/size);
        }
      }
    }
  }

  vs_free(acc);
}
