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
#ifdef VS_HAVE_SSE2
#include <emmintrin.h>
#endif


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

#ifdef VS_HAVE_SSE2
typedef struct { uint16_t mul; int shift; int valid; } VSReciprocal16;

static VSReciprocal16 vs_reciprocal16(int size, uint32_t acc_max){
  VSReciprocal16 r = { 0, 0, 0 };
  int s;
  if(size <= 0 || acc_max > 0xFFFFu) return r;
  for(s = 15; s >= 0; s--){
    uint64_t pow2 = (uint64_t)1 << (16 + s);
    uint64_t mul  = (pow2 + (uint64_t)size - 1) / (uint64_t)size;
    uint64_t e    = mul * (uint64_t)size - pow2;
    if(mul > 0xFFFFu) continue;
    if((uint64_t)acc_max * e >= pow2) continue;
    r.mul = (uint16_t)mul; r.shift = s; r.valid = 1;
    return r;
  }
  return r;
}
#endif

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
  const VSReciprocal rec = vs_reciprocal(size, vs_boxblur_accmax(size));
  const int can_split = (width > 2 * size2 + 1);

  if (rec.valid && can_split) {
    const uint32_t mul = rec.mul;
    const int shift = rec.shift;
    const int mid_end = width - size2 - 1;
    const int h4 = height & ~3;
#ifdef USE_OMP
#pragma omp parallel for schedule(static)
#endif
    for (j = 0; j < h4; j += 4) {
      int i, k;
      const unsigned char *s0 = src + (size_t)j * src_strive;
      const unsigned char *s1 = s0 + src_strive;
      const unsigned char *s2 = s1 + src_strive;
      const unsigned char *s3 = s2 + src_strive;
      const unsigned char *e0 = s0, *e1 = s1, *e2 = s2, *e3 = s3;
      unsigned char *d0 = dest + (size_t)j * dest_strive;
      unsigned char *d1 = d0 + dest_strive;
      unsigned char *d2 = d1 + dest_strive;
      unsigned char *d3 = d2 + dest_strive;
      uint32_t a0 = (uint32_t)(*s0) * (uint32_t)(size2 + 1);
      uint32_t a1 = (uint32_t)(*s1) * (uint32_t)(size2 + 1);
      uint32_t a2 = (uint32_t)(*s2) * (uint32_t)(size2 + 1);
      uint32_t a3 = (uint32_t)(*s3) * (uint32_t)(size2 + 1);
      for (k = 0; k < size2; k++) {
        a0 += *e0++; a1 += *e1++; a2 += *e2++; a3 += *e3++;
      }
      for (i = 0; i <= size2; i++) {
        a0 += *e0++ - *s0; d0[i] = (unsigned char)((a0 * mul) >> shift);
        a1 += *e1++ - *s1; d1[i] = (unsigned char)((a1 * mul) >> shift);
        a2 += *e2++ - *s2; d2[i] = (unsigned char)((a2 * mul) >> shift);
        a3 += *e3++ - *s3; d3[i] = (unsigned char)((a3 * mul) >> shift);
      }
      for (; i < mid_end; i++) {
        a0 += *e0++ - *s0++; d0[i] = (unsigned char)((a0 * mul) >> shift);
        a1 += *e1++ - *s1++; d1[i] = (unsigned char)((a1 * mul) >> shift);
        a2 += *e2++ - *s2++; d2[i] = (unsigned char)((a2 * mul) >> shift);
        a3 += *e3++ - *s3++; d3[i] = (unsigned char)((a3 * mul) >> shift);
      }
      for (; i < width; i++) {
        a0 += *e0 - *s0++; d0[i] = (unsigned char)((a0 * mul) >> shift);
        a1 += *e1 - *s1++; d1[i] = (unsigned char)((a1 * mul) >> shift);
        a2 += *e2 - *s2++; d2[i] = (unsigned char)((a2 * mul) >> shift);
        a3 += *e3 - *s3++; d3[i] = (unsigned char)((a3 * mul) >> shift);
      }
    }
    for (j = h4; j < height; j++) {
      int i, k;
      const unsigned char *s = src + (size_t)j * src_strive;
      const unsigned char *e = s;
      unsigned char *d = dest + (size_t)j * dest_strive;
      uint32_t a = (uint32_t)(*s) * (uint32_t)(size2 + 1);
      for (k = 0; k < size2; k++) a += *e++;
      for (i = 0; i <= size2; i++) {
        a += *e++ - *s; d[i] = (unsigned char)((a * mul) >> shift);
      }
      for (; i < mid_end; i++) {
        a += *e++ - *s++; d[i] = (unsigned char)((a * mul) >> shift);
      }
      for (; i < width; i++) {
        a += *e - *s++; d[i] = (unsigned char)((a * mul) >> shift);
      }
    }
    return;
  }

#ifdef USE_OMP
#pragma omp parallel for schedule(static)
#endif
  for(j=0; j< height; j++){
    int i,k;
    unsigned int acc;
    const unsigned char *start, *end;
    unsigned char *current;
    start = end = src + (size_t)j*src_strive;
    current = dest + (size_t)j*dest_strive;
    acc= (*start)*(size2+1);
    for(k=0; k<size2; k++){
      acc+=(*end);
      end++;
    }
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

void boxblur_vert_C(unsigned char* dest, const unsigned char* src,
        int width, int height, int dest_strive, int src_strive, int size){

  int size2 = size/2;
  const uint32_t acc_max = vs_boxblur_accmax(size);
  const VSReciprocal rec = vs_reciprocal(size, acc_max);
#ifdef VS_HAVE_SSE2
  const VSReciprocal16 rec16 = vs_reciprocal16(size, acc_max);
#endif

  if(width <= 0 || height <= 0)
    return;

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
      int len = c1 - c0;
      int ii, jj, kk;

#ifdef VS_HAVE_SSE2
      if (rec16.valid) {
        uint16_t acc16[512];
        const __m128i zero = _mm_setzero_si128();
        const __m128i vmul = _mm_set1_epi16((short)rec16.mul);
        const int shift16 = rec16.shift;
        const int len16 = len & ~15;

        for (ii = 0; ii < len; ii++)
          acc16[ii] = (uint16_t)((uint32_t)src[c0 + ii] * (uint32_t)(size2 + 1));
        for (kk = 0; kk < size2; kk++) {
          const unsigned char* row = src + (size_t)(kk < height ? kk : height - 1) * src_strive + c0;
          for (ii = 0; ii < len; ii++)
            acc16[ii] = (uint16_t)(acc16[ii] + row[ii]);
        }

        for (jj = 0; jj < height; jj++) {
          int er = jj + size2;     if (er > height - 1) er = height - 1;
          int sr = jj - size2 - 1; if (sr < 0)          sr = 0;
          const unsigned char* endRow   = src + (size_t)er * src_strive + c0;
          const unsigned char* startRow = src + (size_t)sr * src_strive + c0;
          unsigned char* out = dest + (size_t)jj * dest_strive + c0;

          for (ii = 0; ii < len16; ii += 16) {
            __m128i e8 = _mm_loadu_si128((const __m128i*)(endRow + ii));
            __m128i s8 = _mm_loadu_si128((const __m128i*)(startRow + ii));
            __m128i a_lo = _mm_loadu_si128((const __m128i*)(acc16 + ii));
            __m128i a_hi = _mm_loadu_si128((const __m128i*)(acc16 + ii + 8));

            a_lo = _mm_sub_epi16(_mm_add_epi16(a_lo, _mm_unpacklo_epi8(e8, zero)),
                                 _mm_unpacklo_epi8(s8, zero));
            a_hi = _mm_sub_epi16(_mm_add_epi16(a_hi, _mm_unpackhi_epi8(e8, zero)),
                                 _mm_unpackhi_epi8(s8, zero));

            _mm_storeu_si128((__m128i*)(acc16 + ii), a_lo);
            _mm_storeu_si128((__m128i*)(acc16 + ii + 8), a_hi);

            __m128i o_lo = _mm_srli_epi16(_mm_mulhi_epu16(a_lo, vmul), shift16);
            __m128i o_hi = _mm_srli_epi16(_mm_mulhi_epu16(a_hi, vmul), shift16);
            _mm_storeu_si128((__m128i*)(out + ii), _mm_packus_epi16(o_lo, o_hi));
          }
          for (; ii < len; ii++) {
            uint16_t a = (uint16_t)(acc16[ii] + endRow[ii] - startRow[ii]);
            acc16[ii] = a;
            out[ii] = (unsigned char)(((uint32_t)a * rec16.mul) >> (16 + shift16));
          }
        }
        continue;
      }
#endif
      {
        uint32_t acc[512];
        for(ii=0; ii<len; ii++)
          acc[ii] = (uint32_t)src[c0 + ii] * (uint32_t)(size2+1);
        for(kk=0; kk<size2; kk++){
          const unsigned char* row = src + (size_t)(kk < height ? kk : height-1)*src_strive + c0;
          for(ii=0; ii<len; ii++)
            acc[ii] += row[ii];
        }

        for(jj=0; jj<height; jj++){
          int er = jj + size2;          if(er > height-1) er = height-1;
          int sr = jj - size2 - 1;      if(sr < 0)        sr = 0;
          const unsigned char* endRow   = src + (size_t)er*src_strive + c0;
          const unsigned char* startRow = src + (size_t)sr*src_strive + c0;
          unsigned char* out = dest + (size_t)jj*dest_strive + c0;

          if(rec.valid){
            const uint32_t mul = rec.mul;
            const int shift = rec.shift;
            for(ii=0; ii<len; ii++){
              uint32_t a = acc[ii] + (uint32_t)endRow[ii] - (uint32_t)startRow[ii];
              acc[ii] = a;
              out[ii] = (unsigned char)((a * mul) >> shift);
            }
          }else{
            for(ii=0; ii<len; ii++){
              uint32_t a = acc[ii] + (uint32_t)endRow[ii] - (uint32_t)startRow[ii];
              acc[ii] = a;
              out[ii] = (unsigned char)(a/(uint32_t)size);
            }
          }
        }
      }
    }
  }
}
