/*
 * motiondetect_opt.c
 *
 *  Copyright (C) Georg Martius - February 1007-2012
 *   georg dot martius at web dot de
 *  Copyright (C) Alexey Osipov - Jule 2011
 *   simba at lerlan dot ru
 *   speed optimizations (threshold, spiral, SSE, asm)
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
#include "motiondetect_opt.h"

/* NOTE: ARM used to be served from this file by including sse2neon.h and
   #define'ing USE_SSE2 here.  That never worked: the #define was local to this
   translation unit, so motiondetect.c kept dispatching to the scalar C
   compareSubImg and never rounded the field size up to a multiple of 16, and
   the shimmed kernels below were dead code.  ARM now has native kernels in
   motiondetect_neon.c, reached through the runtime dispatcher. */

#ifdef VS_HAVE_SSE2
#include <emmintrin.h>
#endif

#ifdef VS_HAVE_SSE2
/**
   \see contrastSubImg using SSE2 optimization, Planar (1 byte per channel) only
*/
double contrastSubImg1_SSE(unsigned char* const I, const Field* field,
                           int linesize, int height)
{
  int k, j;
  unsigned char* p = NULL;
  int s2 = field->size / 2;

  static unsigned char full[16] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  p = I + (field->x - s2) + (field->y - s2)*linesize;

  __m128i mmin, mmax;

  mmin = _mm_loadu_si128((__m128i const*)full);
  mmax = _mm_setzero_si128();

  for (j = 0; j < field->size; j++){
    for (k = 0; k < field->size; k += 16) {
      __m128i xmm0;
      xmm0 = _mm_loadu_si128((__m128i const*)p);
      mmin = _mm_min_epu8(mmin, xmm0);
      mmax = _mm_max_epu8(mmax, xmm0);
      p += 16;
    }
    p += linesize - field->size;
  }

  __m128i xmm1;
  xmm1 = _mm_srli_si128(mmin, 8);
  mmin = _mm_min_epu8(mmin, xmm1);
  xmm1 = _mm_srli_si128(mmin, 4);
  mmin = _mm_min_epu8(mmin, xmm1);
  xmm1 = _mm_srli_si128(mmin, 2);
  mmin = _mm_min_epu8(mmin, xmm1);
  xmm1 = _mm_srli_si128(mmin, 1);
  mmin = _mm_min_epu8(mmin, xmm1);
  unsigned char mini = (unsigned char)_mm_extract_epi16(mmin, 0);

  xmm1 = _mm_srli_si128(mmax, 8);
  mmax = _mm_max_epu8(mmax, xmm1);
  xmm1 = _mm_srli_si128(mmax, 4);
  mmax = _mm_max_epu8(mmax, xmm1);
  xmm1 = _mm_srli_si128(mmax, 2);
  mmax = _mm_max_epu8(mmax, xmm1);
  xmm1 = _mm_srli_si128(mmax, 1);
  mmax = _mm_max_epu8(mmax, xmm1);
  unsigned char maxi = (unsigned char)_mm_extract_epi16(mmax, 0);

  return (maxi-mini)/(maxi+mini+0.1); // +0.1 to avoid division by 0
}
#endif

/// plain C implementation of variance based contrastSubImg
double contrastSubImg_variance_C(unsigned char* const I,
                                 const Field* field, int linesize, int height) {
  int k, j;
  unsigned char* p = NULL;
  unsigned char* pstart = NULL;
  int s2 = field->size / 2;
  unsigned int sum=0;
  int mean;
  int var=0;
  int numpixel = field->size*field->size;

  pstart = I + (field->x - s2) + (field->y - s2) * linesize;
  p = pstart;
  for (j = 0; j < field->size; j++) {
    for (k = 0; k < field->size; k++, p++) {
      sum+=*p;
    }
    p += linesize - field->size;
  }
  mean=sum/numpixel;
  p = pstart;
  for (j = 0; j < field->size; j++) {
    for (k = 0; k < field->size; k++, p++) {
      var+=abs(*p-mean);
    }
    p += linesize - field->size;
  }
  return (double)var/numpixel/255.0;
}

#ifdef VS_HAVE_SSE2
unsigned int compareSubImg_thr_sse2(unsigned char* const I1, unsigned char* const I2,
                                    const Field* field,
                                    int linesize1, int linesize2, int height,
                                    int bytesPerPixel, int d_x, int d_y,
                                    unsigned int treshold) {
  int k, j;
  unsigned char* p1 = NULL;
  unsigned char* p2 = NULL;
  int s2 = field->size / 2;
  int rowBytes = field->size * bytesPerPixel;
  unsigned int sum = 0;

  p1=I1 + (field->x - s2)*bytesPerPixel + (field->y - s2)*linesize1;
  p2=I2 + (field->x - s2 + d_x)*bytesPerPixel + (field->y - s2 + d_y)*linesize2;
  /* field->size is forced to a multiple of 16 (see vsMotionDetectInit), so a
     row is always a whole number of 16 byte blocks and needs no tail loop. */
  for (j = 0; j < field->size; j++){
    /* PSADBW sums |a-b| over each 8 byte half into the low 16 bits of the two
       64 bit lanes. A row totals at most field->size*bytesPerPixel*255, far
       below 2^32, so the 32 bit lane accumulator cannot overflow -- unlike the
       saturating 16 bit accumulator this replaces, which is the only reason
       that one had to be drained every 8 blocks. */
    __m128i xmmsum = _mm_setzero_si128();
    for (k = 0; k < rowBytes; k+=16){
      __m128i xmm0 = _mm_loadu_si128((__m128i const *)(p1 + k));
      __m128i xmm1 = _mm_loadu_si128((__m128i const *)(p2 + k));
      xmmsum = _mm_add_epi32(xmmsum, _mm_sad_epu8(xmm0, xmm1));
    }
    sum += (unsigned int)_mm_cvtsi128_si32(_mm_add_epi32(xmmsum,
                                                         _mm_srli_si128(xmmsum, 8)));

    if (sum > treshold) // no need to calculate any longer: worse than the best match
      break;
    p1 += linesize1;
    p2 += linesize2;
  }

  return sum;
}
#endif // VS_HAVE_SSE2

#ifdef USE_SSE2_ASM
unsigned int compareSubImg_thr_sse2_asm(unsigned char* const I1, unsigned char* const I2,
                                        const Field* field,
                                        int linesize1, int linesize2, int height,
                                        int bytesPerPixel, int d_x, int d_y,
                                        unsigned int treshold) {
  unsigned char* p1 = NULL;
  unsigned char* p2 = NULL;
  int s2 = field->size / 2;
  unsigned int sum = 0;

  static unsigned char mask[16] = {0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00};
  p1=I1 + (field->x - s2)*bytesPerPixel + (field->y - s2)*linesize1;
  p2=I2 + (field->x - s2 + d_x)*bytesPerPixel + (field->y - s2 + d_y)*linesize2;
  asm (
    "xor %0,%0\n"
    "pxor %%xmm4,%%xmm4\n"         //8 x 16bit partial sums
    "movdqu (%3),%%xmm3\n"         //mask

    //main loop
    "movl %4,%%edx\n"              //edx = field->size * bytesPerPixel / 16
    "mov $8,%%ecx\n"               //cx = 8
    "1:\n"

    //calc intermediate sum of abs differences for 16 bytes
    "movdqu (%1),%%xmm0\n"       //p1
    "movdqu (%2),%%xmm1\n"       //p2
    "movdqu %%xmm0,%%xmm2\n"     //xmm2 = xmm0
    "psubusb %%xmm1,%%xmm0\n"    //xmm0 = xmm0 - xmm1 (by bytes)
    "psubusb %%xmm2,%%xmm1\n"    //xmm1 = xmm1 - xmm2 (by bytes)
    "paddusb %%xmm1,%%xmm0\n"    //xmm0 = xmm0 + xmm1 (absolute difference)
    "movdqu %%xmm0,%%xmm2\n"     //xmm2 = xmm0
    "pand %%xmm3,%%xmm2\n"       //xmm2 = xmm2 & xmm3 (apply mask)
    "psrldq $1,%%xmm0\n"         //xmm0 = xmm0 >> 8 (shift by 1 byte)
    "pand %%xmm3,%%xmm0\n"       //xmm0 = xmm0 & xmm3 (apply mask)
    "paddusw %%xmm0,%%xmm4\n"    //xmm4 = xmm4 + xmm0 (by words)
    "paddusw %%xmm2,%%xmm4\n"    //xmm4 = xmm4 + xmm2 (by words)

    "add $16,%1\n"               //move to next 16 bytes (p1)
    "add $16,%2\n"               //move to next 16 bytes (p2)

    //check if we need flush sum (i.e. xmm4 is about to saturate)
    "dec %%ecx\n"
    "jnz 2f\n"                   //skip flushing if not
    //flushing...
    "movdqu %%xmm4,%%xmm0\n"
    "psrldq $8,%%xmm0\n"
    "paddusw %%xmm0,%%xmm4\n"
    "movdqu %%xmm4,%%xmm0\n"
    "psrldq $4,%%xmm0\n"
    "paddusw %%xmm0,%%xmm4\n"
    "movdqu %%xmm4,%%xmm0\n"
    "psrldq $2,%%xmm0\n"
    "paddusw %%xmm0,%%xmm4\n"
    "movd %%xmm4,%%ecx\n"
    "and $0xFFFF,%%ecx\n"
    "addl %%ecx,%0\n"
    "pxor %%xmm4,%%xmm4\n"       //clearing xmm4
    "mov $8,%%ecx\n"             //cx = 8

    //check if we need to go to another line
    "2:\n"
    "dec %%edx\n"
    "jnz 1b\n"                   //skip if not

    //move p1 and p2 to the next line
    "add %5,%1\n"
    "add %5,%2\n"
    "cmp %7,%0\n"                //if (sum > treshold)
    "ja 3f\n"                    //    break;
    "movl %4,%%edx\n"

    //check if all lines done
    "decl %6\n"
    "jnz 1b\n"                   //if not, continue looping
    "3:\n"
    :"=r"(sum)
    :"r"(p1),"r"(p2),"r"(mask),"g"(field->size * bytesPerPixel / 16),"g"((unsigned char*)(long)(linesize1 - field->size * bytesPerPixel)),"g"(field->size), "g"(treshold), "0"(sum)
    :"%xmm0","%xmm1","%xmm2","%xmm3","%xmm4","%ecx","%edx"
    );
  // TODO linesize2 is not properly used here
  return sum;
}
#endif // USE_SSE2_ASM

/*
 * Local variables:
 *   c-file-style: "stroustrup"
 *   c-file-offsets: ((case-label . *) (statement-case-intro . *))
 *   indent-tabs-mode: nil
 *   tab-width:  2
 *   c-basic-offset: 2 t
 * End:
 *
 * vim: expandtab shiftwidth=2:
 */
