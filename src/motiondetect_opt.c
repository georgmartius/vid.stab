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
 *  the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <limits.h>

#include "motiondetect_opt.h"

#ifdef USE_ORC
#include "orc/motiondetectorc.h"
#endif

#ifdef USE_SSE2
#include <emmintrin.h>

#define USE_SSE2_CMP_HOR
#define SSE2_CMP_SUM_ROWS 8
#endif

unsigned int compareSubImg(unsigned char* const I1, unsigned char* const I2,
                                    const Field* field, int width1, int width2, int height,
                                    int bytesPerPixel, int d_x, int d_y,
                                    unsigned int threshold){

  int s2 = field->size / 2;

  // check for frame boundaries. if field is outside frame after translation, return INT_MAX
  if ((field->x - s2 + d_x + field->size > width2) || (field->y - s2 + d_y + field->size > height)){
    return INT_MAX;
  }

  return _compareSubImg(I1, I2, field, width1, width2, height, bytesPerPixel, d_x, d_y, threshold);
}

#ifdef USE_SSE2
/**
   \see contrastSubImg using SSE2 optimization, Planar (1 byte per channel) only
*/
double contrastSubImg1_SSE(unsigned char* const I, const Field* field,
                           int width, int height)
{
  int k, j;
  unsigned char* p = NULL;
  int s2 = field->size / 2;

  static unsigned char full[16] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  p = I + ((field->x - s2) + (field->y - s2)*width);

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
    p += (width - field->size);
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

#ifdef USE_ORC
/**
   calculates the contrast in the given small part of the given image
   using the absolute difference from mean luminance (like Root-Mean-Square,
   but with abs() (Manhattan-Norm))
   For multichannel images use contrastSubImg_Michelson()

   \param I pointer to framebuffer
   \param field Field specifies position(center) and size of subimage
   \param width width of frame
   \param height height of frame
*/
double contrastSubImg_variance_orc(unsigned char* const I, const Field* field,
                                   int width, int height) {
  unsigned char* p = NULL;
  int s2 = field->size / 2;
  int numpixel = field->size*field->size;

  p = I + ((field->x - s2) + (field->y - s2) * width);

  unsigned int sum=0;
  image_sum_optimized((signed int*)&sum, p, width, field->size, field->size);
  unsigned char mean = sum / numpixel;
  int var=0;
  image_variance_optimized(&var, p, width, mean, field->size, field->size);
  return (double)var/numpixel/255.0;
}

/// plain C implementation of variance based contrastSubImg (without ORC)
double contrastSubImg_variance_C(unsigned char* const I,
                                 const Field* field, int width, int height) {
  int k, j;
  unsigned char* p = NULL;
  unsigned char* pstart = NULL;
  int s2 = field->size / 2;
  unsigned int sum=0;
  int mean;
  int var=0;
  int numpixel = field->size*field->size;

  pstart = I + ((field->x - s2) + (field->y - s2) * width);
  p = pstart;
  for (j = 0; j < field->size; j++) {
    for (k = 0; k < field->size; k++, p++) {
      sum+=*p;
    }
    p += (width - field->size);
  }
  mean=sum/numpixel;
  p = pstart;
  for (j = 0; j < field->size; j++) {
    for (k = 0; k < field->size; k++, p++) {
      var+=abs(*p-mean);
    }
    p += (width - field->size);
  }
  return (double)var/numpixel/255.0;
}
#endif






#ifdef USE_ORC
/**
   compares a small part of two given images
   and returns the average absolute difference.
   Field center, size and shift have to be choosen,
   so that no clipping is required.
   Uses optimized inner loops by ORC.

   \param field Field specifies position(center) and size of subimage
   \param d_x shift in x direction
   \param d_y shift in y direction
*/
unsigned int compareSubImg_thr_orc(unsigned char* const I1, unsigned char* const I2,
                                   const Field* field, int width1, int width2, int height,
                                   int bytesPerPixel, int d_x, int d_y,
                                   unsigned int threshold) {
  unsigned char* p1 = NULL;
  unsigned char* p2 = NULL;
  int s2 = field->size / 2;
  int j;
  unsigned int sum = 0;
  p1 = I1 + ((field->x - s2) + (field->y - s2) * width1) * bytesPerPixel;
  p2 = I2 + ((field->x - s2 + d_x) + (field->y - s2 + d_y) * width2) * bytesPerPixel;

  for (j = 0; j < field->size; j++) {
    unsigned int s = 0;
    image_line_difference_optimized(&s, p1, p2, field->size* bytesPerPixel);
    sum += s;
    if( sum > threshold) // no need to calculate any longer: worse than the best match
      break;
    p1 += width1 * bytesPerPixel;
    p2 += width2 * bytesPerPixel;
  }


  return sum;
}

// implementation with 1 orc function, but no threshold
unsigned int compareSubImg_orc(unsigned char* const I1, unsigned char* const I2,
                               const Field* field, int width1, int width2, int height,
                               int bytesPerPixel, int d_x, int d_y,
                               unsigned int threshold) {
  unsigned char* p1 = NULL;
  unsigned char* p2 = NULL;
  int s2 = field->size / 2;
  unsigned int sum=0;
  p1 = I1 + ((field->x - s2) + (field->y - s2) * width1) * bytesPerPixel;
  p2 = I2 + ((field->x - s2 + d_x) + (field->y - s2 + d_y) * width2)
    * bytesPerPixel;

  image_difference_optimized(&sum, p1, width1 * bytesPerPixel, p2, width2 * bytesPerPixel,
                             field->size* bytesPerPixel , field->size);
  return sum;
}
#endif

#ifdef USE_SSE2
unsigned int compareSubImg_thr_sse2(unsigned char* const I1, unsigned char* const I2,
                                    const Field* field,
                                    int width1, int width2, int height,
                                    int bytesPerPixel, int d_x, int d_y,
                                    unsigned int treshold) {
  int k, j;
  unsigned char* p1 = NULL;
  unsigned char* p2 = NULL;
  int s2 = field->size / 2;
  unsigned int sum = 0;

  static unsigned char mask[16] = {0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00};
  unsigned char row = 0;
#ifndef USE_SSE2_CMP_HOR
  unsigned char summes[16];
  int i;
#endif
  __m128i xmmsum, xmmmask;
  xmmsum = _mm_setzero_si128();
  xmmmask = _mm_loadu_si128((__m128i const*)mask);

  p1=I1 + ((field->x - s2) + (field->y - s2)*width1)*bytesPerPixel;
  p2=I2 + ((field->x - s2 + d_x) + (field->y - s2 + d_y)*width2)*bytesPerPixel;
  for (j = 0; j < field->size; j++){
    for (k = 0; k < field->size * bytesPerPixel; k+=16){
      {
        __m128i xmm0, xmm1, xmm2;
        xmm0 = _mm_loadu_si128((__m128i const *)p1);
        xmm1 = _mm_loadu_si128((__m128i const *)p2);

        xmm2 = _mm_subs_epu8(xmm0, xmm1);
        xmm0 = _mm_subs_epu8(xmm1, xmm0);
        xmm0 = _mm_adds_epu8(xmm0, xmm2);

        xmm1 = _mm_and_si128(xmm0, xmmmask);
        xmm0 = _mm_srli_si128(xmm0, 1);
        xmm0 = _mm_and_si128(xmm0, xmmmask);

        xmmsum = _mm_adds_epu16(xmmsum, xmm0);
        xmmsum = _mm_adds_epu16(xmmsum, xmm1);
      }

      p1+=16;
      p2+=16;

      row++;
      if (row == SSE2_CMP_SUM_ROWS) {
        row = 0;
#ifdef USE_SSE2_CMP_HOR
        {
          __m128i xmm1;

          xmm1 = _mm_srli_si128(xmmsum, 8);
          xmmsum = _mm_adds_epu16(xmmsum, xmm1);

          xmm1 = _mm_srli_si128(xmmsum, 4);
          xmmsum = _mm_adds_epu16(xmmsum, xmm1);

          xmm1 = _mm_srli_si128(xmmsum, 2);
          xmmsum = _mm_adds_epu16(xmmsum, xmm1);

          sum += _mm_extract_epi16(xmmsum, 0);
        }
#else
        _mm_storeu_si128((__m128i*)summes, xmmsum);
        for(i = 0; i < 16; i+=2)
          sum += summes[i] + summes[i+1]*256;
#endif
        xmmsum = _mm_setzero_si128();
      }
    }
    if (sum > treshold)
      break;
    p1 += (width1 - field->size) * bytesPerPixel;
    p2 += (width2 - field->size) * bytesPerPixel;
  }

#if (SSE2_CMP_SUM_ROWS != 1) && (SSE2_CMP_SUM_ROWS != 2) && (SSE2_CMP_SUM_ROWS != 4) \
  && (SSE2_CMP_SUM_ROWS != 8) && (SSE2_CMP_SUM_ROWS != 16)
  //process all data left unprocessed
  //this part can be safely ignored if
  //SSE_SUM_ROWS = {1, 2, 4, 8, 16}
#ifdef USE_SSE2_CMP_HOR
  {
    __m128i xmm1;

    xmm1 = _mm_srli_si128(xmmsum, 8);
    xmmsum = _mm_adds_epu16(xmmsum, xmm1);

    xmm1 = _mm_srli_si128(xmmsum, 4);
    xmmsum = _mm_adds_epu16(xmmsum, xmm1);

    xmm1 = _mm_srli_si128(xmmsum, 2);
    xmmsum = _mm_adds_epu16(xmmsum, xmm1);

    sum += _mm_extract_epi16(xmmsum, 0);
  }
#else
  _mm_storeu_si128((__m128i*)summes, xmmsum);
  for(i = 0; i < 16; i+=2)
    sum += summes[i] + summes[i+1]*256;
#endif
#endif

  return sum;
}
#endif // USE_SSE2

#ifdef USE_SSE2_ASM
unsigned int compareSubImg_thr_sse2_asm(unsigned char* const I1, unsigned char* const I2,
                                        const Field* field,
                                        int width1, int width2, int height,
                                        int bytesPerPixel, int d_x, int d_y,
                                        unsigned int treshold) {
  unsigned char* p1 = NULL;
  unsigned char* p2 = NULL;
  int s2 = field->size / 2;
  unsigned int sum = 0;

  static unsigned char mask[16] = {0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00};
  p1=I1 + ((field->x - s2) + (field->y - s2)*width1)*bytesPerPixel;
  p2=I2 + ((field->x - s2 + d_x) + (field->y - s2 + d_y)*width2)*bytesPerPixel;
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
    :"r"(p1),"r"(p2),"r"(mask),"g"(field->size * bytesPerPixel / 16),"g"((unsigned char*)((width1 - field->size) * bytesPerPixel)),"g"(field->size), "g"(treshold), "0"(sum)
    :"%xmm0","%xmm1","%xmm2","%xmm3","%xmm4","%ecx","%edx"
    );
  // TODO width2 is not properly used here
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
