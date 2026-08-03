/*
 *  motiondetect_avx2.c
 *
 *  AVX2 (32 byte vector) kernels for the motion detection inner loops.
 *
 *  This file is compiled with -mavx2 and must therefore never be entered
 *  unless vs_cpu_flags() reported VS_CPU_AVX2.  It contains no code that runs
 *  on the dispatch path itself.
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

#ifdef VS_HAVE_AVX2

#include <immintrin.h>

/* Rows to accumulate before testing the running sum against the threshold.
   1 means "every row", which makes these kernels return exactly the same value
   as compareSubImg_thr / _sse2 in every case, early exit included -- a much
   easier property to test than "greater than the threshold".  Coarser cadences
   are legal (the caller only ever uses the result as "error < minerror", see
   tests/test_simd_equivalence.c) and save a horizontal reduction per row, but
   measured only ~1.5% end to end at 1080p, which is not worth giving up the
   exact-equality guarantee for. */
#ifndef VS_AVX2_CHECK_ROWS
#define VS_AVX2_CHECK_ROWS 1
#endif

/* Horizontal sum of the four 64 bit lanes an accumulated _mm256_sad_epu8
   produces.  Each lane holds at most rows*32*255, so 64 bits cannot overflow
   and the result fits comfortably in 32 bits for any real field size. */
static inline unsigned int hsum_sad256(__m256i v) {
  __m128i lo = _mm256_castsi256_si128(v);
  __m128i hi = _mm256_extracti128_si256(v, 1);
  __m128i s  = _mm_add_epi64(lo, hi);
  s = _mm_add_epi64(s, _mm_unpackhi_epi64(s, s));
  return (unsigned int)_mm_cvtsi128_si32(s);
}

unsigned int compareSubImg_thr_avx2(unsigned char* const I1, unsigned char* const I2,
                                    const Field* field,
                                    int linesize1, int linesize2, int height,
                                    int bytesPerPixel, int d_x, int d_y,
                                    unsigned int treshold) {
  int j;
  int s2 = field->size / 2;
  int rowBytes = field->size * bytesPerPixel;
  unsigned int sum = 0;
  unsigned char* p1;
  unsigned char* p2;
  /* field->size is a multiple of 16 (vsMotionDetectInit), so rowBytes is too:
     a row is a whole number of 32 byte blocks plus at most one 16 byte tail. */
  int mainBytes = rowBytes & ~31;
  int hasTail   = rowBytes & 16;
  __m256i acc = _mm256_setzero_si256();

  p1 = I1 + (field->x - s2) * bytesPerPixel + (field->y - s2) * linesize1;
  p2 = I2 + (field->x - s2 + d_x) * bytesPerPixel + (field->y - s2 + d_y) * linesize2;

  for (j = 0; j < field->size; j++) {
    int k;
    for (k = 0; k < mainBytes; k += 32) {
      __m256i a = _mm256_loadu_si256((__m256i const*)(p1 + k));
      __m256i b = _mm256_loadu_si256((__m256i const*)(p2 + k));
      acc = _mm256_add_epi64(acc, _mm256_sad_epu8(a, b));
    }
    if (hasTail) {
      /* Widen the 16 byte remainder into the low half of a 256 bit vector so
         it can join the same accumulator.  _mm256_zextsi128_si256 would be the
         direct spelling but is missing from GCC before 10. */
      __m128i a = _mm_loadu_si128((__m128i const*)(p1 + mainBytes));
      __m128i b = _mm_loadu_si128((__m128i const*)(p2 + mainBytes));
      __m128i t = _mm_sad_epu8(a, b);
      acc = _mm256_add_epi64(acc, _mm256_inserti128_si256(_mm256_setzero_si256(), t, 0));
    }

    /* Early exit: this candidate is already worse than the best match so far.
       The contract (see tests/test_simd_equivalence.c) is only that the value
       returned by an early exit is greater than the threshold, not that it
       equals the full sum, so checking every VS_AVX2_CHECK_ROWS rows instead
       of every row is allowed and the argmin the caller computes is
       unchanged. */
    if ((j & (VS_AVX2_CHECK_ROWS - 1)) == (VS_AVX2_CHECK_ROWS - 1)) {
      sum = hsum_sad256(acc);
      if (sum > treshold)
        return sum;
    }
    p1 += linesize1;
    p2 += linesize2;
  }

  return hsum_sad256(acc);
}

double contrastSubImg1_avx2(unsigned char* const I, const Field* field,
                            int linesize, int height) {
  int j;
  int s2 = field->size / 2;
  int mainBytes = field->size & ~31;
  int hasTail   = field->size & 16;
  unsigned char* p = I + (field->x - s2) + (field->y - s2) * linesize;
  __m256i vmin = _mm256_set1_epi8((char)0xFF);
  __m256i vmax = _mm256_setzero_si256();
  __m128i lo, hi;
  unsigned char mini, maxi;

  for (j = 0; j < field->size; j++) {
    int k;
    for (k = 0; k < mainBytes; k += 32) {
      __m256i v = _mm256_loadu_si256((__m256i const*)(p + k));
      vmin = _mm256_min_epu8(vmin, v);
      vmax = _mm256_max_epu8(vmax, v);
    }
    if (hasTail) {
      /* Broadcast the 16 byte remainder to both halves: duplicating values is
         harmless for min/max and avoids needing a neutral filler. */
      __m128i t = _mm_loadu_si128((__m128i const*)(p + mainBytes));
      __m256i v = _mm256_broadcastsi128_si256(t);
      vmin = _mm256_min_epu8(vmin, v);
      vmax = _mm256_max_epu8(vmax, v);
    }
    p += linesize;
  }

  /* fold 256 -> 128 -> scalar */
  lo = _mm256_castsi256_si128(vmin);
  hi = _mm256_extracti128_si256(vmin, 1);
  lo = _mm_min_epu8(lo, hi);
  lo = _mm_min_epu8(lo, _mm_srli_si128(lo, 8));
  lo = _mm_min_epu8(lo, _mm_srli_si128(lo, 4));
  lo = _mm_min_epu8(lo, _mm_srli_si128(lo, 2));
  lo = _mm_min_epu8(lo, _mm_srli_si128(lo, 1));
  mini = (unsigned char)_mm_extract_epi16(lo, 0);

  lo = _mm256_castsi256_si128(vmax);
  hi = _mm256_extracti128_si256(vmax, 1);
  lo = _mm_max_epu8(lo, hi);
  lo = _mm_max_epu8(lo, _mm_srli_si128(lo, 8));
  lo = _mm_max_epu8(lo, _mm_srli_si128(lo, 4));
  lo = _mm_max_epu8(lo, _mm_srli_si128(lo, 2));
  lo = _mm_max_epu8(lo, _mm_srli_si128(lo, 1));
  maxi = (unsigned char)_mm_extract_epi16(lo, 0);

  return (maxi - mini) / (maxi + mini + 0.1); // +0.1 to avoid division by 0
}

#endif /* VS_HAVE_AVX2 */

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
