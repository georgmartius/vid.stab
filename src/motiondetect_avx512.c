/*
 *  motiondetect_avx512.c
 *
 *  AVX-512 (64 byte vector) kernels for the motion detection inner loops.
 *  Needs AVX512F + AVX512BW (byte SAD and byte min/max) + AVX512VL (so the
 *  sub-64-byte remainder can be handled in a 128/256 bit register).
 *
 *  Compiled with -mavx512bw -mavx512vl; never enter unless vs_cpu_flags()
 *  reported VS_CPU_AVX512.
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

#ifdef VS_HAVE_AVX512

#include <immintrin.h>

/* Rows to accumulate before testing the running sum against the threshold.
   1 means "every row", which makes these kernels return exactly the same value
   as compareSubImg_thr / _sse2 in every case, early exit included -- a much
   easier property to test than "greater than the threshold".  Coarser cadences
   are legal (the caller only ever uses the result as "error < minerror", see
   tests/test_simd_equivalence.c) and save a horizontal reduction per row, but
   measured only ~1.5% end to end at 1080p, which is not worth giving up the
   exact-equality guarantee for. */
#ifndef VS_AVX512_CHECK_ROWS
#define VS_AVX512_CHECK_ROWS 1
#endif

/* Sum of the eight 64 bit lanes of an accumulated _mm512_sad_epu8. */
static inline unsigned int hsum_sad512(__m512i v) {
  return (unsigned int)_mm512_reduce_add_epi64(v);
}

static inline unsigned char hmin_epu8_512(__m512i v) {
  __m256i a = _mm256_min_epu8(_mm512_castsi512_si256(v),
                              _mm512_extracti64x4_epi64(v, 1));
  __m128i b = _mm_min_epu8(_mm256_castsi256_si128(a),
                           _mm256_extracti128_si256(a, 1));
  b = _mm_min_epu8(b, _mm_srli_si128(b, 8));
  b = _mm_min_epu8(b, _mm_srli_si128(b, 4));
  b = _mm_min_epu8(b, _mm_srli_si128(b, 2));
  b = _mm_min_epu8(b, _mm_srli_si128(b, 1));
  return (unsigned char)_mm_extract_epi16(b, 0);
}

static inline unsigned char hmax_epu8_512(__m512i v) {
  __m256i a = _mm256_max_epu8(_mm512_castsi512_si256(v),
                              _mm512_extracti64x4_epi64(v, 1));
  __m128i b = _mm_max_epu8(_mm256_castsi256_si128(a),
                           _mm256_extracti128_si256(a, 1));
  b = _mm_max_epu8(b, _mm_srli_si128(b, 8));
  b = _mm_max_epu8(b, _mm_srli_si128(b, 4));
  b = _mm_max_epu8(b, _mm_srli_si128(b, 2));
  b = _mm_max_epu8(b, _mm_srli_si128(b, 1));
  return (unsigned char)_mm_extract_epi16(b, 0);
}

/* The field rows are a multiple of 16 bytes but not necessarily of 64, so a
   row is split into 64 byte blocks plus a 16/32/48 byte remainder.  A mask
   load handles the remainder in one instruction instead of a ladder of
   narrower loads -- this is the main thing AVX-512 buys here beyond width. */
unsigned int compareSubImg_thr_avx512(unsigned char* const I1, unsigned char* const I2,
                                      const Field* field,
                                      int linesize1, int linesize2, int height,
                                      int bytesPerPixel, int d_x, int d_y,
                                      unsigned int treshold) {
  int j;
  int s2 = field->size / 2;
  int rowBytes = field->size * bytesPerPixel;
  int mainBytes = rowBytes & ~63;
  int tailBytes = rowBytes & 63;
  __mmask64 tailMask = tailBytes ? (__mmask64)((1ULL << tailBytes) - 1) : 0;
  unsigned int sum = 0;
  unsigned char* p1;
  unsigned char* p2;
  __m512i acc = _mm512_setzero_si512();

  p1 = I1 + (field->x - s2) * bytesPerPixel + (field->y - s2) * linesize1;
  p2 = I2 + (field->x - s2 + d_x) * bytesPerPixel + (field->y - s2 + d_y) * linesize2;

  for (j = 0; j < field->size; j++) {
    int k;
    for (k = 0; k < mainBytes; k += 64) {
      __m512i a = _mm512_loadu_si512((void const*)(p1 + k));
      __m512i b = _mm512_loadu_si512((void const*)(p2 + k));
      acc = _mm512_add_epi64(acc, _mm512_sad_epu8(a, b));
    }
    if (tailBytes) {
      /* Lanes outside the mask read as zero in both operands, so their |a-b|
         contribution is zero and the accumulator stays exact. */
      __m512i a = _mm512_maskz_loadu_epi8(tailMask, p1 + mainBytes);
      __m512i b = _mm512_maskz_loadu_epi8(tailMask, p2 + mainBytes);
      acc = _mm512_add_epi64(acc, _mm512_sad_epu8(a, b));
    }

    if ((j & (VS_AVX512_CHECK_ROWS - 1)) == (VS_AVX512_CHECK_ROWS - 1)) {
      sum = hsum_sad512(acc);
      if (sum > treshold)
        return sum;
    }
    p1 += linesize1;
    p2 += linesize2;
  }

  return hsum_sad512(acc);
}

double contrastSubImg1_avx512(unsigned char* const I, const Field* field,
                              int linesize, int height) {
  int j;
  int s2 = field->size / 2;
  int mainBytes = field->size & ~63;
  int tailBytes = field->size & 63;
  __mmask64 tailMask = tailBytes ? (__mmask64)((1ULL << tailBytes) - 1) : 0;
  unsigned char* p = I + (field->x - s2) + (field->y - s2) * linesize;
  __m512i vmin = _mm512_set1_epi8((char)0xFF);
  __m512i vmax = _mm512_setzero_si512();
  unsigned char mini, maxi;

  for (j = 0; j < field->size; j++) {
    int k;
    for (k = 0; k < mainBytes; k += 64) {
      __m512i v = _mm512_loadu_si512((void const*)(p + k));
      vmin = _mm512_min_epu8(vmin, v);
      vmax = _mm512_max_epu8(vmax, v);
    }
    if (tailBytes) {
      /* Merge-masked into the identity element for each reduction, so the
         lanes past the end of the row cannot affect the result. */
      __m512i v = _mm512_maskz_loadu_epi8(tailMask, p + mainBytes);
      vmax = _mm512_max_epu8(vmax, v);              /* zeros are neutral for max */
      vmin = _mm512_mask_min_epu8(vmin, tailMask, vmin, v);
    }
    p += linesize;
  }

  /* There is no _mm512_reduce_{min,max}_epu8 -- the reduce helpers only cover
     32/64 bit lanes -- so fold 512 -> 256 -> 128 and finish with a shuffle
     ladder. */
  mini = hmin_epu8_512(vmin);
  maxi = hmax_epu8_512(vmax);

  return (maxi - mini) / (maxi + mini + 0.1); // +0.1 to avoid division by 0
}

#endif /* VS_HAVE_AVX512 */

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
