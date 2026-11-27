/*
 *  motiondetect_avx2.c
 *
 *  AVX2 (32 byte vector) kernels for the motion detection inner loops.
 *
 *  This file is compiled with -mavx2 and must therefore never be entered
 *  unless vs_cpu_flags() reported VS_CPU_AVX2.  It contains no code that runs
 *  on the dispatch path itself.
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
static inline unsigned int hsum_sad256(__m256i v, __m128i tail) {
  __m128i lo = _mm256_castsi256_si128(v);
  __m128i hi = _mm256_extracti128_si256(v, 1);
  __m128i s  = _mm_add_epi64(_mm_add_epi64(lo, hi), tail);
  s = _mm_add_epi64(s, _mm_unpackhi_epi64(s, s));
  return (unsigned int)_mm_cvtsi128_si32(s);
}

unsigned int compareSubImg_thr_avx2(unsigned char* const I1, unsigned char* const I2,
                                    const Field* field,
                                    int linesize1, int linesize2, int height,
                                    int bytesPerPixel, int d_x, int d_y,
                                    unsigned int treshold) {
  int j;
  const int fsize = field->size;
  const int s2 = fsize / 2;
  const int rowBytes = fsize * bytesPerPixel;
  unsigned int sum = 0;
  const unsigned char* p1 = I1 + (field->x - s2) * bytesPerPixel + (field->y - s2) * linesize1;
  const unsigned char* p2 = I2 + (field->x - s2 + d_x) * bytesPerPixel + (field->y - s2 + d_y) * linesize2;

  /* Fast specialized path for 32-byte rows (e.g. 1080p fine fields):
     4 rows per iteration, checking threshold every 4 rows. */
  if (rowBytes == 32) {
    __m256i acc0 = _mm256_setzero_si256();
    __m256i acc1 = _mm256_setzero_si256();
    const int f4 = fsize & ~3;
    for (j = 0; j < f4; j += 4) {
      __m256i a0 = _mm256_loadu_si256((__m256i const*)(p1));
      __m256i b0 = _mm256_loadu_si256((__m256i const*)(p2));
      __m256i a1 = _mm256_loadu_si256((__m256i const*)(p1 + linesize1));
      __m256i b1 = _mm256_loadu_si256((__m256i const*)(p2 + linesize2));
      acc0 = _mm256_add_epi64(acc0, _mm256_sad_epu8(a0, b0));
      acc1 = _mm256_add_epi64(acc1, _mm256_sad_epu8(a1, b1));
      p1 += linesize1 * 2;
      p2 += linesize2 * 2;
      __m256i a2 = _mm256_loadu_si256((__m256i const*)(p1));
      __m256i b2 = _mm256_loadu_si256((__m256i const*)(p2));
      __m256i a3 = _mm256_loadu_si256((__m256i const*)(p1 + linesize1));
      __m256i b3 = _mm256_loadu_si256((__m256i const*)(p2 + linesize2));
      acc0 = _mm256_add_epi64(acc0, _mm256_sad_epu8(a2, b2));
      acc1 = _mm256_add_epi64(acc1, _mm256_sad_epu8(a3, b3));
      p1 += linesize1 * 2;
      p2 += linesize2 * 2;
      sum = hsum_sad256(_mm256_add_epi64(acc0, acc1), _mm_setzero_si128());
      if (sum > treshold)
        return sum;
    }
    __m256i acc = _mm256_add_epi64(acc0, acc1);
    for (; j < fsize; j++) {
      __m256i a = _mm256_loadu_si256((__m256i const*)(p1));
      __m256i b = _mm256_loadu_si256((__m256i const*)(p2));
      acc = _mm256_add_epi64(acc, _mm256_sad_epu8(a, b));
      p1 += linesize1;
      p2 += linesize2;
    }
    return hsum_sad256(acc, _mm_setzero_si128());
  }

  /* Fast specialized path for 48-byte rows (e.g. 4K fine fields):
     4 rows per iteration, dual accumulators, threshold check every 4 rows. */
  if (rowBytes == 48) {
    __m256i acc0 = _mm256_setzero_si256();
    __m256i acc1 = _mm256_setzero_si256();
    __m128i tacc0 = _mm_setzero_si128();
    __m128i tacc1 = _mm_setzero_si128();
    const int f4 = fsize & ~3;
    for (j = 0; j < f4; j += 4) {
      const unsigned char *q1 = p1 + linesize1, *q2 = p2 + linesize2;
      acc0  = _mm256_add_epi64(acc0, _mm256_sad_epu8(_mm256_loadu_si256((__m256i const*)p1),
                                                     _mm256_loadu_si256((__m256i const*)p2)));
      tacc0 = _mm_add_epi64(tacc0, _mm_sad_epu8(_mm_loadu_si128((__m128i const*)(p1 + 32)),
                                                _mm_loadu_si128((__m128i const*)(p2 + 32))));
      acc1  = _mm256_add_epi64(acc1, _mm256_sad_epu8(_mm256_loadu_si256((__m256i const*)q1),
                                                     _mm256_loadu_si256((__m256i const*)q2)));
      tacc1 = _mm_add_epi64(tacc1, _mm_sad_epu8(_mm_loadu_si128((__m128i const*)(q1 + 32)),
                                                _mm_loadu_si128((__m128i const*)(q2 + 32))));
      p1 += linesize1 * 2;
      p2 += linesize2 * 2;
      q1 = p1 + linesize1; q2 = p2 + linesize2;
      acc0  = _mm256_add_epi64(acc0, _mm256_sad_epu8(_mm256_loadu_si256((__m256i const*)p1),
                                                     _mm256_loadu_si256((__m256i const*)p2)));
      tacc0 = _mm_add_epi64(tacc0, _mm_sad_epu8(_mm_loadu_si128((__m128i const*)(p1 + 32)),
                                                _mm_loadu_si128((__m128i const*)(p2 + 32))));
      acc1  = _mm256_add_epi64(acc1, _mm256_sad_epu8(_mm256_loadu_si256((__m256i const*)q1),
                                                     _mm256_loadu_si256((__m256i const*)q2)));
      tacc1 = _mm_add_epi64(tacc1, _mm_sad_epu8(_mm_loadu_si128((__m128i const*)(q1 + 32)),
                                                _mm_loadu_si128((__m128i const*)(q2 + 32))));
      p1 += linesize1 * 2;
      p2 += linesize2 * 2;
      sum = hsum_sad256(_mm256_add_epi64(acc0, acc1), _mm_add_epi64(tacc0, tacc1));
      if (sum > treshold)
        return sum;
    }
    __m256i acc = _mm256_add_epi64(acc0, acc1);
    __m128i accTail = _mm_add_epi64(tacc0, tacc1);
    for (; j < fsize; j++) {
      acc     = _mm256_add_epi64(acc, _mm256_sad_epu8(_mm256_loadu_si256((__m256i const*)p1),
                                                      _mm256_loadu_si256((__m256i const*)p2)));
      accTail = _mm_add_epi64(accTail, _mm_sad_epu8(_mm_loadu_si128((__m128i const*)(p1 + 32)),
                                                    _mm_loadu_si128((__m128i const*)(p2 + 32))));
      p1 += linesize1;
      p2 += linesize2;
    }
    return hsum_sad256(acc, accTail);
  }

  /* Fast specialized path for 112-byte rows (1080p coarse fields: 3*32 + 16). */
  if (rowBytes == 112) {
    __m256i acc0 = _mm256_setzero_si256();
    __m256i acc1 = _mm256_setzero_si256();
    __m128i accTail = _mm_setzero_si128();
    for (j = 0; j < fsize; j++) {
      acc0 = _mm256_add_epi64(acc0, _mm256_sad_epu8(_mm256_loadu_si256((__m256i const*)(p1)),
                                                    _mm256_loadu_si256((__m256i const*)(p2))));
      acc1 = _mm256_add_epi64(acc1, _mm256_sad_epu8(_mm256_loadu_si256((__m256i const*)(p1 + 32)),
                                                    _mm256_loadu_si256((__m256i const*)(p2 + 32))));
      acc0 = _mm256_add_epi64(acc0, _mm256_sad_epu8(_mm256_loadu_si256((__m256i const*)(p1 + 64)),
                                                    _mm256_loadu_si256((__m256i const*)(p2 + 64))));
      accTail = _mm_add_epi64(accTail, _mm_sad_epu8(_mm_loadu_si128((__m128i const*)(p1 + 96)),
                                                    _mm_loadu_si128((__m128i const*)(p2 + 96))));
      p1 += linesize1;
      p2 += linesize2;
      if ((j & 1) == 1) {
        sum = hsum_sad256(_mm256_add_epi64(acc0, acc1), accTail);
        if (sum > treshold)
          return sum;
      }
    }
    return hsum_sad256(_mm256_add_epi64(acc0, acc1), accTail);
  }

  /* Fast specialized path for 224-byte rows (4K coarse fields: 7*32). */
  if (rowBytes == 224) {
    __m256i acc0 = _mm256_setzero_si256();
    __m256i acc1 = _mm256_setzero_si256();
    for (j = 0; j < fsize; j++) {
      acc0 = _mm256_add_epi64(acc0, _mm256_sad_epu8(_mm256_loadu_si256((__m256i const*)(p1)),
                                                    _mm256_loadu_si256((__m256i const*)(p2))));
      acc1 = _mm256_add_epi64(acc1, _mm256_sad_epu8(_mm256_loadu_si256((__m256i const*)(p1 + 32)),
                                                    _mm256_loadu_si256((__m256i const*)(p2 + 32))));
      acc0 = _mm256_add_epi64(acc0, _mm256_sad_epu8(_mm256_loadu_si256((__m256i const*)(p1 + 64)),
                                                    _mm256_loadu_si256((__m256i const*)(p2 + 64))));
      acc1 = _mm256_add_epi64(acc1, _mm256_sad_epu8(_mm256_loadu_si256((__m256i const*)(p1 + 96)),
                                                    _mm256_loadu_si256((__m256i const*)(p2 + 96))));
      acc0 = _mm256_add_epi64(acc0, _mm256_sad_epu8(_mm256_loadu_si256((__m256i const*)(p1 + 128)),
                                                    _mm256_loadu_si256((__m256i const*)(p2 + 128))));
      acc1 = _mm256_add_epi64(acc1, _mm256_sad_epu8(_mm256_loadu_si256((__m256i const*)(p1 + 160)),
                                                    _mm256_loadu_si256((__m256i const*)(p2 + 160))));
      acc0 = _mm256_add_epi64(acc0, _mm256_sad_epu8(_mm256_loadu_si256((__m256i const*)(p1 + 192)),
                                                    _mm256_loadu_si256((__m256i const*)(p2 + 192))));
      p1 += linesize1;
      p2 += linesize2;
      if ((j & 1) == 1) {
        sum = hsum_sad256(_mm256_add_epi64(acc0, acc1), _mm_setzero_si128());
        if (sum > treshold)
          return sum;
      }
    }
    return hsum_sad256(_mm256_add_epi64(acc0, acc1), _mm_setzero_si128());
  }

  /* General path: 64-byte unrolled dual-accumulator loop + optional 32B/16B tail. */
  {
    const int main64    = rowBytes & ~63;
    const int has32     = rowBytes & 32;
    const int has16     = rowBytes & 16;
    const int off16     = rowBytes & ~31;
    __m256i acc0 = _mm256_setzero_si256();
    __m256i acc1 = _mm256_setzero_si256();
    __m128i accTail = _mm_setzero_si128();

    for (j = 0; j < fsize; j++) {
      int k;
      for (k = 0; k < main64; k += 64) {
        __m256i a0 = _mm256_loadu_si256((__m256i const*)(p1 + k));
        __m256i b0 = _mm256_loadu_si256((__m256i const*)(p2 + k));
        __m256i a1 = _mm256_loadu_si256((__m256i const*)(p1 + k + 32));
        __m256i b1 = _mm256_loadu_si256((__m256i const*)(p2 + k + 32));
        acc0 = _mm256_add_epi64(acc0, _mm256_sad_epu8(a0, b0));
        acc1 = _mm256_add_epi64(acc1, _mm256_sad_epu8(a1, b1));
      }
      if (has32) {
        __m256i a = _mm256_loadu_si256((__m256i const*)(p1 + main64));
        __m256i b = _mm256_loadu_si256((__m256i const*)(p2 + main64));
        acc0 = _mm256_add_epi64(acc0, _mm256_sad_epu8(a, b));
      }
      if (has16) {
        __m128i a = _mm_loadu_si128((__m128i const*)(p1 + off16));
        __m128i b = _mm_loadu_si128((__m128i const*)(p2 + off16));
        accTail = _mm_add_epi64(accTail, _mm_sad_epu8(a, b));
      }

      if ((j & 1) == 1) {
        sum = hsum_sad256(_mm256_add_epi64(acc0, acc1), accTail);
        if (sum > treshold)
          return sum;
      }
      p1 += linesize1;
      p2 += linesize2;
    }
    return hsum_sad256(_mm256_add_epi64(acc0, acc1), accTail);
  }
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
