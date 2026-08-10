/*
 *  motiondetect_neon.c
 *
 *  ARM NEON (16 byte vector) kernels for the motion detection inner loops.
 *
 *  Native rather than shimmed from SSE2: PSADBW's "two 64 bit halves"
 *  semantics have no NEON equivalent, while vabdq/vpadal express the same
 *  reduction directly.
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

#ifdef VS_HAVE_NEON

/* The test suite compiles this file for x86 against a scalar emulation of the
   handful of NEON intrinsics used below, so that the logic is covered by the
   equivalence tests even off ARM hardware. */
#ifdef VS_NEON_EMULATION
#include "neon_emu.h"
#else
#include <arm_neon.h>
#endif

/* Rows to accumulate before testing the running sum against the threshold.
   1 means "every row", which makes these kernels return exactly the same value
   as compareSubImg_thr / _sse2, early exit included.  Coarser cadences are
   legal (the caller only uses the result as "error < minerror") but measured
   only ~1.5% end to end at 1080p, not worth giving up exact equality. */
#ifndef VS_NEON_CHECK_ROWS
#define VS_NEON_CHECK_ROWS 1
#endif

/* The across-vector reductions (vaddvq_/vminvq_/vmaxvq_) are AArch64 only;
   on 32 bit ARM they have to be built from pairwise folds. */
#if defined(__aarch64__) || defined(_M_ARM64) || defined(VS_NEON_EMULATION)

static inline unsigned int vs_haddq_u32(uint32x4_t v) { return vaddvq_u32(v); }
static inline unsigned char vs_hminq_u8(uint8x16_t v) { return vminvq_u8(v); }
static inline unsigned char vs_hmaxq_u8(uint8x16_t v) { return vmaxvq_u8(v); }

#else

static inline unsigned int vs_haddq_u32(uint32x4_t v) {
  uint32x2_t s = vadd_u32(vget_low_u32(v), vget_high_u32(v));
  return vget_lane_u32(vpadd_u32(s, s), 0);
}
static inline unsigned char vs_hminq_u8(uint8x16_t v) {
  uint8x8_t s = vmin_u8(vget_low_u8(v), vget_high_u8(v));
  s = vpmin_u8(s, s);
  s = vpmin_u8(s, s);
  s = vpmin_u8(s, s);
  return vget_lane_u8(s, 0);
}
static inline unsigned char vs_hmaxq_u8(uint8x16_t v) {
  uint8x8_t s = vmax_u8(vget_low_u8(v), vget_high_u8(v));
  s = vpmax_u8(s, s);
  s = vpmax_u8(s, s);
  s = vpmax_u8(s, s);
  return vget_lane_u8(s, 0);
}

#endif

/* Sum of absolute differences over a field.

   The accumulation is two staged: vabdq_u8 gives 16 byte-sized differences,
   vpadalq_u8 pairwise-adds them into 8 lanes of 16 bits, and only every 16th
   row are those folded into 32 bit lanes.  A 16 bit lane accumulates at most
   2*255 per row (two bytes pairwise added), so 16 rows reach 8160 and 64 rows
   would still fit -- draining every 16 rows leaves a wide margin and keeps the
   inner loop down to two instructions per 16 bytes. */
unsigned int compareSubImg_thr_neon(unsigned char* const I1, unsigned char* const I2,
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
  uint16x8_t acc16 = vdupq_n_u16(0);
  uint32x4_t acc32 = vdupq_n_u32(0);

  p1 = I1 + (field->x - s2) * bytesPerPixel + (field->y - s2) * linesize1;
  p2 = I2 + (field->x - s2 + d_x) * bytesPerPixel + (field->y - s2 + d_y) * linesize2;

  for (j = 0; j < field->size; j++) {
    int k;
    /* rowBytes is a multiple of 16 (vsMotionDetectInit rounds the field size
       up), so there is no tail. */
    for (k = 0; k < rowBytes; k += 16) {
      uint8x16_t a = vld1q_u8(p1 + k);
      uint8x16_t b = vld1q_u8(p2 + k);
      acc16 = vpadalq_u8(acc16, vabdq_u8(a, b));
    }

    if ((j & 15) == 15) {                 /* drain before 16 bit lanes fill up */
      acc32 = vpadalq_u16(acc32, acc16);
      acc16 = vdupq_n_u16(0);
    }

    if ((j & (VS_NEON_CHECK_ROWS - 1)) == (VS_NEON_CHECK_ROWS - 1)) {
      sum = vs_haddq_u32(vpadalq_u16(acc32, acc16));
      if (sum > treshold)
        return sum;
    }
    p1 += linesize1;
    p2 += linesize2;
  }

  return vs_haddq_u32(vpadalq_u16(acc32, acc16));
}

double contrastSubImg1_neon(unsigned char* const I, const Field* field,
                            int linesize, int height) {
  int j;
  int s2 = field->size / 2;
  unsigned char* p = I + (field->x - s2) + (field->y - s2) * linesize;
  uint8x16_t vmin = vdupq_n_u8(0xFF);
  uint8x16_t vmax = vdupq_n_u8(0x00);
  unsigned char mini, maxi;

  for (j = 0; j < field->size; j++) {
    int k;
    for (k = 0; k < field->size; k += 16) {
      uint8x16_t v = vld1q_u8(p + k);
      vmin = vminq_u8(vmin, v);
      vmax = vmaxq_u8(vmax, v);
    }
    p += linesize;
  }

  mini = vs_hminq_u8(vmin);
  maxi = vs_hmaxq_u8(vmax);

  return (maxi - mini) / (maxi + mini + 0.1); // +0.1 to avoid division by 0
}

#endif /* VS_HAVE_NEON */

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
