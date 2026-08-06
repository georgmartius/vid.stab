/*
 *  motiondetect_dispatch.c
 *
 *  Runtime selection of the motion detection SIMD kernels.
 *
 *  Kept in its own translation unit, compiled with only baseline compiler
 *  flags: the files holding the AVX2/AVX-512/NEON kernels are compiled with
 *  -mavx2 and friends, which lets the compiler emit those instructions
 *  anywhere in the file -- including in code that would run before the
 *  corresponding check.  Nothing here may be built with such a flag.
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
#include "motiondetect_internal.h"
#include "vidstabdefines.h"

/* contrastSubImg1() is contrastSubImg() specialised to bytesPerPixel == 1;
   the planar path is the only caller and every SIMD kernel is planar only. */
static double contrastSubImg1_C(unsigned char* const I, const Field* field,
                                int linesize, int height) {
  return contrastSubImg(I, field, linesize, height, 1);
}

/* Safe defaults: correct everywhere, upgraded by vs_simd_init(). */
vsCompareSubImgFn   compareSubImg   = compareSubImg_thr;
vsContrastSubImg1Fn contrastSubImg1 = contrastSubImg1_C;

/* What vs_simd_init() actually picked.  This is not the same as the highest
   extension the CPU reports (see the AVX-512 note below), so it is recorded
   here rather than derived from vs_cpu_flags(). */
static const char* vs_simd_selected = "scalar";

void vs_simd_init(void) {
  static int done = 0;
  unsigned int flags;

  /* Benign race: every racing caller stores the same pointers. */
  if (done)
    return;

  flags = vs_cpu_flags();
  (void)flags;   /* unused in a build with no SIMD kernels at all */

#ifdef VS_HAVE_SSE2
  if (flags & VS_CPU_SSE2) {
    compareSubImg   = compareSubImg_thr_sse2;
    contrastSubImg1 = contrastSubImg1_SSE;
    vs_simd_selected = "SSE2";
  }
#endif
#ifdef VS_HAVE_NEON
  if (flags & VS_CPU_NEON) {
    compareSubImg   = compareSubImg_thr_neon;
    contrastSubImg1 = contrastSubImg1_neon;
    vs_simd_selected = "NEON";
  }
#endif
#ifdef VS_HAVE_AVX2
  if (flags & VS_CPU_AVX2) {
    compareSubImg   = compareSubImg_thr_avx2;
    contrastSubImg1 = contrastSubImg1_avx2;
    vs_simd_selected = "AVX2";
  }
#endif
#ifdef VS_HAVE_AVX512
  /* AVX-512 is deliberately NOT preferred over AVX2 by default.

     These kernels are memory bound rather than compute bound, so the wider
     vectors buy little, while issuing 512 bit instructions drops the core
     clock on the parts that implement AVX-512 frequency licensing (Skylake-X
     and the other Xeon/HEDT generations of that era).  The slowdown hits the
     *whole* frame, including the scalar and 256 bit code around the kernel.
     Measured on an i7-7800X at 1080p, AVX-512 came out ~18% slower than AVX2
     both single threaded and on 9 threads -- see
     docs/superpowers/simd-optimization-report.md.

     Newer cores (Ice Lake and later, Zen 4/5) do not throttle this way and may
     well come out ahead, but there is no reliable runtime way to ask "does
     512 bit width cost me frequency here", and guessing from the family/model
     would need a lookup table that ages badly.  So the kernels are built and
     tested, and reachable with VIDSTAB_SIMD=avx512 for anyone who measures a
     win on their hardware, but the automatic choice stays AVX2. */
  if ((flags & VS_CPU_AVX512) && vs_cpu_simd_forced()) {
    compareSubImg   = compareSubImg_thr_avx512;
    contrastSubImg1 = contrastSubImg1_avx512;
    vs_simd_selected = "AVX-512";
  }
#endif

  /* USE_SSE2_ASM is the historical hand written assembly variant.  It is not
     part of the runtime dispatch: it ignores linesize2 (see the TODO in
     motiondetect_opt.c) and so is only correct when both frames share a
     stride.  Left reachable only through an explicit opt in build. */
#ifdef USE_SSE2_ASM
  if (flags & VS_CPU_SSE2) {
    compareSubImg = compareSubImg_thr_sse2_asm;
    vs_simd_selected = "SSE2 (asm)";
  }
#endif

  done = 1;
}

const char* vs_simd_active_name(void) {
  vs_simd_init();
  return vs_simd_selected;
}

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
