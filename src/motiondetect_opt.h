/*
 *  motiondetect_opt.h
 *
 *  Copyright (C) Georg Martius - February 2011
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

#ifndef MOTIONDETECT_OPT_H
#define MOTIONDETECT_OPT_H

#include "motiondetect.h"
#include "vidstab_export.h"
#include "cpudetect.h"

/* --- dispatch ---------------------------------------------------------------
   Which kernel to use is decided at *runtime* (see cpudetect.h), not by the
   preprocessor, so that a distribution binary built on any machine still uses
   AVX2 or AVX-512 where the host supports it.  The two hot kernels are
   therefore called through function pointers.

   Both pointers are statically initialised to the portable C implementations,
   so they are safe to call before vs_simd_init() has run; vs_simd_init() only
   ever upgrades them.  vsMotionDetectInit() calls it. */

typedef unsigned int (*vsCompareSubImgFn)(unsigned char* const I1,
                                          unsigned char* const I2,
                                          const Field* field,
                                          int linesize1, int linesize2, int height,
                                          int bytesPerPixel, int d_x, int d_y,
                                          unsigned int threshold);

/// \param linesize distance between two rows in BYTES (see vidstabdefines.h)
typedef double (*vsContrastSubImg1Fn)(unsigned char* const I, const Field* field,
                                      int linesize, int height);

/* Called as compareSubImg(...) / contrastSubImg1(...) exactly like the macros
   they replace. */
extern VS_API vsCompareSubImgFn   compareSubImg;
extern VS_API vsContrastSubImg1Fn contrastSubImg1;

/** Pick the best kernels for this machine.  Idempotent and cheap after the
    first call; safe to call from several threads. */
VS_API void vs_simd_init(void);

/** Name of the kernel family currently selected, e.g. "AVX2" or "scalar".
    Calls vs_simd_init() if that has not happened yet. */
VS_API const char* vs_simd_active_name(void);

/** True if the selected kernels require the field size to be a multiple of 16.
    All the SIMD kernels do, and the field size is rounded up unconditionally
    (see vsMotionDetectInit) so that a given input yields the same result on
    every machine regardless of which kernel runs. */
#define VS_SIMD_FIELD_ALIGNMENT 16

/* --- the individual kernels -------------------------------------------------
   Declared unconditionally so the dispatcher and the tests can name them; each
   is only *defined* when the corresponding VS_HAVE_* was set for the build. */

VS_API double contrastSubImg_variance_C(unsigned char* const I, const Field* field,
                        int linesize, int height);

#ifdef VS_HAVE_SSE2
VS_API double contrastSubImg1_SSE(unsigned char* const I, const Field* field,
                           int linesize, int height);
VS_API unsigned int compareSubImg_thr_sse2(unsigned char* const I1, unsigned char* const I2,
                                    const Field* field, int linesize1, int linesize2, int height,
                                    int bytesPerPixel, int d_x, int d_y,
                                    unsigned int threshold);
#endif

#ifdef VS_HAVE_AVX2
VS_API double contrastSubImg1_avx2(unsigned char* const I, const Field* field,
                                   int linesize, int height);
VS_API unsigned int compareSubImg_thr_avx2(unsigned char* const I1, unsigned char* const I2,
                                    const Field* field, int linesize1, int linesize2, int height,
                                    int bytesPerPixel, int d_x, int d_y,
                                    unsigned int threshold);
#endif

#ifdef VS_HAVE_AVX512
VS_API double contrastSubImg1_avx512(unsigned char* const I, const Field* field,
                                     int linesize, int height);
VS_API unsigned int compareSubImg_thr_avx512(unsigned char* const I1, unsigned char* const I2,
                                    const Field* field, int linesize1, int linesize2, int height,
                                    int bytesPerPixel, int d_x, int d_y,
                                    unsigned int threshold);
#endif

#ifdef VS_HAVE_NEON
VS_API double contrastSubImg1_neon(unsigned char* const I, const Field* field,
                                   int linesize, int height);
VS_API unsigned int compareSubImg_thr_neon(unsigned char* const I1, unsigned char* const I2,
                                    const Field* field, int linesize1, int linesize2, int height,
                                    int bytesPerPixel, int d_x, int d_y,
                                    unsigned int threshold);
#endif

#ifdef USE_SSE2_ASM
unsigned int compareSubImg_thr_sse2_asm(unsigned char* const I1, unsigned char* const I2,
                                        const Field* field, int linesize1, int linesize2,
                                        int height, int bytesPerPixel,
                                        int d_x, int d_y, unsigned int threshold);
#endif

#endif  /* MOTIONDETECT_OPT_H */

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
