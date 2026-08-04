/*
 *  cpudetect.h
 *
 *  Runtime detection of the SIMD instruction set extensions vid.stab can use.
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

#ifndef VS_CPUDETECT_H
#define VS_CPUDETECT_H

#include "vidstab_export.h"

/* --- target architecture ---------------------------------------------------
   One place that decides what the *target* is, so no other file has to repeat
   the compiler specific predefined macro spellings.

   Note on ARM: GCC on AArch64 defines __ARM_NEON but not __ARM_NEON__, while
   Clang defines both.  Testing only the trailing-underscore spelling (as this
   library did until now) therefore silently disables NEON for every GCC
   AArch64 build.  Both spellings are checked here, plus __aarch64__/_M_ARM64,
   on which NEON is architecturally guaranteed. */
#if defined(__x86_64__) || defined(__i386__) || defined(_M_X64) || defined(_M_IX86)
#  define VS_X86 1
#elif defined(__aarch64__) || defined(_M_ARM64) || defined(__ARM_NEON) || defined(__ARM_NEON__)
#  define VS_ARM_NEON 1
#endif

/** Instruction set extensions vid.stab has hand written kernels for.
    The values are a bitmask; on x86 the AVX flags imply all lower ones. */
typedef enum {
  VS_CPU_NONE   = 0,
  VS_CPU_SSE2   = 1 << 0,  ///< x86, 16 byte vectors
  VS_CPU_AVX2   = 1 << 1,  ///< x86, 32 byte vectors
  VS_CPU_AVX512 = 1 << 2,  ///< x86, 64 byte vectors (needs AVX512F+BW+VL)
  VS_CPU_NEON   = 1 << 3   ///< ARM/AArch64, 16 byte vectors
} VSCpuFlags;

/** Returns the extensions usable on this machine, as a VSCpuFlags bitmask.

    Only extensions the library was actually compiled with support for are
    reported, so the result already answers "may I call this kernel?" and no
    caller needs a second #ifdef.  The answer is computed once and cached.

    The environment variable VIDSTAB_SIMD caps the result, which is what the
    test suite and the benchmark use to exercise every kernel on one machine:
      VIDSTAB_SIMD=none|sse2|avx2|avx512|neon
    An unrecognised value is ignored (with a warning). */
VS_API unsigned int vs_cpu_flags(void);

/** True if VIDSTAB_SIMD named a level explicitly, i.e. the selection is the
    caller's choice rather than the library's.  The dispatcher uses this to
    decide whether it may pick a kernel it would not choose on its own -- see
    the AVX-512 note in motiondetect_dispatch.c. */
VS_API int vs_cpu_simd_forced(void);

/** Human readable name of the highest extension in `flags`, e.g. "AVX2".
    Never returns NULL; "scalar" if no bit is set. */
VS_API const char* vs_cpu_flags_name(unsigned int flags);

#endif /* VS_CPUDETECT_H */

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
