/*
 *  cpudetect.c
 *
 *  Runtime detection of the SIMD instruction set extensions vid.stab can use.
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

#include "cpudetect.h"
#include "vidstabdefines.h"

#include <stdlib.h>
#include <string.h>

#if defined(VS_X86)
#  if defined(_MSC_VER)
#    include <intrin.h>
#  else
#    include <cpuid.h>
#  endif
#endif

/* ------------------------------------------------------------------ x86 --- */
#if defined(VS_X86)

static void vs_cpuid(unsigned int leaf, unsigned int subleaf,
                     unsigned int regs[4]) {
#if defined(_MSC_VER)
  int r[4];
  __cpuidex(r, (int)leaf, (int)subleaf);
  regs[0] = (unsigned int)r[0]; regs[1] = (unsigned int)r[1];
  regs[2] = (unsigned int)r[2]; regs[3] = (unsigned int)r[3];
#else
  unsigned int a = 0, b = 0, c = 0, d = 0;
  __cpuid_count(leaf, subleaf, a, b, c, d);
  regs[0] = a; regs[1] = b; regs[2] = c; regs[3] = d;
#endif
}

/* Contents of the extended control register.  A CPU may report AVX support
   while the OS has not enabled saving the wide registers on a context switch,
   in which case using them corrupts state -- XCR0 is the only way to tell. */
static unsigned long long vs_xgetbv0(void) {
#if defined(_MSC_VER)
  return _xgetbv(0);
#else
  unsigned int lo, hi;
  __asm__ volatile("xgetbv" : "=a"(lo), "=d"(hi) : "c"(0));
  return ((unsigned long long)hi << 32) | lo;
#endif
}

static unsigned int vs_cpu_detect(void) {
  unsigned int regs[4];
  unsigned int flags = 0;
  unsigned int maxleaf;

  vs_cpuid(0, 0, regs);
  maxleaf = regs[0];
  if (maxleaf < 1)
    return 0;

  vs_cpuid(1, 0, regs);
  if (regs[3] & (1u << 26))          /* EDX.SSE2 */
    flags |= VS_CPU_SSE2;

  /* Everything below needs the OS to have enabled XSAVE for the YMM/ZMM
     state, otherwise the registers are not preserved across context
     switches. */
  if (!(regs[2] & (1u << 27)))       /* ECX.OSXSAVE */
    return flags;
  {
    unsigned long long xcr0 = vs_xgetbv0();
    const unsigned long long ymm_state = 0x6;   /* XMM | YMM */
    const unsigned long long zmm_state = 0xE6;  /* + opmask | ZMM_hi256 | hi16_ZMM */

    if ((xcr0 & ymm_state) != ymm_state)
      return flags;

    if (maxleaf >= 7) {
      vs_cpuid(7, 0, regs);
      if (regs[1] & (1u << 5))       /* EBX.AVX2 */
        flags |= VS_CPU_AVX2;
      if ((xcr0 & zmm_state) == zmm_state) {
        /* The kernels need F (foundation), BW (byte/word ops, for the SAD and
           min/max on bytes) and VL (so the 256 bit tail can use masking). */
        if ((regs[1] & (1u << 16)) &&    /* AVX512F  */
            (regs[1] & (1u << 30)) &&    /* AVX512BW */
            (regs[1] & (1u << 31)))      /* AVX512VL */
          flags |= VS_CPU_AVX512;
      }
    }
  }
  return flags;
}

/* ------------------------------------------------------------------ ARM --- */
#elif defined(VS_ARM_NEON)

static unsigned int vs_cpu_detect(void) {
  /* NEON is architecturally mandatory on AArch64 and the 32 bit builds that
     reach here were compiled with a NEON enabled -mfpu, so if this file
     compiled at all the instructions are available.  There is no portable
     runtime probe (and unlike x86 there is nothing useful to fall back to). */
  return VS_CPU_NEON;
}

#else

static unsigned int vs_cpu_detect(void) {
  return 0;
}

#endif

/* --------------------------------------------------------------- common --- */

/* Which extensions this build actually contains kernels for.  Detection is
   masked with this so callers can dispatch on the result without #ifdefs. */
static unsigned int vs_cpu_compiled_in(void) {
  unsigned int f = 0;
#ifdef VS_HAVE_SSE2
  f |= VS_CPU_SSE2;
#endif
#ifdef VS_HAVE_AVX2
  f |= VS_CPU_AVX2;
#endif
#ifdef VS_HAVE_AVX512
  f |= VS_CPU_AVX512;
#endif
#ifdef VS_HAVE_NEON
  f |= VS_CPU_NEON;
#endif
  return f;
}

/* VIDSTAB_SIMD=<name> caps the detected flags at <name>, so a single machine
   can exercise (and benchmark) every kernel.  Returns a mask to AND with. */
static unsigned int vs_cpu_env_mask(void) {
  const char* e = getenv("VIDSTAB_SIMD");
  if (e == NULL || *e == '\0')
    return ~0u;

  if (strcmp(e, "none") == 0 || strcmp(e, "scalar") == 0)
    return 0;
  if (strcmp(e, "sse2") == 0)
    return VS_CPU_SSE2;
  if (strcmp(e, "avx2") == 0)
    return VS_CPU_SSE2 | VS_CPU_AVX2;
  if (strcmp(e, "avx512") == 0)
    return VS_CPU_SSE2 | VS_CPU_AVX2 | VS_CPU_AVX512;
  if (strcmp(e, "neon") == 0)
    return VS_CPU_NEON;

  vs_log_warn("vid.stab", "ignoring unrecognised VIDSTAB_SIMD=\"%s\" "
              "(expected none, sse2, avx2, avx512 or neon)\n", e);
  return ~0u;
}

/* Whether VIDSTAB_SIMD named a level (as opposed to being unset or invalid). */
static int vs_cpu_env_is_explicit(void) {
  const char* e = getenv("VIDSTAB_SIMD");
  if (e == NULL || *e == '\0')
    return 0;
  return strcmp(e, "none")   == 0 || strcmp(e, "scalar") == 0 ||
         strcmp(e, "sse2")   == 0 || strcmp(e, "avx2")   == 0 ||
         strcmp(e, "avx512") == 0 || strcmp(e, "neon")   == 0;
}

unsigned int vs_cpu_flags(void) {
  /* Benign race: concurrent first callers all compute the same value.  Worth
     avoiding a mutex for, since this is read on hot-ish paths. */
  static int done = 0;
  static unsigned int cached = 0;

  if (!done) {
    cached = vs_cpu_detect() & vs_cpu_compiled_in() & vs_cpu_env_mask();
    done = 1;
  }
  return cached;
}

int vs_cpu_simd_forced(void) {
  static int done = 0;
  static int cached = 0;

  if (!done) {
    cached = vs_cpu_env_is_explicit();
    done = 1;
  }
  return cached;
}

const char* vs_cpu_flags_name(unsigned int flags) {
  if (flags & VS_CPU_AVX512) return "AVX-512";
  if (flags & VS_CPU_AVX2)   return "AVX2";
  if (flags & VS_CPU_NEON)   return "NEON";
  if (flags & VS_CPU_SSE2)   return "SSE2";
  return "scalar";
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
