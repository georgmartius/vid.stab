# VidstabSimd.cmake
#
# Decides which SIMD kernels are compiled into the library and with which
# per-file flags.  Nothing here changes the flags of the *rest* of the library:
# the AVX2/AVX-512 kernels live in their own translation units and are only
# entered after vs_cpu_flags() has confirmed the host supports them, so the
# resulting binary still runs on a plain SSE2 machine.
#
# Sets, in the parent scope:
#   VIDSTAB_SIMD_SOURCES  - source files to add to the library
#   VIDSTAB_SIMD_DEFS     - VS_HAVE_* definitions for the whole library
#   VIDSTAB_SIMD_SUMMARY  - human readable list for the status message
# and attaches the required COMPILE_OPTIONS to the individual sources.

include(CheckCSourceCompiles)

function(vidstab_configure_simd SRCDIR)
  set(_sources "")
  set(_defs "")
  set(_summary "")

  # ---------------------------------------------------------------- x86 ---
  # SSE2_FOUND comes from FindSSE.cmake, which already refuses to answer yes
  # for a non-x86 target.
  if(SSE2_FOUND)
    list(APPEND _sources "${SRCDIR}/motiondetect_opt.c")
    list(APPEND _defs VS_HAVE_SSE2)
    list(APPEND _summary "SSE2")

    # SSE2 is baseline on x86_64 and the library has always required it where
    # available, so it stays a global flag (32 bit x86 needs it explicitly).
    if(NOT MSVC)
      set(VIDSTAB_SSE2_FLAG "-msse2" PARENT_SCOPE)
    endif()

    # -- AVX2 --
    if(MSVC)
      set(_avx2_flags /arch:AVX2)
    else()
      set(_avx2_flags -mavx2)
    endif()
    string(REPLACE ";" " " _avx2_flagstr "${_avx2_flags}")
    set(CMAKE_REQUIRED_FLAGS "${_avx2_flagstr}")
    check_c_source_compiles("
      #include <immintrin.h>
      int main(void){
        __m256i a = _mm256_set1_epi8(1), b = _mm256_set1_epi8(2);
        return (int)_mm256_extract_epi32(_mm256_sad_epu8(a,b), 0);
      }" VIDSTAB_HAVE_AVX2)
    unset(CMAKE_REQUIRED_FLAGS)
    if(VIDSTAB_HAVE_AVX2)
      list(APPEND _sources "${SRCDIR}/motiondetect_avx2.c")
      list(APPEND _defs VS_HAVE_AVX2)
      list(APPEND _summary "AVX2")
      set_source_files_properties("${SRCDIR}/motiondetect_avx2.c"
                                  PROPERTIES COMPILE_OPTIONS "${_avx2_flags}")
    endif()

    # -- AVX-512 (F + BW + VL) --
    if(MSVC)
      set(_avx512_flags /arch:AVX512)
    else()
      set(_avx512_flags -mavx512f -mavx512bw -mavx512vl)
    endif()
    # CMAKE_REQUIRED_FLAGS is a *string*: a CMake list would arrive at the
    # compiler semicolon separated and the probe would fail for the wrong
    # reason.  COMPILE_OPTIONS below does want the list form.
    string(REPLACE ";" " " _avx512_flagstr "${_avx512_flags}")
    set(CMAKE_REQUIRED_FLAGS "${_avx512_flagstr}")
    check_c_source_compiles("
      #include <immintrin.h>
      int main(void){
        __m512i a = _mm512_set1_epi8(1), b = _mm512_set1_epi8(2);
        __m512i s = _mm512_sad_epu8(a,b);
        __m512i m = _mm512_maskz_loadu_epi8((__mmask64)0xF, &a);
        s = _mm512_add_epi64(s, m);
        return (int)_mm512_reduce_add_epi64(s);
      }" VIDSTAB_HAVE_AVX512)
    unset(CMAKE_REQUIRED_FLAGS)
    if(VIDSTAB_HAVE_AVX512)
      list(APPEND _sources "${SRCDIR}/motiondetect_avx512.c")
      list(APPEND _defs VS_HAVE_AVX512)
      list(APPEND _summary "AVX-512")
      set_source_files_properties("${SRCDIR}/motiondetect_avx512.c"
                                  PROPERTIES COMPILE_OPTIONS "${_avx512_flags}")
    endif()
  endif()

  # ---------------------------------------------------------------- ARM ---
  # No flag is needed: NEON is mandatory on AArch64, and 32 bit ARM builds that
  # want it pass their own -mfpu=neon.  The probe is what decides.
  check_c_source_compiles("
    #include <arm_neon.h>
    int main(void){
      uint8x16_t a = vdupq_n_u8(1), b = vdupq_n_u8(2);
      uint16x8_t s = vpadalq_u8(vdupq_n_u16(0), vabdq_u8(a,b));
      return (int)vgetq_lane_u16(s, 0);
    }" VIDSTAB_HAVE_NEON)
  if(VIDSTAB_HAVE_NEON)
    list(APPEND _sources "${SRCDIR}/motiondetect_neon.c")
    list(APPEND _defs VS_HAVE_NEON)
    list(APPEND _summary "NEON")
  endif()

  # ------------------------------------------------------------- always ---
  list(APPEND _sources "${SRCDIR}/cpudetect.c" "${SRCDIR}/motiondetect_dispatch.c")
  # motiondetect_opt.c also holds contrastSubImg_variance_C, which the library
  # needs whether or not SSE2 was compiled in.
  if(NOT SSE2_FOUND)
    list(APPEND _sources "${SRCDIR}/motiondetect_opt.c")
  endif()

  if(NOT _summary)
    set(_summary "none (scalar C only)")
  endif()

  set(VIDSTAB_SIMD_SOURCES "${_sources}" PARENT_SCOPE)
  set(VIDSTAB_SIMD_DEFS    "${_defs}"    PARENT_SCOPE)
  set(VIDSTAB_SIMD_SUMMARY "${_summary}" PARENT_SCOPE)
endfunction()
