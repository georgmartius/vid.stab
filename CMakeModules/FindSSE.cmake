include(CheckCCompilerFlag)

# SSE is an x86-only feature set, so detection must be done for the TARGET
# architecture, never for the build host.  When cross-compiling (e.g. from an
# x86_64 host to riscv64 or arm) any host-based or host-compiler-based probe
# yields a false positive and "-msse2" ends up on a compiler that rejects it.
# See https://github.com/georgmartius/vid.stab/issues/109
set(_SSE_TARGET_IS_X86 FALSE)
if(APPLE AND CMAKE_OSX_ARCHITECTURES)
      # Explicit (possibly universal) architecture list: only enable SSE if every
      # requested architecture is x86.
      set(_SSE_TARGET_IS_X86 TRUE)
      foreach(_sse_arch IN LISTS CMAKE_OSX_ARCHITECTURES)
            if(NOT _sse_arch MATCHES "^(x86_64h?|i[3-6]86)$")
                  set(_SSE_TARGET_IS_X86 FALSE)
            endif()
      endforeach()
      unset(_sse_arch)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^([xX]86(_64)?|[iI][3-6]86|AMD64|amd64|[eE][mM]64[tT])$")
      set(_SSE_TARGET_IS_X86 TRUE)
endif()

if(NOT _SSE_TARGET_IS_X86)
      if(CMAKE_CROSSCOMPILING)
            message(STATUS "Cross-compiling for non-x86 target "
                    "'${CMAKE_SYSTEM_PROCESSOR}': skipping SSE detection")
      else()
            message(STATUS "Non-x86 target '${CMAKE_SYSTEM_PROCESSOR}': "
                    "skipping SSE detection")
      endif()

      # Define the variables (as false) rather than leaving them unset, so that
      # if(SSE2_FOUND) in the callers behaves sanely.  No FORCE: an explicit
      # -DSSE2_FOUND=... from the user is preserved.
      set(SSE2_FOUND   FALSE CACHE BOOL "SSE2 available on target")
      set(SSE3_FOUND   FALSE CACHE BOOL "SSE3 available on target")
      set(SSSE3_FOUND  FALSE CACHE BOOL "SSSE3 available on target")
      set(SSE4_1_FOUND FALSE CACHE BOOL "SSE4.1 available on target")

elseif (MSVC)
      # SSE2 is part of the baseline every x86 Windows target is built for:
      # unconditionally so on x64, and the default /arch for 32 bit since
      # MSVC 2012.  Nothing beyond SSE2 may be requested here -- the SSE code
      # in this library is plain SSE2, and raising /arch would only make the
      # compiler emit newer instructions into ordinary C code and leave the
      # binaries refusing to start on CPUs that lack them.
      message(STATUS "MSVC detected, assuming SSE2 is available on the target")

      set(SSE2_FOUND TRUE CACHE BOOL "SSE2 available on target" FORCE)
      # Not baseline anywhere, and nothing in this project asks for them.
      set(SSE3_FOUND FALSE CACHE BOOL "SSE3 available on target" FORCE)
      set(SSSE3_FOUND FALSE CACHE BOOL "SSSE3 available on target" FORCE)
      set(SSE4_1_FOUND FALSE CACHE BOOL "SSE4.1 available on target" FORCE)

else()
      # GNU/Clang-like flow
      check_c_compiler_flag(-msse2 HAVE_SSE2)
      check_c_compiler_flag(-msse3 HAVE_SSE3)
      check_c_compiler_flag(-mssse3 HAVE_SSSE3)
      check_c_compiler_flag(-msse4.1 HAVE_SSE4_1)

      if(HAVE_SSE2)
            try_compile(SSE_OK "${PROJECT_BINARY_DIR}"
                    "${CMAKE_CURRENT_LIST_DIR}/TestSSE2.c"
                    COMPILE_DEFINITIONS "-msse2" )
            if(NOT SSE_OK)
                  message(STATUS "SSE2 test compilation fails")
                  set(HAVE_SSE2 FALSE)
            endif()
      endif()

      if(HAVE_SSE3)
            try_compile(SSE_OK "${PROJECT_BINARY_DIR}"
                    "${CMAKE_CURRENT_LIST_DIR}/TestSSE3.c"
                    COMPILE_DEFINITIONS "-msse3" )
            if(NOT SSE_OK)
                  message(STATUS "SSE3 test compilation fails")
                  set(HAVE_SSE3 FALSE)
            endif()
      endif()

      if(HAVE_SSSE3)
            try_compile(SSE_OK "${PROJECT_BINARY_DIR}"
                    "${CMAKE_CURRENT_LIST_DIR}/TestSSSE3.c"
                    COMPILE_DEFINITIONS "-mssse3" )
            if(NOT SSE_OK)
                  message(STATUS "SSSE3 test compilation fails")
                  set(HAVE_SSSE3 FALSE)
            endif()
      endif()

      if(HAVE_SSE4_1)
            try_compile(SSE_OK "${PROJECT_BINARY_DIR}"
                    "${CMAKE_CURRENT_LIST_DIR}/TestSSE41.c"
                    COMPILE_DEFINITIONS "-msse4.1" )
            if(NOT SSE_OK)
                  message(STATUS "SSE4.1 test compilation fails")
                  set(HAVE_SSE4_1 FALSE)
            endif()
      endif()

      set(SSE2_FOUND   ${HAVE_SSE2}   CACHE BOOL "SSE2 available on target")
      set(SSE3_FOUND   ${HAVE_SSE3}   CACHE BOOL "SSE3 available on target")
      set(SSSE3_FOUND  ${HAVE_SSSE3}  CACHE BOOL "SSSE3 available on target")
      set(SSE4_1_FOUND ${HAVE_SSE4_1} CACHE BOOL "SSE4.1 available on target")
endif()

# Output messages
if(NOT SSE2_FOUND)
      message(STATUS "SSE2 is not supported on target platform.")
endif()
if(NOT SSE3_FOUND)
      message(STATUS "SSE3 is not supported on target platform.")
endif()
if(NOT SSSE3_FOUND)
      message(STATUS "SSSE3 is not supported on target platform.")
endif()
if(NOT SSE4_1_FOUND)
      message(STATUS "SSE4.1 is not supported on target platform.")
endif()

mark_as_advanced(SSE2_FOUND SSE3_FOUND SSSE3_FOUND SSE4_1_FOUND)

unset(_SSE_TARGET_IS_X86)
