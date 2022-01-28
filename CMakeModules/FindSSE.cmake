# Check if SSE instructions are available by the compiler and target platform (be aware of cross compilation)
include(CheckCCompilerFlag)

check_c_compiler_flag(-msse2 HAVE_SSE2)
check_c_compiler_flag(-msse3 HAVE_SSE3)
check_c_compiler_flag(-mssse3 HAVE_SSSE3)
check_c_compiler_flag(-msse4.1 HAVE_SSE4_1)

# Some compilers understand SSE flags, even when target platform doesn't support it (Clang with arm target)
# It is necessary try to compile actual code
if(HAVE_SSE2)
      try_compile(SSE_OK "${PROJECT_BINARY_DIR}"
          "${PROJECT_SOURCE_DIR}/CMakeModules/TestSSE2.c"
          COMPILE_DEFINITIONS "-msse2" )
      if(NOT SSE_OK)
            message(STATUS "SSE2 test compilation fails")
            set(HAVE_SSE2 FALSE)
      endif()
endif()

if(HAVE_SSE3)
      try_compile(SSE_OK "${PROJECT_BINARY_DIR}"
          "${PROJECT_SOURCE_DIR}/CMakeModules/TestSSE3.c"
          COMPILE_DEFINITIONS "-msse3" )
      if(NOT SSE_OK)
            message(STATUS "SSE3 test compilation fails")
            set(HAVE_SSE3 FALSE)
      endif()
endif()

if(HAVE_SSSE3)
      try_compile(SSE_OK "${PROJECT_BINARY_DIR}"
          "${PROJECT_SOURCE_DIR}/CMakeModules/TestSSSE3.c"
          COMPILE_DEFINITIONS "-mssse3" )
      if(NOT SSE_OK)
            message(STATUS "SSE3 test compilation fails")
            set(HAVE_SSSE3 FALSE)
      endif()
endif()

if(HAVE_SSE4_1)
      try_compile(SSE_OK "${PROJECT_BINARY_DIR}"
          "${PROJECT_SOURCE_DIR}/CMakeModules/TestSSE41.c"
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

if(NOT SSE2_FOUND)
      MESSAGE(STATUS "SSE2 is not supported on target platform.")
endif(NOT SSE2_FOUND)
if(NOT SSE3_FOUND)
      MESSAGE(STATUS "SSE3 is not supported on target platform.")
endif(NOT SSE3_FOUND)
if(NOT SSSE3_FOUND)
      MESSAGE(STATUS "SSSE3 is not supported on target platform.")
endif(NOT SSSE3_FOUND)
if(NOT SSE4_1_FOUND)
      MESSAGE(STATUS "SSE4.1 is not supported on target platform.")
endif(NOT SSE4_1_FOUND)

mark_as_advanced(SSE2_FOUND SSE3_FOUND SSSE3_FOUND SSE4_1_FOUND)
