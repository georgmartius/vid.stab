# to use intel compiler: cd cmake; CC=icc CXX=icpc cmake ../
#  very fast: faster than orc code at imgcompare without any options.
#  library needs libimf.so (link statically?)

cmake_minimum_required (VERSION 3.14)
project (vid.stab)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/../CMakeModules/")

include (FindSSE)
include (VidstabSimd)

option(USE_OMP "use parallelization use OMP" ON)

# Default to debug builds if no explicit build type specified.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Debug")
endif()

add_definitions(-Wall -Wno-pointer-sign -DTESTING -std=gnu99)

# Same SIMD setup as the library, so the equivalence tests check every kernel
# this toolchain can build -- not merely the one the dispatcher would pick.
vidstab_configure_simd("${CMAKE_CURRENT_SOURCE_DIR}/../src")
add_compile_definitions(${VIDSTAB_SIMD_DEFS})
message(STATUS "tests: SIMD kernels compiled in: ${VIDSTAB_SIMD_SUMMARY}")

# On a host without NEON, build the NEON kernel anyway against the scalar
# emulation of the intrinsics in neon_emu.h and put it through the same
# equivalence tests.  Without this the ARM kernel would be compiled by nobody
# on the machines where the tests actually run, and could rot unnoticed.
# It proves the logic, not the performance -- that still needs real hardware.
if(NOT VIDSTAB_HAVE_NEON)
  list(APPEND VIDSTAB_SIMD_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/../src/motiondetect_neon.c")
  # Both defines are global rather than per-source: motiondetect_neon.c needs
  # VS_NEON_EMULATION to pick up neon_emu.h, and test_simd_equivalence.c needs
  # it to know the kernel is plain C and therefore safe to call on any CPU.
  add_compile_definitions(VS_HAVE_NEON VS_NEON_EMULATION)
  message(STATUS "tests: also checking the NEON kernel via neon_emu.h")
endif()

if(SSE2_FOUND)
add_definitions( -msse2 -ffast-math -fno-show-column ) # -DUSE_SSE2_ASM
endif()

if(USE_OMP)
add_definitions(-fopenmp -DUSE_OMP)
endif()

# Make sure the compiler can find include files from transcode
include_directories (../src)
# neon_emu.h lives here, for the emulated NEON build below
include_directories (${CMAKE_CURRENT_SOURCE_DIR})

# --- LP solver backend for the L1 optimal camera path -------------------------
# The built-in interior point solver keeps vid.stab dependency free; GLPK can be
# selected instead and serves as the cross-check in the test suite.
set(VIDSTAB_LPSOLVER "builtin" CACHE STRING
    "LP solver for the L1 optimal camera path: builtin or glpk")
set_property(CACHE VIDSTAB_LPSOLVER PROPERTY STRINGS builtin glpk)

set(LP_SOURCES ../src/l1campathoptimization.c ../src/lpsolver_ipm.c)
set(LP_DEFS -DUSE_IPM -DVS_HAVE_LPSOLVER)
set(LP_BACKEND "built-in interior point")
if(VIDSTAB_LPSOLVER STREQUAL "glpk")
  find_package(GLPK)
  if(GLPK_FOUND)
    set(LP_SOURCES ../src/l1campathoptimization.c ../src/lpsolver_glpk.c)
    set(LP_DEFS -DUSE_GLPK -DVS_HAVE_LPSOLVER)
    set(LP_BACKEND "GLPK ${GLPK_VERSION}")
  else()
    message(WARNING "VIDSTAB_LPSOLVER=glpk but GLPK was not found, using the built-in solver")
  endif()
endif()
add_definitions(${LP_DEFS})
message(STATUS "L1 optimal camera path LP solver: ${LP_BACKEND}")

add_executable (tests tests.c testutils.c testframework.c ../src/vsvector.c
  ../src/transform.c ../src/transformfloat.c ../src/transformfixedpoint.c
  ../src/libvidstab.c ../src/transformtype.c ../src/frameinfo.c
  ../src/serialize.c ../src/localmotion2transform.c
  ../src/motiondetect.c ../src/boxblur.c
  ${VIDSTAB_SIMD_SOURCES}
  ${LP_SOURCES})

# The benchmark shares the test build's sources and SIMD configuration, so it
# can time every kernel compiled in rather than only the dispatched one.
# Informational: it asserts nothing and is not run by "make test".
add_executable (bench EXCLUDE_FROM_ALL ../bench/bench_motiondetect.c
  ../src/vsvector.c
  ../src/transform.c ../src/transformfloat.c ../src/transformfixedpoint.c
  ../src/libvidstab.c ../src/transformtype.c ../src/frameinfo.c
  ../src/serialize.c ../src/localmotion2transform.c
  ../src/motiondetect.c ../src/boxblur.c
  ${VIDSTAB_SIMD_SOURCES}
  ${LP_SOURCES})
target_link_libraries(bench m)
if(USE_OMP)
target_link_libraries(bench gomp)
endif()

target_link_libraries(tests m)
if(GLPK_FOUND AND VIDSTAB_LPSOLVER STREQUAL "glpk")
target_link_libraries(tests GLPK::GLPK)
endif()
if(USE_OMP)
target_link_libraries(tests gomp)
endif()

# Everything the tests write goes into this directory (TEST_OUTPUT_DIR in
# testutils.h, relative to the working directory the tests are run from, which
# for the usual in-build run is the build directory).  Have "make clean" take
# it with it.
set(TEST_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/testout")
if(CMAKE_VERSION VERSION_LESS 3.15)
  # deprecated in 3.15 in favour of ADDITIONAL_CLEAN_FILES, and Makefile only
  set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${TEST_OUTPUT_DIR}")
else()
  set_directory_properties(PROPERTIES ADDITIONAL_CLEAN_FILES "${TEST_OUTPUT_DIR}")
endif()
