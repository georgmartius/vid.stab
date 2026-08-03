# to use intel compiler: cd cmake; CC=icc CXX=icpc cmake ../
#  very fast: faster than orc code at imgcompare without any options.
#  library needs libimf.so (link statically?)

cmake_minimum_required (VERSION 3.14)
project (vid.stab)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/../CMakeModules/")

include (FindSSE)

option(USE_OMP "use parallelization use OMP" ON)

# Default to debug builds if no explicit build type specified.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Debug")
endif()

add_definitions(-Wall -Wno-pointer-sign -DTESTING -std=gnu99)

if(SSE2_FOUND)
add_definitions( -DUSE_SSE2 -msse2 -ffast-math -fno-show-column ) # -DUSE_SSE2_ASM
endif()

if(USE_OMP)
add_definitions(-fopenmp -DUSE_OMP)
endif()

# Make sure the compiler can find include files from transcode
include_directories (../src)

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
  ../src/motiondetect.c ../src/motiondetect_opt.c ../src/boxblur.c
  ${LP_SOURCES})

target_link_libraries(tests m)
if(GLPK_FOUND AND VIDSTAB_LPSOLVER STREQUAL "glpk")
target_link_libraries(tests GLPK::GLPK)
endif()
if(USE_OMP)
target_link_libraries(tests gomp)
endif()
