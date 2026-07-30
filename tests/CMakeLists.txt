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

option(USE_GLPK "use GLPK as the LP solver for the L1 optimal camera path" ON)
if(USE_GLPK)
  find_package(GLPK)
endif()
if(GLPK_FOUND)
  set(LP_SOURCES ../src/l1campathoptimization.c ../src/lpsolver_glpk.c)
  add_definitions(-DUSE_GLPK -DVS_HAVE_LPSOLVER)
endif()

add_executable (tests tests.c testutils.c testframework.c ../src/vsvector.c
  ../src/transform.c ../src/transformfloat.c ../src/transformfixedpoint.c
  ../src/libvidstab.c ../src/transformtype.c ../src/frameinfo.c
  ../src/serialize.c ../src/localmotion2transform.c
  ../src/motiondetect.c ../src/motiondetect_opt.c ../src/boxblur.c
  ${LP_SOURCES})

target_link_libraries(tests m)
if(GLPK_FOUND)
target_link_libraries(tests GLPK::GLPK)
endif()
if(USE_OMP)
target_link_libraries(tests gomp)
endif()
