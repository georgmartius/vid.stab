# to use intel compiler: cd cmake; CC=icc CXX=icpc cmake ../
#  very fast: faster than orc code at imgcompare without any options.
#  library needs libimf.so (link statically?)

cmake_minimum_required (VERSION 2.6)
project (vid.stab)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/../CMakeModules/")

include (FindSSE)

option(USE_OMP "use parallelization use OMP" ON)

# Default to debug builds if no explicit build type specified.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Debug")
endif()

add_definitions(-Wall -Wno-pointer-sign -DTESTING -std=gnu99)
find_package(Orc)
if(ORC_FOUND)
add_definitions( -DUSE_ORC ${ORC_DEFINITIONS})
include_directories( ${ORC_INCLUDE_DIRS} )
else()
add_definitions( -DDISABLE_ORC)
endif()

if(SSE2_FOUND)
add_definitions( -DUSE_SSE2 -msse2 -ffast-math -fno-show-column ) # -DUSE_SSE2_ASM
endif()

if(USE_OMP)
add_definitions(-fopenmp -DUSE_OMP)
endif()

# Make sure the compiler can find include files from transcode
include_directories (../src)

add_executable (tests tests.c testutils.c testframework.c ../src/vsvector.c
  ../src/transform.c ../src/transformfloat.c ../src/transformfixedpoint.c
  ../src/libvidstab.c ../src/transformtype.c ../src/frameinfo.c
  ../src/serialize.c ../src/localmotion2transform.c
  ../src/motiondetect.c ../src/motiondetect_opt.c ../src/orc/motiondetectorc.c ../src/boxblur.c)

target_link_libraries(tests m)
if(ORC_FOUND)
target_link_libraries(tests ${ORC_LIBRARIES})
endif()
if(USE_OMP)
target_link_libraries(tests gomp)
endif()
