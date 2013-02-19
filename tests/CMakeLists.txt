# to use intel compiler: cd cmake; CC=icc CXX=icpc cmake ../
#  very fast: faster than orc code at imgcompare without any options.
#  library needs libimf.so (link statically?)

cmake_minimum_required (VERSION 2.6)
project (vid.stab)

SET(CMAKE_BUILTTYPE None)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/../CMakeModules/")

#add_definitions( -Wall -O3 -Wno-pointer-sign -DTESTING )
add_definitions(  -Wall -O0 -g -Wno-pointer-sign -DTESTING )
find_package(Orc)
if(ORC_FOUND)
add_definitions( -DUSE_ORC ${ORC_DEFINITIONS})
include_directories( ${ORC_INCLUDE_DIRS} )
else()
add_definitions( -DDISABLE_ORC)
endif()

add_definitions( -DUSE_SSE2 -msse2 -ffast-math -fno-show-column ) # -DUSE_SSE2_ASM

if(USE_OMP)
add_definitions(-fopenmp -DUSE_OMP)
endif()

# Make sure the compiler can find include files from transcode
include_directories (../src)

add_executable (tests tests.c testutils.c testframework.c ../src/dsvector.c
  ../src/transform.c ../src/transformfloat.c ../src/transformfixedpoint.c
  ../src/libdeshake.c ../src/transformtype.c ../src/frameinfo.c
	../src/serialize.c ../src/localmotion2transform.c
  ../src/motiondetect.c ../src/motiondetect_opt.c ../src/orc/motiondetectorc.c ../src/boxblur.c)

target_link_libraries(tests m)
if(ORC_FOUND)
target_link_libraries(tests ${ORC_LIBRARIES})
endif()
if(USE_OMP)
target_link_libraries(tests gomp)
endif()

