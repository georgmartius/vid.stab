# to use intel compiler: CC=icc CXX=icpc cmake ../
#  very fast: faster than orc code at imgcompare without any options.
#  library needs libimf.so (link statically?)

cmake_minimum_required (VERSION 2.6) 
project (vid.stab) 

SET(CMAKE_BUILTTYPE None)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/../CMakeModules/")

set(TRANSCODE_ROOT ../../../transcode)

find_package(Orc REQUIRED) 

add_definitions( -Wall -O3 -msse2 -ffast-math -DTESTING ${ORC_DEFINITIONS})
#add_definitions(  -Wall -O0 -g  -DTESTING ${ORC_DEFINITIONS})

# Make sure the compiler can find include files from transcode 
include_directories (../src ${TRANSCODE_ROOT}/src ${TRANSCODE_ROOT} ${ORC_INCLUDE_DIRS}) 

add_executable (tests tests.c ../src/orc/motiondetectorc.c ../src/dslist.c ../src/transform.c ../src/transformfloat.c ../src/transformfixedpoint.c ../src/orc/transformorc.c ../src/libdeshake.c ../src/transformtype.c ../src/motiondetect.c)
target_link_libraries(tests imf m ${ORC_LIBRARIES}) #link the math library
 
