cmake_minimum_required (VERSION 2.6) 
project (vid.stab.transcode) 

SET(CMAKE_BUILTTYPE None)
 
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/../CMakeModules/")

# set your transcode path here!
set(TRANSCODE_ROOT ../../../transcode)

# we tried it with 0.4.14 
#  0.4.10 did not work (not all opcode implemented)
find_package(Orc REQUIRED) 

add_definitions( -Wall -O3 -Wno-pointer-sign -msse2 -ffast-math -DTRANSCODE ${ORC_DEFINITIONS})

# Make sure the compiler can find include files from transcode 
include_directories (${TRANSCODE_ROOT}/src ${TRANSCODE_ROOT}/ ${ORC_INCLUDE_DIRS}) 
include_directories (../src)
 
# Create the stabilize and transform libraries
add_library (filter_transform SHARED filter_transform.c 
  ../src/transformtype.c ../src/libdeshake.c ../src/transform.c 
  ../src/transformfixedpoint.c ../src/dslist.c) #  orc/transformorc.c) 
add_library (filter_stabilize SHARED filter_stabilize.c 
  ../src/transformtype.c ../src/libdeshake.c ../src/motiondetect.c 
  ../src/dslist.c ../src/orc/motiondetectorc.c) 

target_link_libraries(filter_stabilize m ${ORC_LIBRARIES}) #link the math library and orc
target_link_libraries(filter_transform m ) #link the math library
