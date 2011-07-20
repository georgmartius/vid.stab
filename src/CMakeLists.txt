cmake_minimum_required (VERSION 2.6) 
project (vid.stab) 

SET(CMAKE_BUILTTYPE None)
 
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/../CMakeModules/")

# we tried it with 0.4.14 
#  0.4.10 did not work (not all opcode implemented)
find_package(Orc REQUIRED) 

add_definitions( -Wall -O3 -msse2 -Wno-pointer-sign ${ORC_DEFINITIONS})

# Make sure the compiler can find include files from transcode 
include_directories (${ORC_INCLUDE_DIRS}) 
 
# Create the deshake library 
add_library (deshake STATIC
    transformtype.c libdeshake.c transform.c transformfixedpoint.c dslist.c transformtype.c
    libdeshake.c motiondetect.c dslist.c orc/motiondetectorc.c) 
#  orc/transformorc.c

target_link_libraries(deshake m ${ORC_LIBRARIES}) #link the math library and orc
