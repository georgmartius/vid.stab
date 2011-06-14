cmake_minimum_required (VERSION 2.6) 
project (vid.stab) 

SET(CMAKE_BUILTTYPE None)
 
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/../CMakeModules/")
set(TRANSCODE_ROOT ../../../transcode)

find_package(Orc REQUIRED) 

add_definitions( -Wall -Wno-pointer-sign -DTRANSCODE ${ORC_DEFINITIONS})

# Make sure the compiler can find include files from transcode 
include_directories (${TRANSCODE_ROOT}/src ${TRANSCODE_ROOT}/ ${ORC_INCLUDE_DIRS}) 
 
# Create the stabilize and transform libraries
add_library (filter_transform SHARED filter_transform.c transformtype.c libdeshake.c transform.c dslist.c) 
add_library (filter_stabilize SHARED filter_stabilize.c transformtype.c libdeshake.c motiondetect.c dslist.c orc/motiondetectorc.c) 

target_link_libraries(filter_stabilize m ${ORC_LIBRARIES}) #link the math library
