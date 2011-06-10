cmake_minimum_required (VERSION 2.6) 
project (vid.stab) 

SET(CMAKE_BUILTTYPE None)
 

set(TRANSCODE_ROOT ../../../transcode)

add_definitions( -Wall -DTRANSCODE)

# Make sure the compiler can find include files from transcode 
include_directories (${TRANSCODE_ROOT}/src ${TRANSCODE_ROOT}/) 
 
# Create the stabilize and transform libraries
add_library (filter_transform SHARED filter_transform.c transformtype.c libdeshake.c dslist.c) 
add_library (filter_stabilize SHARED filter_stabilize.c transformtype.c libdeshake.c motiondetect.c dslist.c) 

 
