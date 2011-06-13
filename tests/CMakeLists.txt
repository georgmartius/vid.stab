cmake_minimum_required (VERSION 2.6) 
project (vid.stab) 

SET(CMAKE_BUILTTYPE None)
 

set(TRANSCODE_ROOT ../../../transcode)

add_definitions( -Wall)

# Make sure the compiler can find include files from transcode 
include_directories (../src ${TRANSCODE_ROOT}/src ${TRANSCODE_ROOT}/) 

add_executable (tests tests.c ../src/dslist.c ../src/transform.c ../src/libdeshake.c ../src/transformtype.c ../src/motiondetect.c)
target_link_libraries(tests m) #link the math library
 
