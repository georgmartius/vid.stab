cmake_minimum_required (VERSION 2.6)
project (vid.stab)

SET(CMAKE_BUILTTYPE None)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/../CMakeModules/")

#add_definitions( -Wall -O3 -Wno-pointer-sign -DTESTING )
#add_definitions( -Wall -O3 -Wno-pointer-sign)
add_definitions(  -Wall -O0 -g -Wno-pointer-sign )
# I tried it with 0.4.14
#  0.4.10 did not work (not all opcode implemented)
# find_package(Orc)  // it actually not used by any active code
if(ORC_FOUND)
add_definitions( -DUSE_ORC ${ORC_DEFINITIONS} )
include_directories( ${ORC_INCLUDE_DIRS} )
else()
add_definitions( -DDISABLE_ORC )
endif()

# here we should check for SSE2
# our  -DUSE_SSE2_ASM code does not work with fpic
add_definitions( -DUSE_SSE2 -msse2 -ffast-math )

# Create the deshake library
add_library (deshake STATIC
    transformtype.c libdeshake.c transform.c transformfixedpoint.c
    motiondetect.c motiondetect_opt.c serialize.c localmotion2transform.c
		boxblur.c dsvector.c orc/motiondetectorc.c)
#  orc/transformorc.c

target_link_libraries(deshake m)
if(ORC_FOUND)
target_link_libraries(deshake ${ORC_LIBRARIES})
endif()
if(USE_OMP)
target_link_libraries(deshake gomp)
endif()
