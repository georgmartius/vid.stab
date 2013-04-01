cmake_minimum_required (VERSION 2.6)
project (vid.stab)

SET(CMAKE_BUILTTYPE None)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/../CMakeModules/")

include (FindSSE)

set(MAJOR_VERSION 0)
set(MINOR_VERSION 9)
set(PATCH_VERSION 6)
set(VIDSTAB_VERSION ${MAJOR_VERSION}.${MINOR_VERSION}${PATCH_VERSION})

add_definitions( -Wall -O3 -g -Wno-pointer-sign -fPIC )
# add_definitions(  -Wall -O0 -g -Wno-pointer-sign )

### ORC is not used in any active code at the moment  ###
# I tried it with 0.4.14
#  0.4.10 did not work (not all opcode implemented)
# find_package(Orc)
if(ORC_FOUND)
add_definitions( -DUSE_ORC ${ORC_DEFINITIONS} )
include_directories( ${ORC_INCLUDE_DIRS} )
else()
add_definitions( -DDISABLE_ORC )
endif()

# here we should check for SSE2
# our  -DUSE_SSE2_ASM code does not work with fpic
if(SSE2_FOUND)
add_definitions( -DUSE_SSE2 -msse2 -ffast-math )
endif()

set(SOURCES frameinfo.c transformtype.c libvidstab.c transform.c transformfixedpoint.c
  motiondetect.c motiondetect_opt.c serialize.c
	localmotion2transform.c boxblur.c vsvector.c orc/motiondetectorc.c)

set(HEADERS frameinfo.h transformtype.h libvidstab.h transform.h
  motiondetect.h serialize.h localmotion2transform.h boxblur.h vsvector.h )


# Create the vidstab library
# add_library (vidstab STATIC ${SOURCES})
add_library (vidstab SHARED ${SOURCES})

#set version of lib
set_target_properties(vidstab PROPERTIES SOVERSION ${MAJOR_VERSION}.${MINOR_VERSION})


target_link_libraries(vidstab m)
if(ORC_FOUND)
target_link_libraries(vidstab ${ORC_LIBRARIES})
endif()
if(USE_OMP)
target_link_libraries(vidstab gomp)
endif()


#if(!NOHEADERS)
FILE(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
INSTALL(FILES ${HEADERS} DESTINATION include/vid.stab)
#endif()

INSTALL(TARGETS vidstab
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

include(create_pkgconfig_file)
create_pkgconfig_file(vidstab "Vid.Stab, a library for stabilizing video clips")
