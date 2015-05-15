cmake_minimum_required (VERSION 2.6)
project (vid.stab)

SET(CMAKE_BUILTTYPE None)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMakeModules/")

include (FindSSE)

set(MAJOR_VERSION 1)
set(MINOR_VERSION 0)
set(PATCH_VERSION 0)
set(VIDSTAB_VERSION ${MAJOR_VERSION}.${MINOR_VERSION}${PATCH_VERSION})

option(BUILD_SHARED_LIBS "build shared libraries instead of static libraries"
       ON)

# determine lib suffix
get_property(LIB64 GLOBAL PROPERTY FIND_LIBRARY_USE_LIB64_PATHS)
if ("${LIB64}" STREQUAL "TRUE")
    set(LIBSUFFIX 64)
else()
    set(LIBSUFFIX "")
endif()

add_definitions( -Wall -O3 -g -Wno-pointer-sign -fPIC -std=gnu99)
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

set(SOURCES src/frameinfo.c src/transformtype.c src/libvidstab.c
  src/transform.c src/transformfixedpoint.c src/motiondetect.c
  src/motiondetect_opt.c src/serialize.c src/localmotion2transform.c
  src/boxblur.c src/vsvector.c src/orc/motiondetectorc.c)

set(HEADERS src/frameinfo.h src/transformtype.h src/libvidstab.h
  src/transform.h src/motiondetect.h src/serialize.h
  src/localmotion2transform.h src/boxblur.h src/vsvector.h )


# Create the vidstab library
add_library (vidstab ${SOURCES})

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
FILE(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h")
INSTALL(FILES ${HEADERS} DESTINATION include/vid.stab)
#endif()

INSTALL(TARGETS vidstab
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib${LIB_SUFFIX}
  ARCHIVE DESTINATION lib${LIB_SUFFIX}
)

include(create_pkgconfig_file)
create_pkgconfig_file(vidstab "Vid.Stab, a library for stabilizing video clips")
