cmake_minimum_required (VERSION 2.8.5)
project (vid.stab C)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMakeModules/")

include (FindSSE)
include (GNUInstallDirs)
# include (ExternalProject) # There must be a nicer way to include this
# include (sse2neon)
find_package(OpenMP)

set(MAJOR_VERSION 1)
set(MINOR_VERSION 2)
set(PATCH_VERSION 0)
set(VIDSTAB_VERSION ${MAJOR_VERSION}.${MINOR_VERSION}${PATCH_VERSION})

# Default to release builds if no explicit build type specified.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release")
endif()

option(BUILD_SHARED_LIBS "build shared libraries instead of static libraries"
       ON)

option(USE_OMP "use parallelization use OMP" ON)

set(CMAKE_C_STANDARD 99)

add_definitions(-Wall -Wno-pointer-sign)

if(NOT WIN32)
  add_definitions(-fPIC)
endif()

if(APPLE)
  add_definitions(-march=armv8-a+fp+simd+crypto+crc)
endif(APPLE)

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

if(USE_OMP AND OPENMP_FOUND)
add_definitions(${OpenMP_C_FLAGS} -DUSE_OMP)
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
set(PKG_EXTRA_LIBS -lm)
if(ORC_FOUND)
target_link_libraries(vidstab ${ORC_LIBRARIES})
set(PKG_EXTRA_LIBS "${PKG_EXTRA_LIBS} ${ORC_LIBRARIES}")
endif()
if(USE_OMP AND OPENMP_FOUND)
if(TARGET OpenMP::OpenMP_C)
target_link_libraries(vidstab OpenMP::OpenMP_C)
endif()
set(PKG_EXTRA_LIBS "${PKG_EXTRA_LIBS} ${OpenMP_C_FLAGS}")
endif()


#if(!NOHEADERS)
FILE(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h")
INSTALL(FILES ${HEADERS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/vid.stab)
#endif()

INSTALL(TARGETS vidstab
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

include(create_pkgconfig_file)
create_pkgconfig_file(vidstab "Vid.Stab, a library for stabilizing video clips")
