cmake_minimum_required (VERSION 3.14)
project (vid.stab C)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/")

include (FindSSE)
include (GNUInstallDirs)
# include (ExternalProject) # There must be a nicer way to include this
# include (sse2neon)
find_package(OpenMP)

set(MAJOR_VERSION 1)
set(MINOR_VERSION 3)
set(PATCH_VERSION 0)
# note the separator between minor and patch: without it this produced "1.20"
# for 1.2.0 (and "1.10" for 1.1.0), which is what ends up in the Version field
# of the generated vidstab.pc
set(VIDSTAB_VERSION ${MAJOR_VERSION}.${MINOR_VERSION}.${PATCH_VERSION})

# Default to release builds if no explicit build type specified.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release")
endif()

option(BUILD_SHARED_LIBS "build shared libraries instead of static libraries"
       ON)

option(USE_OMP "use parallelization use OMP" ON)

set(CMAKE_C_STANDARD 99)

if(CMAKE_C_COMPILER_ID MATCHES "GNU" OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  add_definitions(-Wall -Wno-pointer-sign)
elseif(MSVC)
  add_definitions(/W3)
  # MSVC's <math.h> only defines M_PI and friends when asked to.
  add_definitions(-D_USE_MATH_DEFINES)
endif()

if(NOT WIN32)
  add_definitions(-fPIC)
endif()

if(APPLE AND NOT CMAKE_OSX_ARCHITECTURES AND NOT SSE2_FOUND)
  add_definitions(-march=armv8-a+fp+simd+crypto+crc)
endif()

# --- SIMD kernels ------------------------------------------------------------
# Which kernel runs is decided at runtime (src/cpudetect.c), so the AVX2 and
# AVX-512 variants are compiled in wherever the *compiler* can build them and
# the resulting binary still runs on an SSE2-only host.  Only the SSE2 flag is
# global, since SSE2 is baseline on x86_64.
include (VidstabSimd)
vidstab_configure_simd("${CMAKE_CURRENT_SOURCE_DIR}/src")
message(STATUS "SIMD kernels compiled in: ${VIDSTAB_SIMD_SUMMARY}")

if(NOT MSVC AND SSE2_FOUND)
  add_definitions( -msse2 -ffast-math )
endif()
if(MSVC AND USE_OMP)
  add_compile_options(/openmp)
endif()
if(USE_OMP AND OPENMP_FOUND)
add_definitions(${OpenMP_C_FLAGS} -DUSE_OMP)
endif()

set(SOURCES src/frameinfo.c src/transformtype.c src/libvidstab.c
  src/transform.c src/transformfixedpoint.c src/motiondetect.c
  src/serialize.c src/localmotion2transform.c
  src/boxblur.c src/vsvector.c)
list(APPEND SOURCES ${VIDSTAB_SIMD_SOURCES})
add_compile_definitions(${VIDSTAB_SIMD_DEFS})

# --- LP solver backend for the L1 optimal camera path -------------------------
# The built-in interior point solver keeps vid.stab dependency free; GLPK can be
# selected instead and serves as the cross-check in the test suite.
set(VIDSTAB_LPSOLVER "builtin" CACHE STRING
    "LP solver for the L1 optimal camera path: builtin or glpk")
set_property(CACHE VIDSTAB_LPSOLVER PROPERTY STRINGS builtin glpk)

set(LP_SOURCES src/l1campathoptimization.c src/lpsolver_ipm.c)
set(LP_DEFS -DUSE_IPM -DVS_HAVE_LPSOLVER)
set(LP_BACKEND "built-in interior point")
if(VIDSTAB_LPSOLVER STREQUAL "glpk")
  find_package(GLPK)
  if(GLPK_FOUND)
    set(LP_SOURCES src/l1campathoptimization.c src/lpsolver_glpk.c)
    set(LP_DEFS -DUSE_GLPK -DVS_HAVE_LPSOLVER)
    set(LP_BACKEND "GLPK ${GLPK_VERSION}")
  else()
    message(WARNING "VIDSTAB_LPSOLVER=glpk but GLPK was not found, using the built-in solver")
  endif()
endif()
add_definitions(${LP_DEFS})
message(STATUS "L1 optimal camera path LP solver: ${LP_BACKEND}")
list(APPEND SOURCES ${LP_SOURCES})

set(HEADERS src/frameinfo.h src/transformtype.h src/libvidstab.h
  src/transform.h src/motiondetect.h src/serialize.h
  src/localmotion2transform.h src/boxblur.h src/vsvector.h )


# Create the vidstab library
add_library (vidstab ${SOURCES})

#set version of lib
set_target_properties(vidstab PROPERTIES SOVERSION ${MAJOR_VERSION}.${MINOR_VERSION})


# VS_API (see src/vidstab_api.h) needs to expand to nothing for a static
# build: there is no import library to point a __declspec(dllimport) at, and
# saying so anyway makes consumers fail to link.  PUBLIC, because the headers
# our users compile against have to see the same definition we did.
if(NOT BUILD_SHARED_LIBS)
  target_compile_definitions(vidstab PUBLIC VIDSTAB_STATIC_DEFINE)
endif()
target_include_directories(vidstab PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)
if (NOT MSVC)
  target_link_libraries(vidstab m)
endif ()
set(PKG_EXTRA_LIBS -lm)
if(GLPK_FOUND AND VIDSTAB_LPSOLVER STREQUAL "glpk")
  target_link_libraries(vidstab GLPK::GLPK)
  set(PKG_EXTRA_LIBS "${PKG_EXTRA_LIBS} -lglpk")
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
