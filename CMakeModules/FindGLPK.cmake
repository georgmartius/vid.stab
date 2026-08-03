# find GLPK (GNU Linear Programming Kit)
#
# defines
#   GLPK_FOUND
#   GLPK_INCLUDE_DIRS
#   GLPK_LIBRARIES
# and the imported target GLPK::GLPK

find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(PC_GLPK QUIET glpk)
endif()

find_path(GLPK_INCLUDE_DIR
  NAMES glpk.h
  HINTS ${PC_GLPK_INCLUDEDIR} ${PC_GLPK_INCLUDE_DIRS} $ENV{GLPK_DIR}/include
  PATH_SUFFIXES glpk
  DOC "Directory containing glpk.h")

find_library(GLPK_LIBRARY
  NAMES glpk
  HINTS ${PC_GLPK_LIBDIR} ${PC_GLPK_LIBRARY_DIRS} $ENV{GLPK_DIR}/lib
  DOC "Path to the GLPK library")

if(PC_GLPK_VERSION)
  set(GLPK_VERSION ${PC_GLPK_VERSION})
elseif(GLPK_INCLUDE_DIR AND EXISTS "${GLPK_INCLUDE_DIR}/glpk.h")
  file(STRINGS "${GLPK_INCLUDE_DIR}/glpk.h" _glpk_major
       REGEX "^#define[ \t]+GLP_MAJOR_VERSION[ \t]+[0-9]+")
  file(STRINGS "${GLPK_INCLUDE_DIR}/glpk.h" _glpk_minor
       REGEX "^#define[ \t]+GLP_MINOR_VERSION[ \t]+[0-9]+")
  string(REGEX REPLACE ".*[ \t]([0-9]+).*" "\\1" _glpk_major "${_glpk_major}")
  string(REGEX REPLACE ".*[ \t]([0-9]+).*" "\\1" _glpk_minor "${_glpk_minor}")
  # note the MATCHES rather than a plain truth test: GLP_MINOR_VERSION is 0 for
  # GLPK 5.0, and "0" is false in CMake
  if(_glpk_major MATCHES "^[0-9]+$" AND _glpk_minor MATCHES "^[0-9]+$")
    set(GLPK_VERSION "${_glpk_major}.${_glpk_minor}")
  endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GLPK
  REQUIRED_VARS GLPK_LIBRARY GLPK_INCLUDE_DIR
  VERSION_VAR GLPK_VERSION)

if(GLPK_FOUND)
  set(GLPK_LIBRARIES ${GLPK_LIBRARY})
  set(GLPK_INCLUDE_DIRS ${GLPK_INCLUDE_DIR})
  if(NOT TARGET GLPK::GLPK)
    add_library(GLPK::GLPK UNKNOWN IMPORTED)
    set_target_properties(GLPK::GLPK PROPERTIES
      IMPORTED_LOCATION "${GLPK_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${GLPK_INCLUDE_DIR}")
  endif()
endif()

mark_as_advanced(GLPK_INCLUDE_DIR GLPK_LIBRARY)
