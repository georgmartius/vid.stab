# find ORC
# - Try to find LibOrc-0.4
# Once done this will define
#  ORC_FOUND - System has LibOrc
#  ORC_INCLUDE_DIRS - The LibOrc include directories
#  ORC_LIBRARIES - The libraries needed to use LibOrc
#  ORC_DEFINITIONS - Compiler switches required for using LibOrc

find_package(PkgConfig) 
pkg_check_modules(PC_ORC orc-0.4)
set(ORC_DEFINITIONS ${PC_ORC_CFLAGS_OTHER})

find_path(ORC_INCLUDE_DIR orc/orc.h
          HINTS ${PC_ORC_INCLUDEDIR} ${PC_ORC_INCLUDE_DIRS}
          PATH_SUFFIXES orc)

find_library(ORC_LIBRARY NAMES orc-0.4
             HINTS ${PC_ORC_LIBDIR} ${PC_ORC_LIBRARY_DIRS} )

set(ORC_LIBRARIES ${ORC_LIBRARY} )
set(ORC_INCLUDE_DIRS ${ORC_INCLUDE_DIR} )
include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set ORC_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(LibOrc  DEFAULT_MSG
                                  ORC_LIBRARY ORC_INCLUDE_DIR)

mark_as_advanced(ORC_INCLUDE_DIR ORC_LIBRARY )
# End find ORC
