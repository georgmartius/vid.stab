cmake_minimum_required (VERSION 2.6) 
project (vid.stab.test) 

SET(CMAKE_BUILTTYPE None)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/")

find_package(Orc REQUIRED) 

add_definitions(  -Wall -O0 -g  -DTESTING ${ORC_DEFINITIONS})
include_directories (${ORC_INCLUDE_DIRS}) 

add_executable (orc_bug orc_bug.c orc_bug_orc.c )

target_link_libraries(orc_bug m ${ORC_LIBRARIES}) #link the math library
 
