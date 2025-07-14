include(CheckCCompilerFlag)

if (MSVC)
      message(STATUS "MSVC detected, enabling default SSE2+ support")

      set(SSE2_FOUND TRUE CACHE BOOL "SSE2 available on target" FORCE)
      set(SSE3_FOUND TRUE CACHE BOOL "SSE3 available on target" FORCE)
      set(SSSE3_FOUND TRUE CACHE BOOL "SSSE3 available on target" FORCE)
      set(SSE4_1_FOUND TRUE CACHE BOOL "SSE4.1 available on target" FORCE)
      add_compile_options(/arch:AVX2)

else()
      # GNU/Clang-like flow
      check_c_compiler_flag(-msse2 HAVE_SSE2)
      check_c_compiler_flag(-msse3 HAVE_SSE3)
      check_c_compiler_flag(-mssse3 HAVE_SSSE3)
      check_c_compiler_flag(-msse4.1 HAVE_SSE4_1)

      if(HAVE_SSE2)
            try_compile(SSE_OK "${PROJECT_BINARY_DIR}"
                    "${CMAKE_CURRENT_LIST_DIR}/TestSSE2.c"
                    COMPILE_DEFINITIONS "-msse2" )
            if(NOT SSE_OK)
                  message(STATUS "SSE2 test compilation fails")
                  set(HAVE_SSE2 FALSE)
            endif()
      endif()

      if(HAVE_SSE3)
            try_compile(SSE_OK "${PROJECT_BINARY_DIR}"
                    "${CMAKE_CURRENT_LIST_DIR}/TestSSE3.c"
                    COMPILE_DEFINITIONS "-msse3" )
            if(NOT SSE_OK)
                  message(STATUS "SSE3 test compilation fails")
                  set(HAVE_SSE3 FALSE)
            endif()
      endif()

      if(HAVE_SSSE3)
            try_compile(SSE_OK "${PROJECT_BINARY_DIR}"
                    "${CMAKE_CURRENT_LIST_DIR}/TestSSSE3.c"
                    COMPILE_DEFINITIONS "-mssse3" )
            if(NOT SSE_OK)
                  message(STATUS "SSSE3 test compilation fails")
                  set(HAVE_SSSE3 FALSE)
            endif()
      endif()

      if(HAVE_SSE4_1)
            try_compile(SSE_OK "${PROJECT_BINARY_DIR}"
                    "${CMAKE_CURRENT_LIST_DIR}/TestSSE41.c"
                    COMPILE_DEFINITIONS "-msse4.1" )
            if(NOT SSE_OK)
                  message(STATUS "SSE4.1 test compilation fails")
                  set(HAVE_SSE4_1 FALSE)
            endif()
      endif()

      set(SSE2_FOUND   ${HAVE_SSE2}   CACHE BOOL "SSE2 available on target")
      set(SSE3_FOUND   ${HAVE_SSE3}   CACHE BOOL "SSE3 available on target")
      set(SSSE3_FOUND  ${HAVE_SSSE3}  CACHE BOOL "SSSE3 available on target")
      set(SSE4_1_FOUND ${HAVE_SSE4_1} CACHE BOOL "SSE4.1 available on target")
endif()

# Output messages
if(NOT SSE2_FOUND)
      message(STATUS "SSE2 is not supported on target platform.")
endif()
if(NOT SSE3_FOUND)
      message(STATUS "SSE3 is not supported on target platform.")
endif()
if(NOT SSSE3_FOUND)
      message(STATUS "SSSE3 is not supported on target platform.")
endif()
if(NOT SSE4_1_FOUND)
      message(STATUS "SSE4.1 is not supported on target platform.")
endif()

mark_as_advanced(SSE2_FOUND SSE3_FOUND SSSE3_FOUND SSE4_1_FOUND)
