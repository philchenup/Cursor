cmake_minimum_required(VERSION 3.14)
project(M3TGlewFinderTest LANGUAGES C)

# Isolated test of m3t/cmake/M3TSetupGLEW.cmake:
# 1) A fake GLEW prefix is resolved via GLEW_ROOT / GLEW_INCLUDE_DIRS / GLEW_LIBRARIES.
# 2) BUILD_SHARED_LIBS=ON is the Windows DLL path (no GLEW_STATIC).

if(NOT DEFINED M3T_CMAKE_DIR)
  message(FATAL_ERROR "Pass -DM3T_CMAKE_DIR=/path/to/m3t/cmake")
endif()

find_package(OpenGL QUIET)

set(BUILD_SHARED_LIBS ON)
set(M3T_FETCH_GLEW OFF)
list(INSERT CMAKE_MODULE_PATH 0 "${M3T_CMAKE_DIR}")
include(M3TSetupGLEW)
m3t_setup_glew()

if(NOT TARGET GLEW::GLEW)
  message(FATAL_ERROR "GLEW::GLEW was not created")
endif()

get_target_property(_inc GLEW::GLEW INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(_loc GLEW::GLEW IMPORTED_LOCATION)
if(NOT _inc)
  message(FATAL_ERROR "GLEW::GLEW has no INTERFACE_INCLUDE_DIRECTORIES")
endif()
if(NOT _loc)
  message(FATAL_ERROR "GLEW::GLEW has no IMPORTED_LOCATION")
endif()
if(NOT EXISTS "${_inc}/GL/glew.h")
  message(FATAL_ERROR "glew.h not found under ${_inc}")
endif()
if(NOT EXISTS "${_loc}")
  message(FATAL_ERROR "GLEW library not found at ${_loc}")
endif()

message(STATUS "GLEW finder test passed")
message(STATUS "  include = ${_inc}")
message(STATUS "  library = ${_loc}")
