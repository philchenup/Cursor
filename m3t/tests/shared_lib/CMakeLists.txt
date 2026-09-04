cmake_minimum_required(VERSION 3.14)
project(M3TSharedLibTest LANGUAGES CXX)

# Mirrors the Windows DLL fix: add_library without STATIC + WINDOWS_EXPORT_ALL_SYMBOLS.
option(BUILD_SHARED_LIBS "Build shared libraries" ON)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_library(m3t m3t.cpp)
set_target_properties(m3t PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    WINDOWS_EXPORT_ALL_SYMBOLS ON
    OUTPUT_NAME m3t)

add_executable(m3t_demo main.cpp)
target_link_libraries(m3t_demo PRIVATE m3t)

get_target_property(_type m3t TYPE)
if(BUILD_SHARED_LIBS)
  if(NOT _type STREQUAL "SHARED_LIBRARY")
    message(FATAL_ERROR "Expected SHARED_LIBRARY, got ${_type}")
  endif()
else()
  if(NOT _type STREQUAL "STATIC_LIBRARY")
    message(FATAL_ERROR "Expected STATIC_LIBRARY, got ${_type}")
  endif()
endif()
message(STATUS "m3t target type: ${_type}")
