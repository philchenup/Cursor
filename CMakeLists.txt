# Minimal CMake example: link LZ4 so FLANN/PCL kdtree consumers resolve
# LZ4_resetStreamHC / LZ4_compress_HC_continue / LZ4_decompress_safe* etc.
#
# Replace pcd_utils.cpp with your sources. Requires PCL + LZ4 on the machine.

cmake_minimum_required(VERSION 3.16)
project(pcd_utils_lz4_link_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL REQUIRED COMPONENTS common kdtree search)

add_library(pcd_utils STATIC
  # Drop in your real pcd_utils.cpp (or rename the target).
  examples/pcd_utils_lz4_link_stub.cpp
)

target_include_directories(pcd_utils PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(pcd_utils PUBLIC ${PCL_LIBRARIES})

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(FindOrLinkLZ4)
target_link_lz4(pcd_utils)
