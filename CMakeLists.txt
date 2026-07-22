cmake_minimum_required(VERSION 3.16)
project(WeldCoordinateSystem LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCASCADE REQUIRED)

add_library(weld_cs
  src/WeldCoordinateSystem.cxx
)
target_include_directories(weld_cs
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCASCADE_INCLUDE_DIR}
)
target_link_libraries(weld_cs
  PUBLIC
    TKernel
    TKMath
    TKBRep
    TKG2d
    TKG3d
    TKGeomBase
    TKTopAlgo
)

add_executable(weld_cs_example
  examples/ComputeWeldCoordinateSystem_example.cxx
)
target_link_libraries(weld_cs_example
  PRIVATE
    weld_cs
    TKPrim
)
