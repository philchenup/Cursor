cmake_minimum_required(VERSION 3.16)
project(ScaleAISShape LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OpenCASCADE REQUIRED)

# Core library: scales an AIS_Shape by 1/1000.
add_library(ScaleAISShape src/ScaleAISShape.cpp)
target_include_directories(ScaleAISShape
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCASCADE_INCLUDE_DIR}
)
target_link_libraries(ScaleAISShape
    PUBLIC
        TKernel
        TKMath
        TKG2d
        TKG3d
        TKGeomBase
        TKBRep
        TKTopAlgo
        TKGeomAlgo
        TKPrim
        TKV3d
        TKService
)

# Headless end-to-end demo: build a box, scale it, and verify the result.
add_executable(scale_demo src/ScaleDemo.cpp)
target_link_libraries(scale_demo PRIVATE ScaleAISShape)

# Compile-only check that the interactive-context usage example builds.
add_library(scale_example OBJECT src/ScaleAISShapeExample.cpp)
target_include_directories(scale_example
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCASCADE_INCLUDE_DIR}
)

enable_testing()
add_test(NAME scale_demo COMMAND scale_demo)
