cmake_minimum_required(VERSION 3.16)
project(ais_shape_color_load LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCASCADE REQUIRED)

add_library(read_model
  src/readmodel.cpp
)
target_include_directories(read_model PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCASCADE_INCLUDE_DIR}
)
target_link_libraries(read_model PUBLIC
  TKernel TKMath TKBRep TKG3d TKG2d TKGeomBase TKGeomAlgo
  TKTopAlgo TKMesh TKSTL TKSTEP TKSTEPAttr TKSTEPBase TKXSBase
  TKV3d TKService TKXCAF TKLCAF TKXDESTEP TKCAF
)

# examples/load_with_color.cpp 是调用示例，集成到你的导入槽函数即可。
