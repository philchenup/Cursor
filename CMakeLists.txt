cmake_minimum_required(VERSION 3.10)
project(extract_edge_cloud LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# PCL 1.12
find_package(PCL 1.12 REQUIRED COMPONENTS common io kdtree visualization)
find_package(Eigen3 REQUIRED)

add_executable(extract_edge_cloud src/extract_edge_cloud.cpp)

target_include_directories(extract_edge_cloud PRIVATE
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(extract_edge_cloud PRIVATE ${PCL_LIBRARIES})
target_compile_definitions(extract_edge_cloud PRIVATE ${PCL_DEFINITIONS})

# 部分平台 PCL_DEFINITIONS 含分号列表，用现代写法更稳妥
if(PCL_COMPILE_OPTIONS)
  target_compile_options(extract_edge_cloud PRIVATE ${PCL_COMPILE_OPTIONS})
endif()
