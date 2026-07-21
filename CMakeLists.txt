cmake_minimum_required(VERSION 3.10)
project(extract_edge_cloud)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.8 REQUIRED COMPONENTS common io filters features search visualization)
find_package(OpenMP)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(extract_edge_cloud src/extract_edge_cloud.cpp)
target_link_libraries(extract_edge_cloud ${PCL_LIBRARIES})

if(OpenMP_CXX_FOUND)
  target_link_libraries(extract_edge_cloud OpenMP::OpenMP_CXX)
endif()
