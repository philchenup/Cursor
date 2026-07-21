cmake_minimum_required(VERSION 3.10)
project(stl_to_pointcloud)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.8 REQUIRED COMPONENTS common io visualization)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(stl_to_pointcloud src/stl_to_pointcloud.cpp)
target_link_libraries(stl_to_pointcloud ${PCL_LIBRARIES})
