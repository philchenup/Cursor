cmake_minimum_required(VERSION 3.16)
project(pcl_hidden_point_removal LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Distro VTK/PCL packages may reference MPI::MPI_C even when unused.
find_package(MPI COMPONENTS C QUIET)
if(NOT TARGET MPI::MPI_C)
  add_library(MPI::MPI_C INTERFACE IMPORTED)
endif()

find_package(PCL 1.12 REQUIRED COMPONENTS common surface io)

add_library(pcl_hidden_point_removal INTERFACE)
target_include_directories(pcl_hidden_point_removal INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(pcl_hidden_point_removal INTERFACE ${PCL_LIBRARIES})
if(PCL_DEFINITIONS)
  target_compile_definitions(pcl_hidden_point_removal INTERFACE ${PCL_DEFINITIONS})
endif()

add_executable(hidden_point_removal_example
  examples/hidden_point_removal_example.cpp
)
target_link_libraries(hidden_point_removal_example PRIVATE pcl_hidden_point_removal)

install(FILES include/hidden_point_removal.h DESTINATION include)
