cmake_minimum_required(VERSION 3.10)
project(pcl_vtk_ortho_views LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PCL 1.12 REQUIRED COMPONENTS common io visualization)
find_package(VTK REQUIRED)

add_executable(pcl_vtk_ortho_views src/pcl_vtk_ortho_views.cpp)

target_include_directories(pcl_vtk_ortho_views PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pcl_vtk_ortho_views PRIVATE ${PCL_LIBRARIES})
target_compile_definitions(pcl_vtk_ortho_views PRIVATE ${PCL_DEFINITIONS})

if(PCL_COMPILE_OPTIONS)
  target_compile_options(pcl_vtk_ortho_views PRIVATE ${PCL_COMPILE_OPTIONS})
endif()
