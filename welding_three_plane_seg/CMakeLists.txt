cmake_minimum_required(VERSION 3.16)
project(pcl_sac_ransac_planes LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(MPI QUIET)
find_package(PCL 1.8 REQUIRED COMPONENTS common io filters features kdtree search sample_consensus segmentation visualization)

add_executable(pcl_sac_ransac_planes pcl_sac_ransac_planes.cpp)
target_include_directories(pcl_sac_ransac_planes PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(pcl_sac_ransac_planes PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(pcl_sac_ransac_planes PRIVATE ${PCL_LIBRARIES})
