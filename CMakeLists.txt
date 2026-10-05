cmake_minimum_required(VERSION 3.16)
# 同时启用 C：Ubuntu 打包的 VTK 导出目标依赖 MPI::MPI_C
project(SceneSeamExtractor C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# 先找一遍 MPI，使 VTK::mpi 的链接接口能够解析
find_package(MPI QUIET)
find_package(PCL 1.10 REQUIRED COMPONENTS common filters io segmentation visualization)

add_library(scene_seam_extractor STATIC
    src/SceneSeamExtractor.cpp
)
target_include_directories(scene_seam_extractor PUBLIC include ${PCL_INCLUDE_DIRS})
target_link_libraries(scene_seam_extractor PUBLIC ${PCL_LIBRARIES})
target_compile_definitions(scene_seam_extractor PUBLIC ${PCL_DEFINITIONS})

add_executable(scene_seam_test src/SceneSeamExtractorTest.cpp)
target_link_libraries(scene_seam_test PRIVATE scene_seam_extractor)

enable_testing()
add_test(NAME scene_seam_extraction COMMAND scene_seam_test --no-viewer)
