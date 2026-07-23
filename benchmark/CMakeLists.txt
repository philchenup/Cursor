cmake_minimum_required(VERSION 3.16)
project(pcl_bench LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(PCL 1.12 REQUIRED COMPONENTS
  common io filters features search kdtree sample_consensus segmentation
  surface registration
)
find_package(nlohmann_json 3.2.0 REQUIRED)

add_executable(bench_pcl src/bench_pcl.cpp)
target_include_directories(bench_pcl PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(bench_pcl PRIVATE ${PCL_LIBRARIES} nlohmann_json::nlohmann_json)
target_compile_definitions(bench_pcl PRIVATE ${PCL_DEFINITIONS})
