cmake_minimum_required(VERSION 3.10)
# C is required so PCL's VTK config can resolve MPI::MPI_C on Ubuntu packages.
project(ced_weld_seam LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif ()

find_package(PCL 1.8 REQUIRED COMPONENTS common io kdtree search features keypoints filters)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(detect_weld_seam src/detect_weld_seam.cpp)
target_link_libraries(detect_weld_seam PRIVATE ${PCL_LIBRARIES})

add_executable(test_ced_weld_seam src/test_ced_weld_seam.cpp)
target_link_libraries(test_ced_weld_seam PRIVATE ${PCL_LIBRARIES})

add_library(geng_rcim2022 src/geng_rcim2022.cpp)
target_link_libraries(geng_rcim2022 PUBLIC ${PCL_LIBRARIES})

add_executable(geng_rcim2022_demo src/geng_rcim2022_demo.cpp)
target_link_libraries(geng_rcim2022_demo PRIVATE geng_rcim2022)

add_executable(test_geng_rcim2022 src/test_geng_rcim2022.cpp)
target_link_libraries(test_geng_rcim2022 PRIVATE geng_rcim2022)

enable_testing()
add_test(NAME ced_weld_seam COMMAND test_ced_weld_seam)
add_test(NAME geng_rcim2022 COMMAND test_geng_rcim2022)
