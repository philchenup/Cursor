cmake_minimum_required(VERSION 3.10)
project(filterreg LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  message(WARNING "If link fails with missing libstdc++, reconfigure with -DCMAKE_CXX_COMPILER=g++")
endif()

find_package(Eigen3 REQUIRED)

add_library(filterreg
    permutohedral.cpp
    filterreg.cpp
    pcd_io.cpp
)
target_include_directories(filterreg PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(filterreg PUBLIC Eigen3::Eigen)

add_executable(filterreg_demo main.cpp)
target_link_libraries(filterreg_demo PRIVATE filterreg)

add_custom_command(TARGET filterreg_demo POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_SOURCE_DIR}/data/bunny.pcd
        ${CMAKE_SOURCE_DIR}/data/cloud_0.pcd
        ${CMAKE_SOURCE_DIR}/data/cloud_1.pcd
        $<TARGET_FILE_DIR:filterreg_demo>)
