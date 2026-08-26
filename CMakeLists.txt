cmake_minimum_required(VERSION 3.10)
project(jaka_zu12_trajectory LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT MSVC)
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(jaka_zu12_traj STATIC src/JakaZu12Trajectory.cpp)
target_include_directories(jaka_zu12_traj PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/third_party/jaka
    ${CMAKE_SOURCE_DIR}/tests/stubs
)
