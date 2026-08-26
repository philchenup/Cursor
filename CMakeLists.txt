cmake_minimum_required(VERSION 3.10)
project(jaka_zu12_trajectory LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(jaka_zu12_traj_plan STATIC
    src/JakaZu12TrajectoryPlan.cpp
)
target_include_directories(jaka_zu12_traj_plan PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/third_party/jaka
)

add_library(jaka_zu12_traj STATIC
    src/JakaZu12Trajectory.cpp
)
target_link_libraries(jaka_zu12_traj PUBLIC jaka_zu12_traj_plan)
target_include_directories(jaka_zu12_traj PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/third_party/jaka
)

add_executable(jaka_zu12_traj_test
    tests/JakaZu12TrajectoryPlanTest.cpp
)
target_link_libraries(jaka_zu12_traj_test PRIVATE jaka_zu12_traj_plan)

# 实机示例需要官方 SDK 动态库：https://github.com/JAKARobotics/jakasdk-cpp
set(JAKA_SDK_LIB "" CACHE FILEPATH "Path to libjakaAPI.so / jakaAPI.lib")
if(JAKA_SDK_LIB)
    add_executable(jaka_zu12_traj_example
        src/JakaZu12TrajectoryExample.cpp
    )
    target_link_libraries(jaka_zu12_traj_example PRIVATE jaka_zu12_traj ${JAKA_SDK_LIB} pthread)
endif()

enable_testing()
add_test(NAME jaka_zu12_traj_test COMMAND jaka_zu12_traj_test)
