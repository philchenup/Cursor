cmake_minimum_required(VERSION 3.16)
project(arm_vision_avoidance LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_library(arm_avoidance
    src/arm_avoidance/SixAxisArm.cpp
    src/arm_avoidance/EsdfGrid.cpp
    src/arm_avoidance/DepthVision.cpp
    src/arm_avoidance/ReactiveAvoidance.cpp
)
target_include_directories(arm_avoidance PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(arm_avoidance PRIVATE -Wall -Wextra -Wpedantic)

add_executable(avoidance_demo src/arm_avoidance/Demo.cpp)
target_link_libraries(avoidance_demo PRIVATE arm_avoidance)
target_compile_options(avoidance_demo PRIVATE -Wall -Wextra -Wpedantic)

add_executable(avoidance_tests tests/test_avoidance.cpp)
target_link_libraries(avoidance_tests PRIVATE arm_avoidance)
target_compile_options(avoidance_tests PRIVATE -Wall -Wextra -Wpedantic)

enable_testing()
add_test(NAME avoidance_tests COMMAND avoidance_tests)
