cmake_minimum_required(VERSION 3.16)
project(hand_eye_calibration LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

add_library(hand_eye_calibration
    src/HandEyeCalibration.cpp
)
target_include_directories(hand_eye_calibration
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${PCL_INCLUDE_DIRS}
)
target_link_libraries(hand_eye_calibration
    PUBLIC
        Eigen3::Eigen
        ${PCL_LIBRARIES}
)
if(PCL_DEFINITIONS)
    target_compile_definitions(hand_eye_calibration PUBLIC ${PCL_DEFINITIONS})
endif()

add_executable(hand_eye_calibration_example
    src/HandEyeCalibrationExample.cpp
)
target_link_libraries(hand_eye_calibration_example
    PRIVATE
        hand_eye_calibration
)

enable_testing()
add_test(NAME hand_eye_calibration_example COMMAND hand_eye_calibration_example)
