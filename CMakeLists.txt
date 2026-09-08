cmake_minimum_required(VERSION 3.16)
project(CursorTools LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL REQUIRED COMPONENTS common kdtree)
find_package(OpenCV REQUIRED COMPONENTS core)

add_library(fit_circle_3d STATIC
    src/FitCircle3D.cpp
)
target_include_directories(fit_circle_3d PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
target_compile_definitions(fit_circle_3d PUBLIC ${PCL_DEFINITIONS})
target_link_libraries(fit_circle_3d PUBLIC
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)

add_executable(fit_circle_3d_example
    src/FitCircle3DExample.cpp
)
target_link_libraries(fit_circle_3d_example PRIVATE fit_circle_3d)

add_executable(fit_circle_3d_test
    src/FitCircle3DTest.cpp
)
target_link_libraries(fit_circle_3d_test PRIVATE fit_circle_3d)
