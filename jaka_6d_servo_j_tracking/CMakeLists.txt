cmake_minimum_required(VERSION 3.16)
project(jaka_6d_servo_j_tracking LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Point this at the directory that contains libjakaAPI.so / jakaAPI.dll
# and keep the bundled headers in third_party/jaka, or override JAKA_SDK_DIR.
set(JAKA_SDK_DIR "" CACHE PATH "JAKA SDK library directory")

add_library(jaka_6d_tracker
    src/jaka_6d_tracker.cpp
)
target_include_directories(jaka_6d_tracker
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/jaka
)
target_compile_features(jaka_6d_tracker PUBLIC cxx_std_17)

if(UNIX)
    target_link_libraries(jaka_6d_tracker PUBLIC pthread)
endif()

add_executable(jaka_6d_track_demo src/jaka_6d_track_main.cpp)
target_link_libraries(jaka_6d_track_demo PRIVATE jaka_6d_tracker)

if(JAKA_SDK_DIR)
    find_library(JAKA_API
        NAMES jakaAPI jakaAPI.so libjakaAPI
        PATHS ${JAKA_SDK_DIR}
        REQUIRED
    )
    target_link_libraries(jaka_6d_tracker PUBLIC ${JAKA_API})
    target_link_directories(jaka_6d_track_demo PRIVATE ${JAKA_SDK_DIR})
endif()
