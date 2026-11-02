cmake_minimum_required(VERSION 3.16)
project(ScreenRecord VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTORCC ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

add_executable(screenrecord
    main.cpp
    screenrecord.cpp
    screenrecord.h
)

target_include_directories(screenrecord PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(screenrecord PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)

if (MSVC)
    target_compile_options(screenrecord PRIVATE /W4)
else()
    target_compile_options(screenrecord PRIVATE -Wall -Wextra)
endif()
