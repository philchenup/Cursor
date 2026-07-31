cmake_minimum_required(VERSION 3.16)
project(HWISplash LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt6 COMPONENTS Widgets QUIET)
if(NOT Qt6_FOUND)
    find_package(Qt5 5.15 REQUIRED COMPONENTS Widgets)
    set(QT_PACKAGE Qt5)
else()
    set(QT_PACKAGE Qt6)
endif()

add_executable(hwi_app src/main.cpp)
target_link_libraries(hwi_app PRIVATE ${QT_PACKAGE}::Widgets)

add_custom_command(TARGET hwi_app POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:hwi_app>/icon
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/icon
            $<TARGET_FILE_DIR:hwi_app>/icon
)
