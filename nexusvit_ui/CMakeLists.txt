cmake_minimum_required(VERSION 3.16)
project(NexusVIT_UI LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets)

add_executable(nexusvit_ui
    src/main.cpp
    src/Icons.cpp
    src/ViewportWidget.cpp
    src/MainWindow.cpp
    include/Icons.h
    include/ViewportWidget.h
    include/MainWindow.h
    resources/nexusvit.qrc
)

target_include_directories(nexusvit_ui PRIVATE include)
target_link_libraries(nexusvit_ui PRIVATE Qt6::Widgets)

if(WIN32)
    set_target_properties(nexusvit_ui PROPERTIES WIN32_EXECUTABLE TRUE)
endif()

install(TARGETS nexusvit_ui RUNTIME DESTINATION bin)
