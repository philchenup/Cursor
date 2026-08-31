cmake_minimum_required(VERSION 3.16)
project(NexusVITWorkspaceUi LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt6 COMPONENTS Widgets QUIET)
if(NOT Qt6_FOUND)
    find_package(Qt5 5.15 COMPONENTS Widgets QUIET)
endif()

if(Qt6_FOUND OR Qt5_FOUND)
    if(Qt6_FOUND)
        set(QT_PACKAGE Qt6)
    else()
        set(QT_PACKAGE Qt5)
    endif()

    add_executable(nexusvit_ui_preview
        src/nexus_ui_preview.cpp
        src/NexusWorkspaceWindow.cpp
        include/NexusWorkspaceWindow.h
    )
    target_include_directories(nexusvit_ui_preview PRIVATE include)
    target_link_libraries(nexusvit_ui_preview PRIVATE ${QT_PACKAGE}::Widgets)

    add_custom_command(TARGET nexusvit_ui_preview POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:nexusvit_ui_preview>/ui
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${CMAKE_SOURCE_DIR}/ui/nexus_workspace.qss
                $<TARGET_FILE_DIR:nexusvit_ui_preview>/ui/nexus_workspace.qss
    )
else()
    message(STATUS "Qt not found — skipping nexusvit_ui_preview (HTML mockup in preview/ still applies)")
endif()
