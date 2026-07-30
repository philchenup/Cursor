cmake_minimum_required(VERSION 3.16)
project(vosk_qt_asr LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Prefer Qt6, fall back to Qt5（Visual Studio + Qt VS Tools / CMake 均可）
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets Multimedia)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets Multimedia)

set(VOSK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/vosk" CACHE PATH
    "Directory containing libvosk / vosk.lib / libvosk.so")

find_path(VOSK_INCLUDE_DIR
    NAMES vosk_api.h
    PATHS
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${VOSK_ROOT}"
        /usr/local/include
        /usr/include
)

if(WIN32)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(_VOSK_LIB_CANDIDATES libvosk vosk libvosk_x64)
    else()
        set(_VOSK_LIB_CANDIDATES libvosk vosk)
    endif()
else()
    set(_VOSK_LIB_CANDIDATES vosk libvosk)
endif()

find_library(VOSK_LIBRARY
    NAMES ${_VOSK_LIB_CANDIDATES}
    PATHS
        "${VOSK_ROOT}"
        "${VOSK_ROOT}/lib"
        /usr/local/lib
        /usr/lib
)

if(NOT VOSK_INCLUDE_DIR OR NOT VOSK_LIBRARY)
    message(FATAL_ERROR
        "未找到 Vosk 库。请运行 scripts/setup_vosk.sh（Linux）"
        "或将 Windows 预编译包中的 libvosk.dll / libvosk.lib / vosk_api.h "
        "放到 third_party/vosk/ 与 include/。\n"
        "发布包: https://github.com/alphacep/vosk-api/releases")
endif()

set(PROJECT_SOURCES
    src/main.cpp
    src/audio_capture.cpp
    src/vosk_asr_engine.cpp
    src/main_window.cpp
    include/audio_capture.h
    include/vosk_asr_engine.h
    include/main_window.h
)

add_executable(vosk_qt_asr WIN32 ${PROJECT_SOURCES})

target_include_directories(vosk_qt_asr PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${VOSK_INCLUDE_DIR}
)

target_link_libraries(vosk_qt_asr PRIVATE
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Multimedia
    ${VOSK_LIBRARY}
)

if(UNIX AND NOT APPLE)
    target_link_libraries(vosk_qt_asr PRIVATE pthread)
endif()

# 运行时查找 libvosk.so / libvosk.dll
set_target_properties(vosk_qt_asr PROPERTIES
    BUILD_RPATH "${VOSK_ROOT}"
    INSTALL_RPATH "${VOSK_ROOT}"
)

# 把 model 目录提示复制到构建输出旁（不复制大模型文件内容，仅建链接可选）
add_custom_command(TARGET vosk_qt_asr POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:vosk_qt_asr>"
    COMMENT "Ensure output directory exists"
)

if(WIN32)
    # 复制 DLL 到 exe 旁，方便 Visual Studio F5 调试
    add_custom_command(TARGET vosk_qt_asr POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${VOSK_ROOT}/libvosk.dll"
            "$<TARGET_FILE_DIR:vosk_qt_asr>/"
        COMMENT "Copy libvosk.dll next to executable (Windows)"
    )
endif()

install(TARGETS vosk_qt_asr RUNTIME DESTINATION bin)

message(STATUS "Qt version : ${QT_VERSION_MAJOR}")
message(STATUS "Vosk include: ${VOSK_INCLUDE_DIR}")
message(STATUS "Vosk library: ${VOSK_LIBRARY}")
