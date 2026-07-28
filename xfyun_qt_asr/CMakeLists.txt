cmake_minimum_required(VERSION 3.16)
project(xfyun_qt_asr LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

option(XFYUN_USE_MOCK "Use local MSC mock instead of real iFlytek SDK" ON)

# Prefer Qt6, fall back to Qt5.
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets Multimedia)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets Multimedia)

set(XFYUN_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/xfyun" CACHE PATH
    "Path to iFlytek MSC SDK root (contains include/ and lib/)")

set(PROJECT_SOURCES
    src/main.cpp
    src/audio_capture.cpp
    src/xfyun_asr_engine.cpp
    src/voice_input_dialog.cpp
    include/audio_capture.h
    include/xfyun_asr_engine.h
    include/voice_input_dialog.h
)

add_executable(xfyun_qt_asr ${PROJECT_SOURCES})

target_include_directories(xfyun_qt_asr PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${XFYUN_SDK_ROOT}/include
)

target_link_libraries(xfyun_qt_asr PRIVATE
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Multimedia
)

if(XFYUN_USE_MOCK)
    message(STATUS "Building with XFYUN MSC mock (no official SDK required)")
    target_sources(xfyun_qt_asr PRIVATE src/msc_mock.cpp)
    target_compile_definitions(xfyun_qt_asr PRIVATE XFYUN_USE_MOCK=1)
else()
    message(STATUS "Linking against official iFlytek MSC library from ${XFYUN_SDK_ROOT}")
    if(WIN32)
        if(CMAKE_SIZEOF_VOID_P EQUAL 8)
            set(_MSC_LIB "${XFYUN_SDK_ROOT}/lib/msc_x64.lib")
        else()
            set(_MSC_LIB "${XFYUN_SDK_ROOT}/lib/msc.lib")
        endif()
        target_link_libraries(xfyun_qt_asr PRIVATE "${_MSC_LIB}")
    else()
        find_library(MSC_LIBRARY
            NAMES msc msc_x64
            PATHS "${XFYUN_SDK_ROOT}/lib" "${XFYUN_SDK_ROOT}/bin"
            NO_DEFAULT_PATH
        )
        if(NOT MSC_LIBRARY)
            message(FATAL_ERROR
                "Official MSC library not found under ${XFYUN_SDK_ROOT}. "
                "Place libmsc.so there, or build with -DXFYUN_USE_MOCK=ON")
        endif()
        target_link_libraries(xfyun_qt_asr PRIVATE ${MSC_LIBRARY})
        # Linux: ensure runtime can find .so next to binary / SDK bin.
        set_target_properties(xfyun_qt_asr PROPERTIES
            BUILD_RPATH "${XFYUN_SDK_ROOT}/bin;${XFYUN_SDK_ROOT}/lib"
            INSTALL_RPATH "${XFYUN_SDK_ROOT}/bin;${XFYUN_SDK_ROOT}/lib"
        )
    endif()
endif()

install(TARGETS xfyun_qt_asr RUNTIME DESTINATION bin)

# Headless smoke test for MSC mock + engine pipeline.
if(XFYUN_USE_MOCK)
    add_executable(smoke_asr_engine
        src/smoke_asr_engine.cpp
        src/xfyun_asr_engine.cpp
        src/msc_mock.cpp
        include/xfyun_asr_engine.h
    )
    target_include_directories(smoke_asr_engine PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${XFYUN_SDK_ROOT}/include
    )
    target_compile_definitions(smoke_asr_engine PRIVATE XFYUN_USE_MOCK=1)
    target_link_libraries(smoke_asr_engine PRIVATE Qt${QT_VERSION_MAJOR}::Core)
endif()

# Convenience: copy a sample config next to build tree.
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/config/xfyun.env.example
    ${CMAKE_CURRENT_BINARY_DIR}/xfyun.env.example
    COPYONLY
)
