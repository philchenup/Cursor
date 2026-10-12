cmake_minimum_required(VERSION 3.16)

project(CloudTool VERSION 3.0)

if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if("${CMAKE_BUILD_TYPE}" STREQUAL "")
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_DEBUG_POSTFIX "d")

if(MSVC)
    set(CMAKE_CXX_FLAGS_DEFAULT "/DWIN32 /D_WINDOWS /W3 /GR /EHsc")

    if("${CMAKE_CXX_FLAGS}" STREQUAL "${CMAKE_CXX_FLAGS_DEFAULT}")
        string(APPEND CMAKE_CXX_FLAGS " /fp:precise /bigobj /wd4996 /source-charset:utf-8 ")
        set(MSVC_MP ${CPUCores})

        if(CMAKE_VERSION VERSION_LESS 3.11.0)
            if(MSVC_MP EQUAL 0)
                string(APPEND CMAKE_CXX_FLAGS " /MP")
            elseif(MSVC_MP GREATER 1)
                string(APPEND CMAKE_CXX_FLAGS " /MP${MSVC_MP}")
            endif()
        else()
            if(MSVC_MP EQUAL 0)
                add_compile_options($<$<OR:$<COMPILE_LANGUAGE:C>,$<COMPILE_LANGUAGE:CXX>>:/MP>)
            elseif(MSVC_MP GREATER 1)
                add_compile_options($<$<OR:$<COMPILE_LANGUAGE:C>,$<COMPILE_LANGUAGE:CXX>>:/MP${MSVC_MP}>)
            endif()
        endif()
    endif()
elseif(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS_DEFAULT "")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wabi=11")
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wabi")
    endif()
endif()

# OpenMP (optional)
option(WITH_OPENMP "Build with parallelization using OpenMP" TRUE)

if(WITH_OPENMP)
    find_package(OpenMP COMPONENTS C CXX)
endif()

if(OpenMP_FOUND)
    string(APPEND CMAKE_C_FLAGS " ${OpenMP_C_FLAGS}")
    string(APPEND CMAKE_CXX_FLAGS " ${OpenMP_CXX_FLAGS}")
    string(APPEND CMAKE_EXE_LINKER_FLAGS "${OpenMP_EXE_LINKER_FLAGS}")

    if(${CMAKE_VERSION} VERSION_LESS "3.7")
        message(STATUS "Found OpenMP")
    else()
        message(STATUS "Found OpenMP, spec date ${OpenMP_CXX_SPEC_DATE}")
    endif()
else()
    message(STATUS "Not found OpenMP")
endif()

# qt
find_package(QT NAMES Qt5 COMPONENTS Core Widgets OpenGL Gui REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)

if(QT_FOUND)
    message(STATUS "Found QT version: ${Qt5_VERSION}.")
else()
    message(STATUS "Qt5 is not found.")
endif()

# pcl
# 本机 All-in-one 安装在 D:/PCL 1.15.1，配置文件在 cmake/PCLConfig.cmake。
# 直接指定该目录，避免 PATH 或缓存里的旧 PCL 先被找到。
set(PCL_1_15_ROOT "D:/PCL 1.15.1" CACHE PATH "PCL 1.15.1 installation root")
if(DEFINED ENV{PCL_1_15_ROOT} AND NOT "$ENV{PCL_1_15_ROOT}" STREQUAL "")
    set(PCL_1_15_ROOT "$ENV{PCL_1_15_ROOT}" CACHE PATH "PCL 1.15.1 installation root" FORCE)
endif()
file(TO_CMAKE_PATH "${PCL_1_15_ROOT}" PCL_1_15_ROOT)

set(_pcl_config_dir "${PCL_1_15_ROOT}/cmake")
if(NOT EXISTS "${_pcl_config_dir}/PCLConfig.cmake")
    message(FATAL_ERROR
        "未找到 ${_pcl_config_dir}/PCLConfig.cmake。确认 PCL 1.15.1 安装在 D:/PCL 1.15.1，或设置 PCL_1_15_ROOT。")
endif()

set(PCL_DIR "${_pcl_config_dir}" CACHE PATH "Directory containing PCL 1.15.1 PCLConfig.cmake" FORCE)
list(PREPEND CMAKE_PREFIX_PATH "${PCL_1_15_ROOT}")
message(STATUS "Using PCL 1.15.1: ${PCL_DIR}")

# All-in-one 自带的 VTK 在 3rdParty/VTK。先写好 VTK_DIR，后面的 find_package(VTK) 不再去搜其它版本。
file(GLOB _vtk_configs
    "${PCL_1_15_ROOT}/3rdParty/VTK/lib/cmake/vtk-*/vtk-config.cmake"
    "${PCL_1_15_ROOT}/3rdParty/VTK/lib/cmake/vtk-*/VTKConfig.cmake"
    "${PCL_1_15_ROOT}/3rdParty/VTK/lib/vtk-*/VTKConfig.cmake")
if(_vtk_configs)
    list(GET _vtk_configs 0 _vtk_config)
    get_filename_component(_vtk_dir "${_vtk_config}" DIRECTORY)
    set(VTK_DIR "${_vtk_dir}" CACHE PATH "VTK shipped with PCL 1.15.1" FORCE)
    message(STATUS "Using VTK from PCL 1.15.1: ${VTK_DIR}")
endif()

find_package(PCL 1.15.1 CONFIG REQUIRED)

if(PCL_FOUND)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    message(STATUS "Found PCL version: ${PCL_VERSION}.")
else()
    message(STATUS "PCL is not found.")
endif()

# vtk
find_package(VTK REQUIRED)

if(VTK_FOUND)
    if(VTK_USE_FILE AND EXISTS "${VTK_USE_FILE}")
        include(${VTK_USE_FILE})
    endif()
    message(STATUS "Found VTK version: ${VTK_VERSION}.")
else()
    message(STATUS "VTK is not found.")
endif()

# camera sdk
# 1. Azure Kinect
set(AzureKinect_DIR "${PROJECT_SOURCE_DIR}/camera/AzureKinect")
include_directories("${AzureKinect_DIR}/include")
if(WIN32)
    link_directories("${AzureKinect_DIR}/lib/win32")
elseif(UNIX)
    link_directories("${AzureKinect_DIR}/lib/linux")
endif()

# 2. Photoneo SDK
set(Photoneo_DIR "${PROJECT_SOURCE_DIR}/camera/Photoneo")
include_directories("${Photoneo_DIR}/include")
if(WIN32)
    link_directories("${Photoneo_DIR}/lib/win32")
elseif(UNIX)
    link_directories("${Photoneo_DIR}/lib/linux")
endif()


message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

include_directories(${PROJECT_SOURCE_DIR})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/install)

add_definitions(-DROOT_PATH="${PROJECT_SOURCE_DIR}")
add_definitions(-DDATA_PATH="${PROJECT_SOURCE_DIR}/data/")

add_subdirectory(base)
add_subdirectory(modules)
add_subdirectory(cloudtool)
