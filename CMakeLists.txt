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
# 多个 PCL 并存时，旧版本（1.12 等）经常先出现在 PATH / PCL_DIR 里。
# 先选定 1.15，再找 VTK：PCL 1.15 会把 VTK_DIR 指到它自己编译时用的那一份。
set(_pcl_hint_roots "")
foreach(_env_name IN ITEMS PCL_1_15_ROOT PCL115_ROOT PCL_ROOT)
    if(DEFINED ENV{${_env_name}} AND NOT "$ENV{${_env_name}}" STREQUAL "")
        list(APPEND _pcl_hint_roots "$ENV{${_env_name}}")
    endif()
endforeach()
if(DEFINED ENV{PCL_DIR} AND NOT "$ENV{PCL_DIR}" STREQUAL "")
    list(APPEND _pcl_hint_roots "$ENV{PCL_DIR}")
endif()
foreach(_prefix IN LISTS CMAKE_PREFIX_PATH)
    list(APPEND _pcl_hint_roots "${_prefix}")
endforeach()

if(WIN32)
    foreach(_root IN ITEMS
            "C:/Program Files/PCL 1.15.1"
            "C:/Program Files/PCL 1.15.0"
            "C:/Program Files/PCL 1.15"
            "D:/ThirdParty/PCL 1.15.1"
            "D:/ThirdParty/PCL 1.15.0"
            "D:/ThirdParty/PCL1.15.1"
            "D:/ThirdParty/PCL1.15.0"
            "D:/ThirdParty/PCL1.15"
            "D:/PCL 1.15.1"
            "D:/PCL 1.15.0")
        list(APPEND _pcl_hint_roots "${_root}")
    endforeach()
    foreach(_parent IN ITEMS "C:/Program Files" "D:/ThirdParty" "D:/")
        if(IS_DIRECTORY "${_parent}")
            file(GLOB _pcl115_kids LIST_DIRECTORIES true
                "${_parent}/PCL*1.15*"
                "${_parent}/pcl*1.15*")
            list(APPEND _pcl_hint_roots ${_pcl115_kids})
        endif()
    endforeach()
endif()

macro(_cloudtool_consider_pcl_root root)
    if(NOT "${root}" STREQUAL "" AND EXISTS "${root}")
        foreach(_rel IN ITEMS "" "cmake" "lib/cmake/pcl" "share/pcl-1.15")
            if("${_rel}" STREQUAL "")
                set(_dir "${root}")
            else()
                set(_dir "${root}/${_rel}")
            endif()
            if(EXISTS "${_dir}/PCLConfig.cmake" OR EXISTS "${_dir}/pcl-config.cmake")
                list(APPEND _pcl_candidates "${_dir}")
            endif()
        endforeach()
    endif()
endmacro()

set(_pcl_candidates "")
foreach(_root IN LISTS _pcl_hint_roots)
    _cloudtool_consider_pcl_root("${_root}")
endforeach()
if(PCL_DIR)
    list(APPEND _pcl_candidates "${PCL_DIR}")
endif()

set(_pcl115_dir "")
foreach(_dir IN LISTS _pcl_candidates)
    set(_version "")
    if(EXISTS "${_dir}/PCLConfigVersion.cmake")
        file(STRINGS "${_dir}/PCLConfigVersion.cmake" _ver_lines REGEX "PACKAGE_VERSION[ \t]")
        string(REGEX MATCH "([0-9]+\\.[0-9]+\\.[0-9]+)" _version "${_ver_lines}")
    endif()
    if(NOT _version AND EXISTS "${_dir}/PCLConfig.cmake")
        file(STRINGS "${_dir}/PCLConfig.cmake" _ver_lines REGEX "set\\(PCL_VERSION")
        string(REGEX MATCH "([0-9]+\\.[0-9]+\\.[0-9]+)" _version "${_ver_lines}")
    endif()
    if(_version MATCHES "^1\\.15\\.")
        set(_pcl115_dir "${_dir}")
        break()
    endif()
endforeach()

if(_pcl115_dir)
    set(PCL_DIR "${_pcl115_dir}" CACHE PATH "Directory containing PCL 1.15 PCLConfig.cmake" FORCE)
    if(_pcl115_dir MATCHES "/cmake$")
        get_filename_component(_pcl115_prefix "${_pcl115_dir}" DIRECTORY)
        list(PREPEND CMAKE_PREFIX_PATH "${_pcl115_prefix}")
    endif()
    message(STATUS "Prefer PCL 1.15: ${PCL_DIR}")
    find_package(PCL 1.15 CONFIG REQUIRED)
else()
    message(STATUS "PCL 1.15 was not found in the preferred locations. Searching the default path.")
    find_package(PCL REQUIRED)
endif()

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
