cmake_minimum_required(VERSION 3.14)

project(ct_pcl_modules VERSION 3.0 LANGUAGES C CXX)

if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()
if(POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_DEBUG_POSTFIX "d")

# ---------------------------------------------------------------------------
# Windows / MSVC
# ---------------------------------------------------------------------------
if(MSVC)
    add_compile_options(/fp:precise /bigobj /wd4996 /utf-8 /MP)
    add_compile_definitions(_USE_MATH_DEFINES NOMINMAX _CRT_SECURE_NO_WARNINGS)
endif()

# ---------------------------------------------------------------------------
# OpenMP (optional, used by PCL OMP estimators)
# ---------------------------------------------------------------------------
option(WITH_OPENMP "Build with OpenMP parallelization" ON)
if(WITH_OPENMP)
    find_package(OpenMP COMPONENTS C CXX)
    if(OpenMP_CXX_FOUND)
        message(STATUS "Found OpenMP")
    endif()
endif()

# ---------------------------------------------------------------------------
# PCL (required). No Qt / VTK GUI dependency is needed by this library.
#
# Windows 常见安装路径示例：
#   C:/Program Files/PCL 1.14.1
# 配置时传入其一即可：
#   cmake -DPCL_DIR="C:/Program Files/PCL 1.14.1/cmake" ..
#   cmake -DPCL_ROOT="C:/Program Files/PCL 1.14.1" ..
#   cmake -DCMAKE_PREFIX_PATH="C:/Program Files/PCL 1.14.1" ..
# ---------------------------------------------------------------------------
# 只链接算法模块，不依赖 PCL 可视化 / Qt / VTK GUI
find_package(PCL 1.11 REQUIRED COMPONENTS
    common
    io
    kdtree
    octree
    search
    sample_consensus
    filters
    features
    ml
    segmentation
    surface
)

if(NOT PCL_FOUND)
    message(FATAL_ERROR
        "未找到 PCL。请在 Windows 上安装 PCL All-in-One，并设置 PCL_DIR / PCL_ROOT / CMAKE_PREFIX_PATH。\n"
        "示例:\n"
        "  cmake -S . -B build -G \"Visual Studio 17 2022\" -A x64 "
        "-DPCL_DIR=\"C:/Program Files/PCL 1.14.1/cmake\"")
endif()

message(STATUS "Found PCL version: ${PCL_VERSION}")

# ---------------------------------------------------------------------------
# Library
# ---------------------------------------------------------------------------
option(CT_BUILD_SHARED "Build ct_pcl_modules as a shared library" OFF)
option(CT_BUILD_EXAMPLES "Build example executable" ON)

set(CT_SOURCES
    src/base/cloud.cpp
    src/base/common.cpp
    src/modules/filters.cpp
    src/modules/features.cpp
    src/modules/segmentation.cpp
    src/modules/surface.cpp
)

set(CT_HEADERS
    include/base/cloud.h
    include/base/common.h
    include/base/exports.h
    include/modules/filters.h
    include/modules/features.h
    include/modules/segmentation.h
    include/modules/surface.h
)

if(CT_BUILD_SHARED)
    add_library(ct_pcl_modules SHARED ${CT_SOURCES} ${CT_HEADERS})
    target_compile_definitions(ct_pcl_modules PRIVATE CT_LIBRARY)
else()
    add_library(ct_pcl_modules STATIC ${CT_SOURCES} ${CT_HEADERS})
    target_compile_definitions(ct_pcl_modules PUBLIC CT_STATIC)
endif()

add_library(ct::pcl_modules ALIAS ct_pcl_modules)

target_include_directories(ct_pcl_modules
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${PCL_INCLUDE_DIRS}
)

if(PCL_LIBRARY_DIRS)
    target_link_directories(ct_pcl_modules PUBLIC ${PCL_LIBRARY_DIRS})
endif()
if(PCL_DEFINITIONS)
    add_definitions(${PCL_DEFINITIONS})
endif()
target_link_libraries(ct_pcl_modules PUBLIC ${PCL_LIBRARIES})

if(OpenMP_CXX_FOUND)
    target_link_libraries(ct_pcl_modules PUBLIC OpenMP::OpenMP_CXX)
endif()

# ---------------------------------------------------------------------------
# Example
# ---------------------------------------------------------------------------
if(CT_BUILD_EXAMPLES)
    add_executable(ct_pcl_example examples/example.cpp)
    target_link_libraries(ct_pcl_example PRIVATE ct_pcl_modules)
    set_target_properties(ct_pcl_example PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    )

    set(CT_DEMOS
        demo_cloud
        demo_common
        demo_filters
        demo_features
        demo_segmentation
        demo_surface
    )
    foreach(demo_name IN LISTS CT_DEMOS)
        add_executable(${demo_name} examples/${demo_name}.cpp)
        target_include_directories(${demo_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/examples)
        target_link_libraries(${demo_name} PRIVATE ct_pcl_modules)
        set_target_properties(${demo_name} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
        )
    endforeach()
endif()

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
include(GNUInstallDirs)

install(TARGETS ct_pcl_modules
    EXPORT ct_pcl_modulesTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT ct_pcl_modulesTargets
    FILE ct_pcl_modulesTargets.cmake
    NAMESPACE ct::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ct_pcl_modules
)
