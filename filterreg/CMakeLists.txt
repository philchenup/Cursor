cmake_minimum_required(VERSION 3.14)

project(filterreg VERSION 1.0 LANGUAGES C CXX)

if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()
if(POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_DEBUG_POSTFIX "d")

# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------
option(FILTERREG_BUILD_PCD_UTILS
    "Build pcd_utils (needs PCL, OpenCV, OpenMP, small_gicp, ct::Cloud)" ON)
option(FILTERREG_BUILD_EXAMPLES "Build example executables" ON)

# ---------------------------------------------------------------------------
# Windows / MSVC
# ---------------------------------------------------------------------------
if(MSVC)
    add_compile_options(/fp:precise /bigobj /wd4996 /utf-8 /MP)
    add_compile_definitions(_USE_MATH_DEFINES NOMINMAX _CRT_SECURE_NO_WARNINGS)
endif()

# ---------------------------------------------------------------------------
# Eigen3 (filterreg / permutohedral only need this)
#
# Windows 常见路径：
#   C:/Program Files/Eigen3
#   C:/eigen-3.4.0
#   C:/Program Files/PCL 1.14.1/3rdParty/Eigen3
# 配置示例：
#   cmake -DEigen3_DIR="C:/Program Files/Eigen3/share/eigen3/cmake" ..
#   cmake -DEIGEN3_INCLUDE_DIR="C:/eigen-3.4.0" ..
# ---------------------------------------------------------------------------
function(filterreg_setup_eigen)
    if(TARGET Eigen3::Eigen)
        return()
    endif()

    find_package(Eigen3 QUIET NO_MODULE)
    if(TARGET Eigen3::Eigen)
        return()
    endif()

    set(_eigen_hints
        "${EIGEN3_INCLUDE_DIR}"
        "$ENV{EIGEN3_INCLUDE_DIR}"
        "C:/Program Files/Eigen3/include"
        "C:/Program Files/Eigen3/include/eigen3"
        "C:/eigen-3.4.0"
        "C:/Program Files/PCL 1.14.1/3rdParty/Eigen3/include"
        "C:/Program Files/PCL 1.14.1/3rdParty/Eigen3"
    )
    if(DEFINED PCL_ROOT)
        list(APPEND _eigen_hints
            "${PCL_ROOT}/3rdParty/Eigen3/include"
            "${PCL_ROOT}/3rdParty/Eigen3")
    endif()

    find_path(EIGEN3_INCLUDE_DIR
        NAMES Eigen/Core signature_of_eigen3_matrix_library
        HINTS ${_eigen_hints}
        PATH_SUFFIXES eigen3 include include/eigen3
    )
    if(NOT EIGEN3_INCLUDE_DIR)
        message(FATAL_ERROR
            "未找到 Eigen3。请安装 Eigen，并设置 Eigen3_DIR 或 EIGEN3_INCLUDE_DIR。\n"
            "Windows 示例:\n"
            "  cmake -S . -B build -G \"Visual Studio 17 2022\" -A x64 "
            "-DEIGEN3_INCLUDE_DIR=\"C:/eigen-3.4.0\"")
    endif()
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    target_include_directories(Eigen3::Eigen INTERFACE "${EIGEN3_INCLUDE_DIR}")
    message(STATUS "Using Eigen3 include: ${EIGEN3_INCLUDE_DIR}")
endfunction()

filterreg_setup_eigen()

# ---------------------------------------------------------------------------
# filterreg + permutohedral  (Eigen only)
# ---------------------------------------------------------------------------
add_library(filterreg STATIC
    src/filterreg.cpp
    src/permutohedral.cpp
    include/filterreg.h
    include/permutohedral.h
)

add_library(filterreg::filterreg ALIAS filterreg)

target_include_directories(filterreg
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(filterreg PUBLIC Eigen3::Eigen)

# ---------------------------------------------------------------------------
# pcd_utils (optional)
# ---------------------------------------------------------------------------
if(FILTERREG_BUILD_PCD_UTILS)
    find_package(OpenMP COMPONENTS CXX)

    # PCL：必须一次找齐 pcl_modules + pcd_utils 所需模块（含 registration）。
    # Windows All-in-One 安装后传入 PCL_DIR 或 PCL_ROOT：
    #   cmake -DPCL_DIR="C:/Program Files/PCL 1.14.1/cmake" ..
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
        registration
    )
    message(STATUS "Found PCL version: ${PCL_VERSION}")

    # OpenCV：Windows 传入 OpenCV_DIR
    #   cmake -DOpenCV_DIR="C:/opencv/build" ..
    find_package(OpenCV REQUIRED COMPONENTS core imgproc)
    message(STATUS "Found OpenCV version: ${OpenCV_VERSION}")

    # ct::Cloud：默认使用仓库内的 pcl_modules
    set(PCL_MODULES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../pcl_modules"
        CACHE PATH "Path to pcl_modules (provides ct::Cloud)")
    if(NOT TARGET ct_pcl_modules)
        if(NOT EXISTS "${PCL_MODULES_DIR}/CMakeLists.txt")
            message(FATAL_ERROR
                "未找到 pcl_modules（ct::Cloud）。请设置 PCL_MODULES_DIR。\n"
                "当前值: ${PCL_MODULES_DIR}")
        endif()
        set(CT_BUILD_EXAMPLES OFF CACHE BOOL "Disable pcl_modules examples when nested" FORCE)
        add_subdirectory("${PCL_MODULES_DIR}" "${CMAKE_BINARY_DIR}/pcl_modules")
        # pcl_modules 会再次 find_package(PCL)，把 registration 组件补回来
        find_package(PCL 1.11 REQUIRED COMPONENTS
            common io kdtree octree search sample_consensus
            filters features ml segmentation surface registration)
    endif()

    # small_gicp：
    #   1) 已安装: find_package(small_gicp)
    #   2) 源码目录: -DSMALL_GICP_DIR="C:/src/small_gicp"
    #      只需该目录下有 include/small_gicp（头文件即可）
    find_package(small_gicp QUIET)
    if(NOT small_gicp_FOUND)
        set(SMALL_GICP_DIR "" CACHE PATH "Path to small_gicp source or install prefix")
        if(SMALL_GICP_DIR)
            if(EXISTS "${SMALL_GICP_DIR}/include/small_gicp")
                set(_small_gicp_include "${SMALL_GICP_DIR}/include")
            elseif(EXISTS "${SMALL_GICP_DIR}/small_gicp")
                set(_small_gicp_include "${SMALL_GICP_DIR}")
            endif()
        endif()
        if(NOT _small_gicp_include)
            message(FATAL_ERROR
                "未找到 small_gicp。pcd_utils 需要 small_gicp（voxelgrid_sampling_omp）。\n"
                "请先克隆 https://github.com/koide3/small_gicp 并设置:\n"
                "  cmake -DSMALL_GICP_DIR=\"C:/src/small_gicp\" ..\n"
                "或安装后设置 small_gicp_DIR。\n"
                "若不需要 pcd_utils，可加 -DFILTERREG_BUILD_PCD_UTILS=OFF")
        endif()
        add_library(small_gicp INTERFACE)
        add_library(small_gicp::small_gicp ALIAS small_gicp)
        target_include_directories(small_gicp INTERFACE "${_small_gicp_include}")
        message(STATUS "Using small_gicp include: ${_small_gicp_include}")
    endif()

    if(TARGET small_gicp::small_gicp)
        set(_SMALL_GICP_TARGET small_gicp::small_gicp)
    elseif(TARGET small_gicp)
        set(_SMALL_GICP_TARGET small_gicp)
    else()
        message(FATAL_ERROR "small_gicp 已找到，但没有可用的 CMake target。")
    endif()

    add_library(pcd_utils STATIC
        src/pcd_utils.cpp
        include/pcd_utils.h
    )
    add_library(filterreg::pcd_utils ALIAS pcd_utils)

    target_include_directories(pcd_utils
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<BUILD_INTERFACE:${PCL_MODULES_DIR}/include>
            $<BUILD_INTERFACE:${PCL_MODULES_DIR}/include/base>
            $<INSTALL_INTERFACE:include>
            ${PCL_INCLUDE_DIRS}
            ${OpenCV_INCLUDE_DIRS}
    )

    if(PCL_LIBRARY_DIRS)
        target_link_directories(pcd_utils PUBLIC ${PCL_LIBRARY_DIRS})
    endif()
    if(PCL_DEFINITIONS)
        target_compile_definitions(pcd_utils PUBLIC ${PCL_DEFINITIONS})
    endif()

    target_link_libraries(pcd_utils
        PUBLIC
            filterreg
            ct_pcl_modules
            ${PCL_LIBRARIES}
            ${OpenCV_LIBS}
            ${_SMALL_GICP_TARGET}
    )

    if(OpenMP_CXX_FOUND)
        target_link_libraries(pcd_utils PUBLIC OpenMP::OpenMP_CXX)
        message(STATUS "pcd_utils: OpenMP enabled")
    elseif(MSVC)
        target_compile_options(pcd_utils PRIVATE /openmp)
        message(STATUS "pcd_utils: enabling MSVC /openmp")
    endif()
endif()

# ---------------------------------------------------------------------------
# Examples
# ---------------------------------------------------------------------------
if(FILTERREG_BUILD_EXAMPLES)
    add_executable(example_filterreg examples/example_filterreg.cpp)
    target_link_libraries(example_filterreg PRIVATE filterreg)
    set_target_properties(example_filterreg PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    )

    if(FILTERREG_BUILD_PCD_UTILS)
        add_executable(example_pcd_utils examples/example_pcd_utils.cpp)
        target_link_libraries(example_pcd_utils PRIVATE pcd_utils)
        set_target_properties(example_pcd_utils PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
        )
    endif()
endif()

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
include(GNUInstallDirs)

install(TARGETS filterreg
    EXPORT filterregTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(FILES
    include/filterreg.h
    include/permutohedral.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

if(FILTERREG_BUILD_PCD_UTILS)
    install(TARGETS pcd_utils
        EXPORT filterregTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
    install(FILES include/pcd_utils.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
endif()
