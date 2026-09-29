cmake_minimum_required(VERSION 3.16)

project(m3t_realsense_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_SHARED_LIBS "Build m3t as a shared library (m3t.dll on Windows)" ON)
option(USE_XFEATURES2D "Use OpenCV xfeatures2d (optional texture descriptors)" OFF)
option(M3T_FETCH_GLEW "Download GLEW via FetchContent if it is not found" ON)
option(M3T_FETCH_GLFW "Download GLFW via FetchContent if it is not found" ON)
option(M3T_FETCH_EIGEN "Download Eigen via FetchContent if it is not found" ON)
set(M3T_SOURCE_DIR "" CACHE PATH
    "Optional path to 3DObjectTracking or its M3T subdirectory (skips git clone)")

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

# Windows: use the DLL CRT so OpenCV / RealSense / GLEW all share /MD.
# Official M3T CMake forces GLEW_STATIC + OpenCV_STATIC + /MT, which breaks
# typical Windows installs. This project builds M3T sources itself instead.
if(WIN32)
  set(OpenCV_STATIC OFF)
  set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
  if(BUILD_SHARED_LIBS)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
  endif()
endif()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
foreach(_cfg DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${_cfg} ${CMAKE_BINARY_DIR}/lib)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${_cfg} ${CMAKE_BINARY_DIR}/lib)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${_cfg} ${CMAKE_BINARY_DIR}/bin)
endforeach()

if(MSVC)
  add_compile_options(/utf-8 /bigobj)
  add_compile_definitions(NOMINMAX _USE_MATH_DEFINES)
endif()

include(FetchContent)

find_package(OpenMP REQUIRED)
if(UNIX)
  set(OpenGL_GL_PREFERENCE LEGACY)
endif()
find_package(OpenGL REQUIRED)

# Eigen is a header-only M3T dependency. PCL (already installed on the
# target machine) commonly exposes it; otherwise FetchContent.
find_package(Eigen3 3.3 QUIET NO_MODULE)
if(NOT Eigen3_FOUND)
  find_package(PCL QUIET COMPONENTS common)
  if(PCL_FOUND AND PCL_INCLUDE_DIRS AND NOT TARGET Eigen3::Eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    target_include_directories(Eigen3::Eigen INTERFACE ${PCL_INCLUDE_DIRS})
    message(STATUS "Using Eigen headers from PCL: ${PCL_INCLUDE_DIRS}")
  elseif(M3T_FETCH_EIGEN)
    FetchContent_Declare(
      eigen
      GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
      GIT_TAG 3.4.0
      GIT_SHALLOW TRUE
    )
    FetchContent_GetProperties(eigen)
    if(NOT eigen_POPULATED)
      FetchContent_Populate(eigen)
    endif()
    if(NOT TARGET Eigen3::Eigen)
      add_library(Eigen3::Eigen INTERFACE IMPORTED)
      target_include_directories(Eigen3::Eigen INTERFACE
        ${eigen_SOURCE_DIR})
    endif()
    message(STATUS "Using FetchContent Eigen 3.4.0")
  else()
    message(FATAL_ERROR "Eigen3 not found. Install Eigen, or enable M3T_FETCH_EIGEN.")
  endif()
endif()

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(M3TSetupGLEW)
m3t_setup_glew()

find_package(glfw3 CONFIG QUIET)
if(NOT glfw3_FOUND AND NOT TARGET glfw)
  find_package(glfw3 QUIET)
endif()
if(NOT glfw3_FOUND AND NOT TARGET glfw)
  if(NOT M3T_FETCH_GLFW)
    message(FATAL_ERROR "glfw3 not found. Install GLFW or enable M3T_FETCH_GLFW.")
  endif()
  FetchContent_Declare(
    glfw3
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
    GIT_SHALLOW TRUE
  )
  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(glfw3)
endif()

set(OpenCV_COMPONENTS core imgproc highgui imgcodecs calib3d features2d)
if(USE_XFEATURES2D)
  add_compile_definitions(USE_XFEATURES2D=TRUE)
  list(APPEND OpenCV_COMPONENTS xfeatures2d)
endif()
find_package(OpenCV 4 REQUIRED COMPONENTS ${OpenCV_COMPONENTS}
             OPTIONAL_COMPONENTS cudafeatures2d)

find_package(realsense2 REQUIRED)
if(TARGET realsense2::realsense2)
  set(M3T_REALSENSE_LIB realsense2::realsense2)
elseif(TARGET realsense2)
  set(M3T_REALSENSE_LIB realsense2)
else()
  set(M3T_REALSENSE_LIB ${realsense2_LIBRARY})
endif()

# ---------------------------------------------------------------------------
# M3T sources (pym3t tracks agimus-project/3DObjectTracking)
# ---------------------------------------------------------------------------
function(m3t_resolve_root candidate out_var)
  if(EXISTS "${candidate}/include/m3t/tracker.h")
    set(${out_var} "${candidate}" PARENT_SCOPE)
  elseif(EXISTS "${candidate}/M3T/include/m3t/tracker.h")
    set(${out_var} "${candidate}/M3T" PARENT_SCOPE)
  else()
    set(${out_var} "" PARENT_SCOPE)
  endif()
endfunction()

set(M3T_ROOT "")
if(M3T_SOURCE_DIR)
  m3t_resolve_root("${M3T_SOURCE_DIR}" M3T_ROOT)
  if(NOT M3T_ROOT)
    message(FATAL_ERROR
      "M3T_SOURCE_DIR=${M3T_SOURCE_DIR} does not contain include/m3t/tracker.h")
  endif()
else()
  FetchContent_Declare(
    m3t_repo
    GIT_REPOSITORY https://github.com/agimus-project/3DObjectTracking.git
    GIT_TAG master
    GIT_SHALLOW TRUE
  )
  FetchContent_GetProperties(m3t_repo)
  if(NOT m3t_repo_POPULATED)
    FetchContent_Populate(m3t_repo)
  endif()
  m3t_resolve_root("${m3t_repo_SOURCE_DIR}" M3T_ROOT)
endif()
message(STATUS "M3T sources: ${M3T_ROOT}")

set(M3T_SOURCES
  ${M3T_ROOT}/src/common.cpp
  ${M3T_ROOT}/src/body.cpp
  ${M3T_ROOT}/src/renderer_geometry.cpp
  ${M3T_ROOT}/src/renderer.cpp
  ${M3T_ROOT}/src/normal_renderer.cpp
  ${M3T_ROOT}/src/silhouette_renderer.cpp
  ${M3T_ROOT}/src/basic_depth_renderer.cpp
  ${M3T_ROOT}/src/model.cpp
  ${M3T_ROOT}/src/region_model.cpp
  ${M3T_ROOT}/src/depth_model.cpp
  ${M3T_ROOT}/src/camera.cpp
  ${M3T_ROOT}/src/loader_camera.cpp
  ${M3T_ROOT}/src/viewer.cpp
  ${M3T_ROOT}/src/image_viewer.cpp
  ${M3T_ROOT}/src/normal_viewer.cpp
  ${M3T_ROOT}/src/color_histograms.cpp
  ${M3T_ROOT}/src/modality.cpp
  ${M3T_ROOT}/src/region_modality.cpp
  ${M3T_ROOT}/src/depth_modality.cpp
  ${M3T_ROOT}/src/texture_modality.cpp
  ${M3T_ROOT}/src/link.cpp
  ${M3T_ROOT}/src/constraint.cpp
  ${M3T_ROOT}/src/soft_constraint.cpp
  ${M3T_ROOT}/src/optimizer.cpp
  ${M3T_ROOT}/src/detector.cpp
  ${M3T_ROOT}/src/static_detector.cpp
  ${M3T_ROOT}/src/manual_detector.cpp
  ${M3T_ROOT}/src/refiner.cpp
  ${M3T_ROOT}/src/publisher.cpp
  ${M3T_ROOT}/src/subscriber.cpp
  ${M3T_ROOT}/src/tracker.cpp
  ${M3T_ROOT}/src/realsense_camera.cpp
)

add_library(m3t ${M3T_SOURCES})
add_library(M3T::m3t ALIAS m3t)
target_compile_definitions(m3t PUBLIC USE_REALSENSE=TRUE)
if(OpenCV_CUDA_VERSION)
  target_compile_definitions(m3t PUBLIC USE_CUDA=TRUE)
endif()

set(_m3t_glfw_target glfw)
if(NOT TARGET glfw AND TARGET glfw3)
  set(_m3t_glfw_target glfw3)
endif()

target_link_libraries(m3t PUBLIC
  Eigen3::Eigen
  OpenGL::GL
  GLEW::GLEW
  ${_m3t_glfw_target}
  ${OpenCV_LIBS}
  OpenMP::OpenMP_CXX
  ${M3T_REALSENSE_LIB}
)
if(UNIX)
  target_link_libraries(m3t PUBLIC pthread stdc++fs)
endif()

target_include_directories(m3t PUBLIC
  $<BUILD_INTERFACE:${M3T_ROOT}/include>
  $<BUILD_INTERFACE:${M3T_ROOT}/third_party>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(m3t PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  WINDOWS_EXPORT_ALL_SYMBOLS ON
  OUTPUT_NAME m3t
)

add_executable(run_realsense_example src/run_realsense_example.cpp)
target_include_directories(run_realsense_example PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(run_realsense_example PRIVATE m3t)
if(WIN32)
  target_link_libraries(run_realsense_example PRIVATE shell32)
endif()

if(WIN32 AND TARGET ${M3T_REALSENSE_LIB})
  add_custom_command(TARGET run_realsense_example POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      $<TARGET_FILE:${M3T_REALSENSE_LIB}>
      $<TARGET_FILE_DIR:run_realsense_example>
    COMMENT "Copy RealSense runtime next to run_realsense_example")
endif()

enable_testing()
add_executable(test_run_realsense_options tests/test_run_realsense_options.cpp)
target_include_directories(test_run_realsense_options PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME test_run_realsense_options COMMAND test_run_realsense_options)
