cmake_minimum_required(VERSION 3.14)

# A Multi-Body Multi-Modality Multi-Camera 3D Tracker (M3T)
# =============================================================================
project(M3T)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)   # find_package uses <PackageName>_ROOT
endif()
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)   # also honors UPPER_CASE <PACKAGENAME>_ROOT
endif()


# Options
# =============================================================================
option(BUILD_SHARED_LIBS "Build shared libraries (.dll / .so) instead of static (.lib / .a)" ON)
option(USE_AZURE_KINECT "Use Azure Kinect" OFF)
option(USE_REALSENSE "Use RealSense D435" ON)
option(USE_GTEST "Use gtest" OFF)
option(USE_XFEATURES2D "Use OpenCV xfeatures2d" OFF)
option(M3T_FETCH_GLEW "Download GLEW via FetchContent if it is not found" ON)
option(M3T_FETCH_GLFW "Download GLFW via FetchContent if it is not found" ON)


# Libraries
# =============================================================================

# Windows: link the DLL CRT (/MD) so the tracker, OpenCV, and GLEW all use the
# same runtime. Forcing GLEW_STATIC makes FindGLEW look for glew32s.lib and is
# the usual reason GLEW is "not found" when only glew32.lib/.dll is installed.
if (WIN32)
  if(NOT BUILD_SHARED_LIBS)
    set(GLEW_STATIC TRUE)
    set(GLEW_USE_STATIC_LIBS TRUE)
  endif()
  set(OpenCV_STATIC false)
  set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
  if(MSVC)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD")
  endif()
  # Export C++ symbols so a SHARED m3t produces both m3t.dll and m3t.lib
  if(BUILD_SHARED_LIBS)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
  endif()
endif ()

# Shared objects and import libraries land next to the example executables so
# Windows can load m3t.dll without extra PATH setup.
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
foreach(_m3t_cfg DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${_m3t_cfg} ${CMAKE_BINARY_DIR}/lib)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${_m3t_cfg} ${CMAKE_BINARY_DIR}/lib)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${_m3t_cfg} ${CMAKE_BINARY_DIR}/bin)
endforeach()

# Used libraries
find_package(OpenMP REQUIRED)
find_package(Eigen3 3.4.0 REQUIRED)
if (UNIX)
    set(OpenGL_GL_PREFERENCE LEGACY)
endif ()
find_package(OpenGL REQUIRED)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(M3TSetupGLEW)
m3t_setup_glew()

# Prefer a CMake package, then MODULE/pkg-config, then FetchContent.
find_package(glfw3 CONFIG QUIET)
if(NOT glfw3_FOUND AND NOT TARGET glfw)
  find_package(glfw3 QUIET)
endif()
if(NOT glfw3_FOUND AND NOT TARGET glfw)
  if(NOT M3T_FETCH_GLFW)
    message(FATAL_ERROR "glfw3 not found. Install GLFW or enable M3T_FETCH_GLFW.")
  endif()
  include(FetchContent)
  FetchContent_Declare(
    glfw3
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.5.1
    GIT_SHALLOW TRUE
  )
  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(glfw3)
endif()

set(OpenCV_COMPONENTS core imgproc highgui imgcodecs calib3d features2d)
if(USE_XFEATURES2D)
    add_definitions( -DUSE_XFEATURES2D=TRUE )
    set(OpenCV_COMPONENTS ${OpenCV_COMPONENTS} xfeatures2d)
endif()

find_package(OpenCV 4.12.0 REQUIRED COMPONENTS ${OpenCV_COMPONENTS} OPTIONAL_COMPONENTS cudafeatures2d)

if (USE_AZURE_KINECT)
    find_package(k4a 1.3.0 REQUIRED)
endif ()

if (USE_REALSENSE)
    find_package(realsense2 REQUIRED)
endif ()

find_package(OpenMP)
if (OPENMP_FOUND)
    if (WIN32)
        set(CMAKE_CXX_FLAGS_RELEASE
                "${CMAKE_CXX_FLAGS_RELEASE} -openmp:experimental")
        set(CMAKE_C_FLAGS_RELEASE
                "${CMAKE_C_FLAGS_RELEASE} ${OpenMP_C_FLAGS}")
        set(CMAKE_CXX_FLAGS_RELEASE
                "${CMAKE_CXX_FLAGS_RELEASE} ${OpenMP_CXX_FLAGS}")
        set(CMAKE_EXE_LINKER_FLAGS
                "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    else ()
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        set(CMAKE_EXE_LINKER_FLAGS
                "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    endIf ()
endif ()


# Compiler Settings
# =============================================================================

# Default settings
set(CMAKE_CXX_STANDARD 17)
# Hidden visibility would hide DLL exports on GCC/MinGW shared builds.
if(NOT BUILD_SHARED_LIBS)
    set(CMAKE_CXX_VISIBILITY_PRESET hidden)
endif()
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-march HAS_MARCH)
check_cxx_compiler_flag(-mtune HAS_MTUNE)
if (HAS_MARCH)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
endif ()
if (HAS_MTUNE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mtune=native")
endif ()

# Debug settings
check_cxx_compiler_flag(-ggdb HAS_GGDB)
check_cxx_compiler_flag(-Z7 HAS_Z7)
check_cxx_compiler_flag(-ftrapv HAS_FTRAPV)
check_cxx_compiler_flag(-Od HAS_OD)
if (HAS_GGDB)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ggdb")
elseif (HAS_Z7)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Z7")
endif ()
if (HAS_FTRAPV)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ftrapv")
endif ()
if (HAS_OD)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Od")
endif ()
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

# Release settings
check_cxx_compiler_flag(-Ob3 HAS_OB3)
check_cxx_compiler_flag(-O3 HAS_O3)
check_cxx_compiler_flag(-Ob2 HAS_OB2)
check_cxx_compiler_flag(-O2 HAS_O2)
if (HAS_OB3)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Ob3")
elseif (HAS_O3)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
elseif (HAS_OB2)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Ob2")
elseif (HAS_O2)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")
endif ()
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG")


# Definitions
# =============================================================================
if (USE_AZURE_KINECT)
    add_definitions( -DUSE_AZURE_KINECT=TRUE )
endif ()
if (USE_REALSENSE)
    add_definitions( -DUSE_REALSENSE=TRUE )
endif ()
if(OpenCV_CUDA_VERSION)
    add_definitions( -DUSE_CUDA=TRUE )
endif ()
if (USE_GTEST)
    add_definitions( -DUSE_GTEST=TRUE )
endif ()


# Directories
# =============================================================================
add_subdirectory(src)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/CMakeLists.txt")
    add_subdirectory(test)
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/examples/CMakeLists.txt")
    add_subdirectory(examples)
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/doc/CMakeLists.txt")
    add_subdirectory(doc)
endif()
