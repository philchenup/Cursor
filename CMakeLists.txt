cmake_minimum_required(VERSION 3.16)
project(CylindricalHoleDetector LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_EXAMPLE "Build CLI example" ON)

# ---------------------------------------------------------------------------
# Dependencies: OpenCASCADE (>= 7.4) and PCL
# ---------------------------------------------------------------------------
find_package(OpenCASCADE REQUIRED)
if(DEFINED OpenCASCADE_VERSION AND OpenCASCADE_VERSION VERSION_LESS "7.4.0")
  message(FATAL_ERROR
    "OpenCASCADE >= 7.4.0 required (found ${OpenCASCADE_VERSION})")
endif()

find_package(PCL 1.10 REQUIRED COMPONENTS common io)

add_library(cylindrical_hole_detector
  src/CylindricalHoleDetector.cpp
)
add_library(occt::cylindrical_hole_detector ALIAS cylindrical_hole_detector)

target_include_directories(cylindrical_hole_detector
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCASCADE_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

# OCCT installs libraries under OpenCASCADE_LIBRARY_DIR on most packages.
if(DEFINED OpenCASCADE_LIBRARY_DIR)
  target_link_directories(cylindrical_hole_detector PUBLIC ${OpenCASCADE_LIBRARY_DIR})
endif()

target_link_libraries(cylindrical_hole_detector
  PUBLIC
    ${PCL_LIBRARIES}
    TKDESTEP
    TKMesh
    TKTopAlgo
    TKGeomAlgo
    TKGeomBase
    TKBRep
    TKMath
    TKG3d
    TKG2d
    TKernel
    TKXSBase
    TKShHealing
)

if(PCL_DEFINITIONS)
  target_compile_definitions(cylindrical_hole_detector PUBLIC ${PCL_DEFINITIONS})
endif()

if(BUILD_EXAMPLE)
  add_executable(detect_holes examples/main.cpp)
  target_link_libraries(detect_holes PRIVATE cylindrical_hole_detector)
endif()

install(TARGETS cylindrical_hole_detector
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
