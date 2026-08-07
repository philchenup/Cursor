cmake_minimum_required(VERSION 3.16)
project(pcl_hidden_point_removal LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Distro VTK/PCL packages may reference MPI::MPI_C even when unused.
find_package(MPI COMPONENTS C QUIET)
if(NOT TARGET MPI::MPI_C)
  add_library(MPI::MPI_C INTERFACE IMPORTED)
endif()

find_package(PCL 1.12 REQUIRED COMPONENTS common surface io)

# pcl::ConvexHull needs Qhull at link time. On MSVC this commonly surfaces as:
#   LNK2001: unresolved external symbol qh_new_qhull / qh_freeqhull / ...
# Explicitly locate Qhull so dependents inherit the correct .lib.
set(_HPR_QHULL_TARGET "")
if(TARGET QHULL::QHULL)
  set(_HPR_QHULL_TARGET QHULL::QHULL)
elseif(TARGET Qhull::qhull_r)
  set(_HPR_QHULL_TARGET Qhull::qhull_r)
elseif(TARGET qhull::qhull_r)
  set(_HPR_QHULL_TARGET qhull::qhull_r)
else()
  find_package(Qhull QUIET)
  if(TARGET QHULL::QHULL)
    set(_HPR_QHULL_TARGET QHULL::QHULL)
  elseif(TARGET Qhull::qhull_r)
    set(_HPR_QHULL_TARGET Qhull::qhull_r)
  elseif(TARGET qhull::qhull_r)
    set(_HPR_QHULL_TARGET qhull::qhull_r)
  else()
    find_library(HPR_QHULL_LIBRARY
      NAMES qhull_r qhullstatic_r qhull qhullstatic
      PATHS
        "${PCL_LIBRARY_DIRS}"
        "${PCL_ROOT}/lib"
        "$ENV{PCL_ROOT}/lib"
        "$ENV{QHULL_ROOT}/lib"
    )
    if(HPR_QHULL_LIBRARY)
      add_library(HPR::Qhull UNKNOWN IMPORTED)
      set_target_properties(HPR::Qhull PROPERTIES
        IMPORTED_LOCATION "${HPR_QHULL_LIBRARY}")
      set(_HPR_QHULL_TARGET HPR::Qhull)
      message(STATUS "HiddenPointRemoval: linking Qhull via ${HPR_QHULL_LIBRARY}")
    else()
      message(WARNING
        "HiddenPointRemoval: Qhull library not found. "
        "MSVC builds will fail with LNK2001 on qh_* symbols. "
        "Install Qhull and link qhull_r.lib / qhullstatic_r.lib.")
    endif()
  endif()
endif()

add_library(pcl_hidden_point_removal INTERFACE)
target_include_directories(pcl_hidden_point_removal INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(pcl_hidden_point_removal INTERFACE ${PCL_LIBRARIES})
if(_HPR_QHULL_TARGET)
  target_link_libraries(pcl_hidden_point_removal INTERFACE ${_HPR_QHULL_TARGET})
endif()
if(PCL_DEFINITIONS)
  target_compile_definitions(pcl_hidden_point_removal INTERFACE ${PCL_DEFINITIONS})
endif()

add_executable(hidden_point_removal_example
  examples/hidden_point_removal_example.cpp
)
target_link_libraries(hidden_point_removal_example PRIVATE pcl_hidden_point_removal)

install(FILES include/hidden_point_removal.h DESTINATION include)
