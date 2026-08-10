# FindOrLinkLZ4.cmake
# Helper for MSVC/PCL projects that hit LNK2001 on:
#   LZ4_resetStreamHC, LZ4_setStreamDecode, LZ4_decompress_safe_continue,
#   LZ4_decompress_safe, LZ4_compress_HC_continue
#
# Usage:
#   include(cmake/FindOrLinkLZ4.cmake)
#   target_link_lz4(YourTarget)

function(target_link_lz4 _target)
  if(TARGET lz4::lz4)
    target_link_libraries(${_target} PRIVATE lz4::lz4)
    return()
  endif()

  find_package(lz4 CONFIG QUIET)
  if(TARGET lz4::lz4)
    target_link_libraries(${_target} PRIVATE lz4::lz4)
    return()
  endif()

  find_library(LZ4_LIBRARY
    NAMES lz4 liblz4 lz4_static liblz4_static
    PATHS
      ENV LZ4_ROOT
      "${LZ4_ROOT}"
      "${CMAKE_PREFIX_PATH}"
    PATH_SUFFIXES lib lib64
  )

  if(NOT LZ4_LIBRARY)
    message(FATAL_ERROR
      "LZ4 not found. Install lz4 (e.g. vcpkg install lz4:x64-windows) "
      "or set LZ4_ROOT / CMAKE_PREFIX_PATH so lz4.lib is visible. "
      "Required when FLANN/PCL kdtree pulls in LZ4 serialization APIs.")
  endif()

  message(STATUS "Linking LZ4: ${LZ4_LIBRARY}")
  target_link_libraries(${_target} PRIVATE ${LZ4_LIBRARY})
endfunction()
