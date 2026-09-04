# Locate or fetch GLEW and expose the GLEW::GLEW imported target.
#
# Windows notes:
# - Official GLEW zips do not ship CMake config files, so find_package(GLEW)
#   often fails even when include/ and lib/ are present.
# - CMake's FindGLEW looks at GLEW_INCLUDE_DIR / GLEW_LIBRARY (singular).
#   Users commonly pass GLEW_INCLUDE_DIRS / GLEW_LIBRARIES (plural), or a
#   directory instead of glew32.lib — both are accepted here.
# - GLEW_STATIC / GLEW_USE_STATIC_LIBS force a search for glew32s.lib. Shared
#   builds prefer glew32.lib + glew32.dll.
#
# Search order: CONFIG -> MODULE -> GLEW_ROOT/hints -> FetchContent.

function(m3t_glew_usable_path out_var value)
  if("${value}" STREQUAL "" OR "${value}" MATCHES "-NOTFOUND$")
    set(${out_var} "" PARENT_SCOPE)
  else()
    set(${out_var} "${value}" PARENT_SCOPE)
  endif()
endfunction()

function(m3t_define_glew_target include_dir library_file)
  if(TARGET GLEW::GLEW)
    return()
  endif()
  add_library(GLEW::GLEW UNKNOWN IMPORTED GLOBAL)
  set_target_properties(GLEW::GLEW PROPERTIES
    IMPORTED_LOCATION "${library_file}"
    INTERFACE_INCLUDE_DIRECTORIES "${include_dir}")
  if(library_file MATCHES "glew32s" OR library_file MATCHES "glews")
    set_property(TARGET GLEW::GLEW APPEND PROPERTY
      INTERFACE_COMPILE_DEFINITIONS GLEW_STATIC)
  endif()
  if(TARGET OpenGL::GL)
    set_property(TARGET GLEW::GLEW APPEND PROPERTY
      INTERFACE_LINK_LIBRARIES OpenGL::GL)
  endif()
endfunction()

macro(m3t_setup_glew)
  if(TARGET GLEW::GLEW)
    message(STATUS "GLEW already available: GLEW::GLEW")
  else()
    # Snapshot user/cache hints before find_package overwrites GLEW_* with -NOTFOUND.
    set(_m3t_saved_glew_roots "")
    set(_m3t_saved_include_hints "")
    set(_m3t_saved_library_hints "")
    set(_m3t_saved_library_file "")

    foreach(_m3t_var GLEW_ROOT GLEW_DIR GLEW_ROOT_DIR GLEW_PATH)
      m3t_glew_usable_path(_m3t_val "${${_m3t_var}}")
      if(_m3t_val)
        list(APPEND _m3t_saved_glew_roots "${_m3t_val}")
      endif()
      if(DEFINED ENV{${_m3t_var}} AND NOT "$ENV{${_m3t_var}}" STREQUAL "")
        list(APPEND _m3t_saved_glew_roots "$ENV{${_m3t_var}}")
      endif()
    endforeach()

    foreach(_m3t_var GLEW_INCLUDE_DIR GLEW_INCLUDE_DIRS)
      m3t_glew_usable_path(_m3t_val "${${_m3t_var}}")
      if(_m3t_val)
        list(APPEND _m3t_saved_include_hints "${_m3t_val}")
        get_filename_component(_m3t_parent "${_m3t_val}" DIRECTORY)
        list(APPEND _m3t_saved_glew_roots "${_m3t_parent}")
      endif()
    endforeach()

    foreach(_m3t_var GLEW_LIBRARY GLEW_LIBRARIES)
      m3t_glew_usable_path(_m3t_val "${${_m3t_var}}")
      if(_m3t_val)
        if(EXISTS "${_m3t_val}" AND NOT IS_DIRECTORY "${_m3t_val}")
          set(_m3t_saved_library_file "${_m3t_val}")
        elseif(IS_DIRECTORY "${_m3t_val}")
          list(APPEND _m3t_saved_library_hints "${_m3t_val}")
          get_filename_component(_m3t_parent "${_m3t_val}" DIRECTORY)
          list(APPEND _m3t_saved_glew_roots "${_m3t_parent}")
        endif()
      endif()
    endforeach()

    # 1) CMake CONFIG (vcpkg, conda, custom install)
    find_package(GLEW QUIET CONFIG)
    if(NOT TARGET GLEW::GLEW)
      # 2) CMake MODULE (builtin FindGLEW)
      find_package(GLEW QUIET MODULE)
    endif()

    if(TARGET GLEW::GLEW)
      get_target_property(_m3t_glew_loc GLEW::GLEW IMPORTED_LOCATION)
      if(NOT _m3t_glew_loc)
        get_target_property(_m3t_glew_loc GLEW::GLEW IMPORTED_LOCATION_RELEASE)
      endif()
      message(STATUS "Found GLEW via find_package: ${_m3t_glew_loc}")
    else()
      # 3) Manual search — honor the variables people actually set on Windows
      set(_m3t_glew_roots "${_m3t_saved_glew_roots}")
      set(_m3t_glew_include_hints "${_m3t_saved_include_hints}")
      set(_m3t_glew_library_hints "${_m3t_saved_library_hints}")
      set(_m3t_glew_library_file "${_m3t_saved_library_file}")

      if(WIN32)
        list(APPEND _m3t_glew_roots
          "C:/glew"
          "C:/GLEW"
          "C:/Tools/glew"
          "C:/Tools/glew-2.2.0"
          "C:/Tools/glew-2.1.0"
          "C:/Program Files/glew"
          "C:/Program Files/GLEW"
          "C:/Program Files (x86)/glew"
          "$ENV{PROGRAMFILES}/glew"
        )
      endif()

      find_path(M3T_GLEW_INCLUDE_DIR
        NAMES GL/glew.h
        HINTS ${_m3t_glew_include_hints} ${_m3t_glew_roots}
        PATH_SUFFIXES include
      )

      if(WIN32)
        if(BUILD_SHARED_LIBS)
          set(_m3t_glew_names glew32 glew GLEW glew32s glews)
        else()
          set(_m3t_glew_names glew32s glews glew32 glew GLEW)
        endif()
      else()
        set(_m3t_glew_names GLEW glew glew32)
      endif()

      if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(_m3t_glew_lib_suffixes
          lib/Release/x64
          lib/Debug/x64
          lib/x64
          lib64
          lib
          bin/Release/x64
          bin/x64
          lib/Release
          lib/Debug
        )
      else()
        set(_m3t_glew_lib_suffixes
          lib/Release/Win32
          lib/Debug/Win32
          lib/Win32
          lib
          bin/Release/Win32
          lib/Release
          lib/Debug
        )
      endif()

      if(NOT _m3t_glew_library_file)
        find_library(M3T_GLEW_LIBRARY
          NAMES ${_m3t_glew_names}
          HINTS ${_m3t_glew_library_hints} ${_m3t_glew_roots}
          PATH_SUFFIXES ${_m3t_glew_lib_suffixes}
        )
      else()
        set(M3T_GLEW_LIBRARY "${_m3t_glew_library_file}" CACHE FILEPATH "GLEW library" FORCE)
      endif()

      if(M3T_GLEW_INCLUDE_DIR AND M3T_GLEW_LIBRARY)
        m3t_define_glew_target("${M3T_GLEW_INCLUDE_DIR}" "${M3T_GLEW_LIBRARY}")
        message(STATUS "Found GLEW: ${M3T_GLEW_LIBRARY}")
        message(STATUS "GLEW include: ${M3T_GLEW_INCLUDE_DIR}")
      elseif(M3T_FETCH_GLEW)
        message(STATUS "GLEW not found on the system; downloading glew-cmake 2.2.0")
        include(FetchContent)
        set(glew-cmake_BUILD_SHARED ${BUILD_SHARED_LIBS} CACHE BOOL "" FORCE)
        if(BUILD_SHARED_LIBS)
          set(glew-cmake_BUILD_STATIC OFF CACHE BOOL "" FORCE)
        else()
          set(glew-cmake_BUILD_STATIC ON CACHE BOOL "" FORCE)
        endif()
        set(ONLY_LIBS ON CACHE BOOL "" FORCE)
        FetchContent_Declare(
          glew
          GIT_REPOSITORY https://github.com/Perlmint/glew-cmake.git
          GIT_TAG glew-cmake-2.2.0
          GIT_SHALLOW TRUE
        )
        FetchContent_MakeAvailable(glew)

        if(BUILD_SHARED_LIBS AND TARGET libglew_shared)
          add_library(GLEW::GLEW ALIAS libglew_shared)
        elseif(TARGET libglew_static)
          add_library(GLEW::GLEW ALIAS libglew_static)
        elseif(TARGET libglew_shared)
          add_library(GLEW::GLEW ALIAS libglew_shared)
        elseif(TARGET GLEW::glew)
          add_library(GLEW::GLEW ALIAS GLEW::glew)
        else()
          message(FATAL_ERROR "Fetched GLEW but no usable CMake target was created")
        endif()
        message(STATUS "Using FetchContent GLEW (glew-cmake 2.2.0)")
      else()
        message(FATAL_ERROR
          "Could not find GLEW.\n"
          "Set GLEW_ROOT to the GLEW install prefix, or pass:\n"
          "  -DGLEW_INCLUDE_DIR=<prefix>/include\n"
          "  -DGLEW_LIBRARY=<prefix>/lib/Release/x64/glew32.lib\n"
          "Windows official zip layout:\n"
          "  <root>/include/GL/glew.h\n"
          "  <root>/lib/Release/x64/glew32.lib\n"
          "  <root>/bin/Release/x64/glew32.dll\n"
          "Alternatively enable M3T_FETCH_GLEW (default ON) to download GLEW.")
      endif()
    endif()
  endif()
endmacro()
