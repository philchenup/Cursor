# target_use_winsock_compat(<target>)
#
# Apply the WinSock1/WinSock2 include-order fix to a Windows target
# (typically the Qt "mainwindow" executable). No-op on non-Windows.

get_filename_component(_WINSOCK_COMPAT_INCLUDE_DIR
    "${CMAKE_CURRENT_LIST_DIR}/../include" ABSOLUTE)

function(target_use_winsock_compat target)
    if(NOT WIN32)
        return()
    endif()

    set(_winsock_compat_include_dir "${_WINSOCK_COMPAT_INCLUDE_DIR}")

    target_compile_definitions(${target} PRIVATE
        WIN32_LEAN_AND_MEAN
        NOMINMAX
        _WINSOCK_DEPRECATED_NO_WARNINGS
    )
    target_include_directories(${target} PRIVATE "${_winsock_compat_include_dir}")

    if(MSVC)
        target_compile_options(${target} PRIVATE
            "/FI${_winsock_compat_include_dir}/WinSockCompat.h"
        )
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE
            "-include" "${_winsock_compat_include_dir}/WinSockCompat.h"
        )
    endif()
endfunction()
