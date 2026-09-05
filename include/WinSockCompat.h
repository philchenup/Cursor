#ifndef WINSOCK_COMPAT_H
#define WINSOCK_COMPAT_H

/**
 * Fix MSVC E0338 / C2733:
 *   重载函数 "__WSAFDIsSet" 的多个实例包含“C”链接
 *   WinSock2.h
 *
 * Root cause:
 *   windows.h (via Qt, OpenCASCADE, PCL, …) includes the old winsock.h,
 *   which declares __WSAFDIsSet with C linkage. A later include of
 *   WinSock2.h (Qt Network, JAKA SDK, Boost.Asio, rl::hal::Socket, …)
 *   redeclares the same C function → E0338 / C2733.
 *
 * This header MUST be processed before any of those libraries:
 *   - first #include in mainwindow.h / pch.h / stdafx.h, or
 *   - forced include (/FI or -include) on the mainwindow target.
 */

#if defined(_WIN32) || defined(_WIN64) || defined(WIN32)

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#ifndef NOMINMAX
#define NOMINMAX
#endif

#ifndef _WINSOCK_DEPRECATED_NO_WARNINGS
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#endif

/* WinSock2.h defines _WINSOCKAPI_ then includes windows.h, so the
 * legacy winsock.h is never pulled in. Do not include windows.h first. */
#include <winsock2.h>
#include <ws2tcpip.h>

#endif /* _WIN32 */

#endif /* WINSOCK_COMPAT_H */
