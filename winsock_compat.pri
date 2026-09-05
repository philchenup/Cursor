# Include from the Qt .pro that builds the mainwindow target:
#   include($$PWD/winsock_compat.pri)
#
# Stops MSVC E0338 / C2733 on __WSAFDIsSet (WinSock vs WinSock2).

INCLUDEPATH += $$PWD/include

win32 {
    DEFINES += WIN32_LEAN_AND_MEAN
    DEFINES += NOMINMAX
    DEFINES += _WINSOCK_DEPRECATED_NO_WARNINGS
}

win32-msvc {
    # Forced include: WinSock2 is seen before Qt/OCC/PCL in every TU,
    # including moc_*.cpp. Quote the path for spaces in $$PWD.
    QMAKE_CXXFLAGS += /FI$$shell_quote($$PWD/include/WinSockCompat.h)
}

win32-g++ {
    QMAKE_CXXFLAGS += -include $$shell_quote($$PWD/include/WinSockCompat.h)
}
