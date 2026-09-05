# Cursor

## WinSock E0338 (`__WSAFDIsSet` C 链接冲突)

MSVC 错误列表里 `mainwindow` + `WinSock2.h` 的 **E0338** 是 `winsock.h` 与 `WinSock2.h` 重复声明导致的。用法见 [README_WinSock.md](README_WinSock.md)，把头文件 `include/WinSockCompat.h` 放在 `mainwindow.h` / PCH 的最前面，或用 `winsock_compat.pri` / `cmake/WinSockCompat.cmake` 强制包含。

## ScaleAISShapeBy1000

将 OpenCASCADE 的 `AIS_Shape*` 缩小 1000 倍，并返回一个新的 `AIS_Shape*`。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
AIS_Shape* scaled = ScaleAISShapeBy1000(ais);

// 推荐用 Handle 接管返回值，避免泄漏
Handle(AIS_Shape) scaledHandle = ScaleAISShapeBy1000(ais);
```
