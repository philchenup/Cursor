# Cursor

## ScaleAISShapeBy1000

将 OpenCASCADE 的 `AIS_Shape*` 缩小 1000 倍，并返回一个新的 `AIS_Shape*`。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
AIS_Shape* scaled = ScaleAISShapeBy1000(ais);

// 推荐用 Handle 接管返回值，避免泄漏
Handle(AIS_Shape) scaledHandle = ScaleAISShapeBy1000(ais);
```

## M3T Windows CMake

`m3t/` 是 M3T 工程的 CMake 补丁：Windows 默认生成 `m3t.dll`（不再只有 `.lib`），并修复 GLEW 查找（官方 zip / `GLEW_ROOT` / 自动 FetchContent）。用法见 `m3t/README.md`。
