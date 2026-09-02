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

## 开发环境 / Building

本项目依赖 OpenCASCADE (OCCT) 与 CMake。在 Ubuntu 上一次性配置并构建：

```bash
bash .cursor/install.sh
```

该脚本会安装 OCCT 开发库及构建工具，然后配置并编译项目（使用 `g++`）。

单独构建与运行端到端演示：

```bash
cmake -S . -B build -DCMAKE_CXX_COMPILER=g++
cmake --build build -j"$(nproc)"
./build/scale_demo   # 构建一个盒体，缩小 1000 倍并校验结果
ctest --test-dir build --output-on-failure
```

