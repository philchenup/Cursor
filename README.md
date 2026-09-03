# Cursor

## FormatCommData

按 `CommConfig` 把 `std::vector<std::vector<float>>` 格式化成通信字符串。内层 vector 是组内，外层是组间；每个 float 按 `valid_num` 位有效数字转换。

```cpp
#include "CommFormat.h"

CommConfig cfg;                 // ",", ";", ".", 3 位有效数字
std::vector<std::vector<float>> groups = {
    {1.234f, 5.678f},
    {9.01f}
};

QString payload = FormatCommData(groups, cfg);
// "1.23,5.68;9.01."
```

需要 Qt Core。把 `include/CommFormat.h` 和 `src/CommFormat.cpp` 加入工程即可；本地可用 `make test` 验证。

## ScaleAISShapeBy1000

将 OpenCASCADE 的 `AIS_Shape*` 缩小 1000 倍，并返回一个新的 `AIS_Shape*`。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
AIS_Shape* scaled = ScaleAISShapeBy1000(ais);

// 推荐用 Handle 接管返回值，避免泄漏
Handle(AIS_Shape) scaledHandle = ScaleAISShapeBy1000(ais);
```
