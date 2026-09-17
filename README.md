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

## 上位机 ↔ KUKA 焊接通讯表

通讯表为两列：**数据类型**（FLOAT32 / INT32 / BOOL）+ **信号名**（如 `Extern_Speed`、`Robot_X`、`Robot_J6`），与 PLC 映射表相同。前 19 项为运动字，其后为焊接工艺字，共 120 项。

- 规范：[`docs/kuka_weld_comm.md`](docs/kuka_weld_comm.md)
- 通讯表：[`docs/kuka_weld_comm_table.md`](docs/kuka_weld_comm_table.md) / [`.csv`](docs/kuka_weld_comm_table.csv)
- EKI：[`kuka/EthernetKRL/WeldHost.xml`](kuka/EthernetKRL/WeldHost.xml)

```bash
make test
make docs
```

