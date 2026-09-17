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

焊接作业通过 Ethernet KRL 与 KR C 交换工艺与状态。信号与焊缝工艺表（内缩、速度、摆动、多层多道）对齐，共 101 项。

- 规范：[`docs/kuka_weld_comm.md`](docs/kuka_weld_comm.md)
- 通讯表 CSV：[`docs/kuka_weld_comm_table.csv`](docs/kuka_weld_comm_table.csv)
- EKI 配置：[`kuka/EthernetKRL/WeldHost.xml`](kuka/EthernetKRL/WeldHost.xml)
- 数据结构：`include/KukaWeldComm.h`

```bash
make test      # 编解码与通讯表一致性
make docs      # 重新导出 CSV 与 WeldHost.xml
```
