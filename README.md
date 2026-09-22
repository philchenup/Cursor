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

通讯表仅含焊接总流程必要项（参考起终点、焊枪姿态、焊速、摆动、寻缝命令与找到的点），共 39 项。焊中纠偏走 RSI。KUKA→上位机读取数据打包 **64** 字节，上位机→KUKA 下发 92 字节。

- 规范：[`docs/kuka_weld_comm.md`](docs/kuka_weld_comm.md)
- 通讯表：[`docs/kuka_weld_comm_table.md`](docs/kuka_weld_comm_table.md) / [`.csv`](docs/kuka_weld_comm_table.csv)
- EKI：[`kuka/EthernetKRL/WeldHost.xml`](kuka/EthernetKRL/WeldHost.xml)

## PROFINET 读写外部轴 / 关节 / TCP

CIFX `xChannelIORead` / `xChannelIOWrite` 按 64 字节帧交换过程数据。有效载荷 60 字节：E1–E3、A1–A6、TCP XYZABC，各 FLOAT32。

- 规范与 CIFX 改法：[`docs/kuka_profinet_io.md`](docs/kuka_profinet_io.md)
- 地址表：[`docs/kuka_profinet_io_map.csv`](docs/kuka_profinet_io_map.csv)
- KRL：[`kuka/profinet/kuka_pn_axis_pose.src`](kuka/profinet/kuka_pn_axis_pose.src)

```bash
make test
make docs
```

