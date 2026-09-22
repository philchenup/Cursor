# 上位机 ↔ KUKA 焊接通讯数据表

只保留总流程必要项。上位机下发参考起终点、焊枪姿态、焊速、摆动，并编排寻缝；激光经 RSI 找点后 TCP 到位，再焊到找到的终点。焊中纠偏不走本表。

完整表：[`kuka_weld_comm_table.md`](kuka_weld_comm_table.md)

## 打包字节数

INT32 / FLOAT32 / BOOL 均按 4 字节对齐（对应 EKI 的 INT/REAL；BOOL 以 INT 传输）。

| 方向 | 信号数 | 打包字节数 |
| --- | --- | --- |
| 上位机→KUKA（下发） | 23 | 92 |
| KUKA→上位机（读取） | 16 | **64** |

上位机配置通讯 **读取数据字节大小** 时填 **64**。C++ 结构 `sizeof(KukaCyclic)` 同为 64。

EKI 线上仍是 XML 文本，长度随数值变化；socket 接收缓冲建议 ≥ 4096。现有运动通道 `kukasend` 回传 13 个 REAL（E2 + J1–J6 + XYZABC）为 52 字节，与本焊接流程表不是同一帧。

用 CIFX PROFINET 读 3 个外部轴 + 6 关节 + 6 TCP 位姿时，过程数据同样放入 64 字节 IO 帧，布局见 [`kuka_profinet_io.md`](kuka_profinet_io.md)（前 60 字节为 15×FLOAT32）。

## 流程

1. 下发参考起点/终点、`Torch_A/B/C`、焊速、摆动，`Laser_Mode=1`（或 `2` 含焊中跟踪）
2. `Host_Cmd=13` 寻起点 → `Laser_StartValid` 后 `Found_Start*`
3. `Host_Cmd=14` TCP 按焊枪姿态移到找到的起点
4. `Host_Cmd=15` 寻终点 → `Laser_EndValid` 后 `Found_End*`
5. `Host_Cmd=16` 焊到找到的终点（`Weld_ArcEnable=1`）；`Laser_Mode=2` 时焊中 RSI 跟踪
6. `Host_Cmd=6` 为 13→14→15→16 自动串行
