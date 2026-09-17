# 上位机 ↔ KUKA 焊接通讯数据表

上位机做**整段流程控制**；KUKA 做 LIN 插补；激光寻缝器经 **RSI** 把找到的焊缝点交给控制器。纠偏闭环不走 Ethernet KRL。

表为三列：数据类型、信号名、含义。共 150 项。

- 完整表：[`kuka_weld_comm_table.md`](kuka_weld_comm_table.md) / [`kuka_weld_comm_table.csv`](kuka_weld_comm_table.csv)
- EKI：[`../kuka/EthernetKRL/WeldHost.xml`](../kuka/EthernetKRL/WeldHost.xml)

## 流程（上位机编排）

1. 下发参考起点/终点、焊枪姿态 `Torch_A/B/C`、焊速、摆动（正弦/三角、幅度、弦长），`Laser_Mode=1` 或 `2`。
2. `Host_Cmd=13 FindStart`：焊枪停在超前位，激光在枪前寻缝。
3. `Laser_StartValid=1` 后，`Found_StartX/Y/Z` 有效。
4. `Host_Cmd=14 MoveToFoundStart`：TCP 按 `Torch_*` 直线移到找到的起点。
5. `Host_Cmd=15 FindEnd`：激光寻终点（或焊中跟踪得到终点）。
6. `Laser_EndValid=1` 后，`Host_Cmd=16 WeldToFoundEnd`：从当前点插补到找到的终点；`Weld_ArcEnable=1` 则焊接。`Laser_Mode=2` 时焊中 RSI 纠偏。
7. `Host_Cmd=6 Start` 等价于上述 13→14→15→16 自动串起来。

寻缝失败：`Laser_Lost=1` 或超时，`Kuka_Phase=Fault`。

## 通道分工

| 通道 | 内容 |
| --- | --- |
| 上位机 ↔ KUKA（本表 / EKI） | 命令、参考起终点、姿态、焊速、摆动、寻缝使能、找到的点、状态 |
| 激光 ↔ KUKA（RSI） | 焊中 ΔY/ΔZ 实时纠偏；本表 `Laser_dY/dZ` 仅监视 |
