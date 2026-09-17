# 上位机 ↔ KUKA 焊接通讯数据表

只保留总流程必要项。上位机下发参考起终点、焊枪姿态、焊速、摆动，并编排寻缝；激光经 RSI 找点后 TCP 到位，再焊到找到的终点。焊中纠偏不走本表。

完整表：[`kuka_weld_comm_table.md`](kuka_weld_comm_table.md)

## 流程

1. 下发参考起点/终点、`Torch_A/B/C`、焊速、摆动，`Laser_Mode=1`（或 `2` 含焊中跟踪）
2. `Host_Cmd=13` 寻起点 → `Laser_StartValid` 后 `Found_Start*`
3. `Host_Cmd=14` TCP 按焊枪姿态移到找到的起点
4. `Host_Cmd=15` 寻终点 → `Laser_EndValid` 后 `Found_End*`
5. `Host_Cmd=16` 焊到找到的终点（`Weld_ArcEnable=1`）；`Laser_Mode=2` 时焊中 RSI 跟踪
6. `Host_Cmd=6` 为 13→14→15→16 自动串行
