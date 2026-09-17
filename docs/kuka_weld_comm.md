# 上位机 ↔ KUKA 焊接通讯数据表

表格式与 PLC/示教器映射一致：两列 **数据类型**、**信号名**。运动区前 19 项与现场表相同（`Extern_Speed` … `Robot_J6`），其后为焊接工艺字。

- Markdown：[`kuka_weld_comm_table.md`](kuka_weld_comm_table.md)
- CSV：[`kuka_weld_comm_table.csv`](kuka_weld_comm_table.csv)
- EKI：[`../kuka/EthernetKRL/WeldHost.xml`](../kuka/EthernetKRL/WeldHost.xml)

数据类型：`FLOAT32` / `INT32` / `BOOL`。单位：mm、deg、mm/s、A、V、m/min、ms。

## 上位机 → KUKA

| 数据类型 | 信号名 | 工艺含义 |
| --- | --- | --- |
| FLOAT32 | Extern_Speed | 外部轴（地轨）速度 |
| FLOAT32 | Extern_Acc | 外部轴加速度 |
| FLOAT32 | Robot_Speed | 机器人速度（焊接时即焊速指令） |
| FLOAT32 | Robot_Acc | 机器人加速度 |
| FLOAT32 | Extern_E1 | 外部轴 E1 位置 |
| FLOAT32 | Extern_E2 | 外部轴 E2 |
| FLOAT32 | Extern_E3 | 外部轴 E3 |
| FLOAT32 | Robot_X | TCP X |
| FLOAT32 | Robot_Y | TCP Y |
| FLOAT32 | Robot_Z | TCP Z |
| FLOAT32 | Robot_A | 姿态 A |
| FLOAT32 | Robot_B | 姿态 B |
| FLOAT32 | Robot_C | 姿态 C |
| FLOAT32 | Robot_J1 | 关节 1 |
| FLOAT32 | Robot_J2 | 关节 2 |
| FLOAT32 | Robot_J3 | 关节 3 |
| FLOAT32 | Robot_J4 | 关节 4 |
| FLOAT32 | Robot_J5 | 关节 5 |
| FLOAT32 | Robot_J6 | 关节 6 |
| INT32 | Host_Heartbeat | 上位机心跳 |
| INT32 | Host_Cmd | 命令字 |
| INT32 | Host_CmdSeq | 命令序号（边沿触发） |
| INT32 | Job_Id | 作业号 |
| INT32 | Job_SeamCount | 焊缝条数 |
| INT32 | Job_PassCount | 焊道总数 |
| INT32 | Job_ToolNo | $TOOL |
| INT32 | Job_BaseNo | $BASE |
| FLOAT32 | Job_Override | 速度倍率 % |
| FLOAT32 | Job_Approach | 接近高度 |
| FLOAT32 | Job_Retract | 回撤高度 |
| INT32 | Seam_Id | 焊缝序号 |
| FLOAT32 | Seam_StartX … Seam_StartC | 工艺表起点（内缩前） |
| FLOAT32 | Seam_EndX … Seam_EndC | 工艺表终点 |
| FLOAT32 | Seam_Inset | 内缩 |
| FLOAT32 | Seam_WeldSpeed | 焊接速度 |
| INT32 | Seam_WeaveMode | 0 直线焊 / 1 摆动焊 |
| INT32 | Seam_WeaveType | 0 正弦 / 1 三角 |
| FLOAT32 | Seam_Amplitude | 摆动幅度 |
| FLOAT32 | Seam_Chord | 摆动弦长 |
| INT32 | Seam_MultiMode | 0 单层单道 / 1 多层多道 |
| FLOAT32 | Seam_Thickness | 板厚 |
| FLOAT32 | Seam_Groove | 坡口角度 |
| FLOAT32 | Seam_FitUpGap | 装配间隙 |
| FLOAT32 | Seam_Penetration | 熔深 |
| INT32 | Pass_SeamId / Layer / Local / Seq / Kind | 焊道：所属焊缝、层、道、顺序、0打底1填充2盖面 |
| FLOAT32 | Pass_StartX … Pass_EndC | 本焊道起终点 |
| FLOAT32 | Pass_Speed | 本焊道速度 |
| INT32 | Traj_Index / Traj_Count / Traj_Flag | 轨迹点序号、总数、起弧/收弧标志 |
| FLOAT32 | Traj_Speed | 该点进给 |
| BOOL | Weld_ArcEnable / Weld_GasEnable | 允许起弧 / 送气 |
| FLOAT32 | Weld_Current / Weld_Voltage / Weld_WireSpeed | 电流、电压、送丝 |
| INT32 | Weld_GasPreflow / Weld_GasPostflow / Weld_Crater | 预吹、滞后、填弧坑 ms |

完整逐行表见 Markdown/CSV（含 `Seam_StartX` 等到每一个 FLOAT32）。

## KUKA → 上位机

实际运动字与指令区同布局，名称加 `Act_` 前缀，避免与指令区重名。

| 数据类型 | 信号名 | 工艺含义 |
| --- | --- | --- |
| INT32 | Kuka_Heartbeat | 机器人心跳 |
| INT32 | Kuka_CmdAck | 已接受命令序号 |
| INT32 | Kuka_Phase | 工艺阶段（地轨/接近/焊接/收弧/回Home/故障） |
| INT32 | Kuka_OpMode | 0T1 1T2 2AUT 3EXT |
| BOOL | Kuka_ProActive / DrivesOn / EStop | 程序运行、使能、急停 |
| INT32 | Kuka_MsgId | 报警号 |
| INT32 | Kuka_SeamId / Layer / PassSeq / TrajIndex | 当前焊缝/层/道/轨迹点 |
| FLOAT32 | Act_Extern_Speed … Act_Robot_J6 | 实际速度、外部轴、TCP、关节（19 字，同指令区） |
| BOOL | Kuka_ArcOn / Collision / DownloadOk / JobDone | 起弧反馈、碰撞、下载完成、作业完成 |
| FLOAT32 | Kuka_Progress | 进度 % |

## 流程

下载作业/焊缝/焊道 → `Host_Cmd=Start` → 写 `Extern_E1` 地轨对齐 → `Robot_X…C` 接近与焊接 → `Weld_ArcEnable` 起弧 → 沿轨迹更新 `Robot_*` → 收弧回撤 → `Kuka_JobDone`。

`Host_Cmd`：0 Idle，1 Reset，2–5 Download*，6 Start，7 Pause，8 Resume，9 Stop，10 ArcOff，11 GoHome，12 AckFault。
