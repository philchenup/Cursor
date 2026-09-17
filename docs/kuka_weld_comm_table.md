# 上位机 ↔ KUKA 焊接通讯数据结构表

共 39 个信号，仅含总流程必要项。

## 上位机 → KUKA

| 数据类型 | 信号名 | 含义 |
| --- | --- | --- |
| INT32 | Host_Heartbeat | 上位机心跳 |
| INT32 | Host_Cmd | 0空闲 1复位 6自动全流程 7暂停 8继续 9停止 11回Home 12故障确认 13寻起点 14TCP到找到的起点 15寻终点 16焊到找到的终点 |
| INT32 | Host_CmdSeq | 命令序号，变化时执行一次 |
| INT32 | Seam_Id | 焊缝号 |
| FLOAT32 | Seam_StartX | 参考起点 X mm（寻缝搜索中心） |
| FLOAT32 | Seam_StartY | 参考起点 Y mm |
| FLOAT32 | Seam_StartZ | 参考起点 Z mm |
| FLOAT32 | Seam_EndX | 参考终点 X mm（寻缝搜索中心） |
| FLOAT32 | Seam_EndY | 参考终点 Y mm |
| FLOAT32 | Seam_EndZ | 参考终点 Z mm |
| FLOAT32 | Torch_A | 焊枪姿态 A deg |
| FLOAT32 | Torch_B | 焊枪姿态 B deg |
| FLOAT32 | Torch_C | 焊枪姿态 C deg |
| FLOAT32 | Seam_WeldSpeed | 焊接速度 mm/s |
| INT32 | Seam_WeaveMode | 0 直线焊  1 摆动焊 |
| INT32 | Seam_WeaveType | 0 正弦  1 三角 |
| FLOAT32 | Seam_Amplitude | 摆动幅度 mm |
| FLOAT32 | Seam_Chord | 摆动弦长 mm |
| INT32 | Laser_Mode | 0 不用激光  1 寻缝  2 寻缝+焊中RSI跟踪 |
| FLOAT32 | Laser_LookAhead | 激光超前距 mm（焊枪前方） |
| FLOAT32 | Laser_SearchRadius | 相对参考点的寻缝范围 mm |
| INT32 | Laser_Timeout | 寻缝超时 ms |
| BOOL | Weld_ArcEnable | 1 焊接到终点  0 只移动到终点 |

## KUKA → 上位机

| 数据类型 | 信号名 | 含义 |
| --- | --- | --- |
| INT32 | Kuka_Heartbeat | 机器人心跳 |
| INT32 | Kuka_CmdAck | 已接受的命令序号 |
| INT32 | Kuka_Phase | 寻缝/找到起点/TCP到位/寻终点/焊接/故障 |
| BOOL | Kuka_EStop | 急停 |
| INT32 | Kuka_MsgId | 故障码，0 为正常 |
| BOOL | Laser_Ready | 激光寻缝器就绪 |
| BOOL | Laser_StartValid | 已找到起点，Found_Start* 有效 |
| BOOL | Laser_EndValid | 已找到终点，Found_End* 有效 |
| BOOL | Laser_Lost | 寻缝或跟踪丢失 |
| FLOAT32 | Found_StartX | 激光找到的起点 X mm |
| FLOAT32 | Found_StartY | 激光找到的起点 Y mm |
| FLOAT32 | Found_StartZ | 激光找到的起点 Z mm |
| FLOAT32 | Found_EndX | 激光找到的终点 X mm |
| FLOAT32 | Found_EndY | 激光找到的终点 Y mm |
| FLOAT32 | Found_EndZ | 激光找到的终点 Z mm |
| BOOL | Kuka_JobDone | 本焊缝流程完成 |

## 上位机→KUKA 报文示例

```xml
<Host><Host_Heartbeat>1</Host_Heartbeat><Host_Cmd>13</Host_Cmd><Host_CmdSeq>1</Host_CmdSeq><Seam_Id>1</Seam_Id><Seam_StartX>0</Seam_StartX><Seam_StartY>0</Seam_StartY><Seam_StartZ>0</Seam_StartZ><Seam_EndX>100</Seam_EndX><Seam_EndY>0</Seam_EndY><Seam_EndZ>0</Seam_EndZ><Torch_A>0</Torch_A><Torch_B>90</Torch_B><Torch_C>180</Torch_C><Seam_WeldSpeed>10</Seam_WeldSpeed><Seam_WeaveMode>1</Seam_WeaveMode><Seam_WeaveType>0</Seam_WeaveType><Seam_Amplitude>5</Seam_Amplitude><Seam_Chord>20</Seam_Chord><Laser_Mode>1</Laser_Mode><Laser_LookAhead>30</Laser_LookAhead><Laser_SearchRadius>20</Laser_SearchRadius><Laser_Timeout>5000</Laser_Timeout><Weld_ArcEnable>1</Weld_ArcEnable></Host>
```

## KUKA→上位机 报文示例

```xml
<Kuka><Kuka_Heartbeat>1</Kuka_Heartbeat><Kuka_CmdAck>0</Kuka_CmdAck><Kuka_Phase>4</Kuka_Phase><Kuka_EStop>0</Kuka_EStop><Kuka_MsgId>0</Kuka_MsgId><Laser_Ready>1</Laser_Ready><Laser_StartValid>0</Laser_StartValid><Laser_EndValid>0</Laser_EndValid><Laser_Lost>0</Laser_Lost><Found_StartX>0</Found_StartX><Found_StartY>0</Found_StartY><Found_StartZ>0</Found_StartZ><Found_EndX>0</Found_EndX><Found_EndY>0</Found_EndY><Found_EndZ>0</Found_EndZ><Kuka_JobDone>0</Kuka_JobDone></Kuka>
```
