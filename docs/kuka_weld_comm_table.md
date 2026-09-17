# 上位机 ↔ KUKA 焊接通讯数据结构表

共 150 个信号。命令 Start=6，阶段 Welding=10。

## 上位机 → KUKA

| 数据类型 | 信号名 | 含义 |
| --- | --- | --- |
| FLOAT32 | Extern_Speed | 外部轴速度指令 mm/s |
| FLOAT32 | Extern_Acc | 外部轴加速度指令 |
| FLOAT32 | Robot_Speed | 机器人速度指令；焊接时为焊速 mm/s |
| FLOAT32 | Robot_Acc | 机器人加速度指令 |
| FLOAT32 | Extern_E1 | 外部轴 E1 位置指令（地轨）mm |
| FLOAT32 | Extern_E2 | 外部轴 E2 位置指令 mm |
| FLOAT32 | Extern_E3 | 外部轴 E3 位置指令 mm |
| FLOAT32 | Robot_X | TCP X 位置指令 mm |
| FLOAT32 | Robot_Y | TCP Y 位置指令 mm |
| FLOAT32 | Robot_Z | TCP Z 位置指令 mm |
| FLOAT32 | Robot_A | TCP 姿态 A 指令 deg（绕 Z） |
| FLOAT32 | Robot_B | TCP 姿态 B 指令 deg（绕 Y） |
| FLOAT32 | Robot_C | TCP 姿态 C 指令 deg（绕 X） |
| FLOAT32 | Robot_J1 | 关节 1 角度指令 deg |
| FLOAT32 | Robot_J2 | 关节 2 角度指令 deg |
| FLOAT32 | Robot_J3 | 关节 3 角度指令 deg |
| FLOAT32 | Robot_J4 | 关节 4 角度指令 deg |
| FLOAT32 | Robot_J5 | 关节 5 角度指令 deg |
| FLOAT32 | Robot_J6 | 关节 6 角度指令 deg |
| INT32 | Host_Heartbeat | 上位机心跳，周期递增 |
| INT32 | Host_Cmd | 命令字：0空闲 1复位 2-5下载 6自动全流程 7暂停 8继续 9停止 10收弧 11回Home 12故障确认 13寻起点 14TCP到找到的起点 15寻终点 16焊/移动到找到的终点 |
| INT32 | Host_CmdSeq | 命令序号，KUKA 仅在变化时执行一次 |
| INT32 | Job_Id | 焊接作业号 |
| INT32 | Job_SeamCount | 本作业焊缝条数 |
| INT32 | Job_PassCount | 本作业焊道总数 |
| INT32 | Job_ToolNo | 焊枪工具坐标系 $TOOL 编号 |
| INT32 | Job_BaseNo | 工件坐标系 $BASE 编号 |
| FLOAT32 | Job_Override | 建议速度倍率 % |
| FLOAT32 | Job_Approach | 接近高度 mm（沿 TCP -Z） |
| FLOAT32 | Job_Retract | 收弧后回撤高度 mm |
| INT32 | Seam_Id | 焊缝序号，对应工艺表序号 |
| FLOAT32 | Seam_StartX | 焊缝参考起点 X mm（内缩前，寻缝搜索中心） |
| FLOAT32 | Seam_StartY | 焊缝参考起点 Y mm |
| FLOAT32 | Seam_StartZ | 焊缝参考起点 Z mm |
| FLOAT32 | Seam_StartA | 焊缝参考起点姿态 A deg |
| FLOAT32 | Seam_StartB | 焊缝参考起点姿态 B deg |
| FLOAT32 | Seam_StartC | 焊缝参考起点姿态 C deg |
| FLOAT32 | Seam_EndX | 焊缝参考终点 X mm（内缩前，寻缝搜索中心） |
| FLOAT32 | Seam_EndY | 焊缝参考终点 Y mm |
| FLOAT32 | Seam_EndZ | 焊缝参考终点 Z mm |
| FLOAT32 | Seam_EndA | 焊缝参考终点姿态 A deg |
| FLOAT32 | Seam_EndB | 焊缝参考终点姿态 B deg |
| FLOAT32 | Seam_EndC | 焊缝参考终点姿态 C deg |
| FLOAT32 | Seam_Inset | 内缩 mm，沿焊缝从起终点各收回 |
| FLOAT32 | Seam_WeldSpeed | 焊接速度 mm/s，对应工艺表焊接速度 |
| INT32 | Seam_WeaveMode | 摆动方式：0 直线焊  1 摆动焊 |
| INT32 | Seam_WeaveType | 摆动类型：0 正弦  1 三角 |
| FLOAT32 | Seam_Amplitude | 摆动幅度 mm（侧向峰值） |
| FLOAT32 | Seam_Chord | 摆动弦长 mm（一个周期沿焊缝长度） |
| INT32 | Seam_MultiMode | 0 单层单道  1 多层多道 |
| FLOAT32 | Seam_Thickness | 板厚 mm |
| FLOAT32 | Seam_Groove | 坡口角度 deg |
| FLOAT32 | Seam_FitUpGap | 装配间隙 mm |
| FLOAT32 | Seam_Penetration | 熔深 mm |
| INT32 | Pass_SeamId | 本焊道所属焊缝号 |
| INT32 | Pass_Layer | 层号，打底层为 1 |
| INT32 | Pass_Local | 层内道号 |
| INT32 | Pass_Seq | 全局焊接顺序 |
| INT32 | Pass_Kind | 焊道类型：0 打底  1 填充  2 盖面 |
| FLOAT32 | Pass_StartX | 本焊道起点 X mm（含层/道偏移） |
| FLOAT32 | Pass_StartY | 本焊道起点 Y mm |
| FLOAT32 | Pass_StartZ | 本焊道起点 Z mm |
| FLOAT32 | Pass_StartA | 本焊道起点姿态 A deg |
| FLOAT32 | Pass_StartB | 本焊道起点姿态 B deg |
| FLOAT32 | Pass_StartC | 本焊道起点姿态 C deg |
| FLOAT32 | Pass_EndX | 本焊道终点 X mm |
| FLOAT32 | Pass_EndY | 本焊道终点 Y mm |
| FLOAT32 | Pass_EndZ | 本焊道终点 Z mm |
| FLOAT32 | Pass_EndA | 本焊道终点姿态 A deg |
| FLOAT32 | Pass_EndB | 本焊道终点姿态 B deg |
| FLOAT32 | Pass_EndC | 本焊道终点姿态 C deg |
| FLOAT32 | Pass_Speed | 本焊道焊接速度 mm/s |
| INT32 | Traj_Index | 当前轨迹点序号 |
| INT32 | Traj_Count | 本焊道轨迹点总数 |
| FLOAT32 | Traj_Speed | 该轨迹点进给速度 mm/s |
| INT32 | Traj_Flag | bit0 起弧点  bit1 收弧点  bit2 本焊道末点 |
| BOOL | Weld_ArcEnable | 允许起弧 |
| BOOL | Weld_GasEnable | 允许送保护气 |
| FLOAT32 | Weld_Current | 焊接电流设定 A |
| FLOAT32 | Weld_Voltage | 电弧电压设定 V |
| FLOAT32 | Weld_WireSpeed | 送丝速度 m/min |
| INT32 | Weld_GasPreflow | 起弧前气体预吹时间 ms |
| INT32 | Weld_GasPostflow | 收弧后气体滞后时间 ms |
| INT32 | Weld_Crater | 填弧坑时间 ms |
| FLOAT32 | Torch_A | 焊枪姿态 A deg；寻到焊缝点后按此姿态 TCP 到位 |
| FLOAT32 | Torch_B | 焊枪姿态 B deg |
| FLOAT32 | Torch_C | 焊枪姿态 C deg |
| INT32 | Laser_Mode | 0 不用激光按参考点焊  1 寻缝后到位  2 寻缝+焊中RSI跟踪 |
| BOOL | Laser_FindEnable | 允许激光寻缝（起点/终点） |
| BOOL | Laser_TrackEnable | 允许焊中 RSI 纠偏（纠偏闭环不走本表） |
| FLOAT32 | Laser_LookAhead | 激光超前距 mm（焊枪前方寻缝） |
| FLOAT32 | Laser_SearchRadius | 相对参考点的寻缝范围 mm |
| FLOAT32 | Laser_SearchSpeed | 寻缝移动速度 mm/s |
| INT32 | Laser_Timeout | 单次寻缝超时 ms |

## KUKA → 上位机

| 数据类型 | 信号名 | 含义 |
| --- | --- | --- |
| INT32 | Kuka_Heartbeat | 机器人心跳 |
| INT32 | Kuka_CmdAck | 已接受的命令序号 |
| INT32 | Kuka_Phase | 工艺阶段：地轨/接近/寻缝/找到起点/TCP到位/寻终点/焊接到终点/收弧/回Home/故障 |
| INT32 | Kuka_OpMode | 运行模式：0 T1  1 T2  2 AUT  3 EXT |
| BOOL | Kuka_ProActive | 解释器程序正在运行 |
| BOOL | Kuka_DrivesOn | 驱动已使能 |
| BOOL | Kuka_EStop | 急停 |
| INT32 | Kuka_MsgId | 报警号，0 表示无报警 |
| INT32 | Kuka_SeamId | 当前焊缝号 |
| INT32 | Kuka_Layer | 当前层号 |
| INT32 | Kuka_PassSeq | 当前焊道全局序号 |
| INT32 | Kuka_TrajIndex | 当前轨迹点序号 |
| FLOAT32 | Act_Extern_Speed | 外部轴实际速度 mm/s |
| FLOAT32 | Act_Extern_Acc | 外部轴实际加速度 |
| FLOAT32 | Act_Robot_Speed | 机器人实际速度 mm/s |
| FLOAT32 | Act_Robot_Acc | 机器人实际加速度 |
| FLOAT32 | Act_Extern_E1 | 外部轴 E1 实际位置 mm |
| FLOAT32 | Act_Extern_E2 | 外部轴 E2 实际位置 mm |
| FLOAT32 | Act_Extern_E3 | 外部轴 E3 实际位置 mm |
| FLOAT32 | Act_Robot_X | TCP 实际 X mm |
| FLOAT32 | Act_Robot_Y | TCP 实际 Y mm |
| FLOAT32 | Act_Robot_Z | TCP 实际 Z mm |
| FLOAT32 | Act_Robot_A | TCP 实际姿态 A deg |
| FLOAT32 | Act_Robot_B | TCP 实际姿态 B deg |
| FLOAT32 | Act_Robot_C | TCP 实际姿态 C deg |
| FLOAT32 | Act_Robot_J1 | 关节 1 实际角度 deg |
| FLOAT32 | Act_Robot_J2 | 关节 2 实际角度 deg |
| FLOAT32 | Act_Robot_J3 | 关节 3 实际角度 deg |
| FLOAT32 | Act_Robot_J4 | 关节 4 实际角度 deg |
| FLOAT32 | Act_Robot_J5 | 关节 5 实际角度 deg |
| FLOAT32 | Act_Robot_J6 | 关节 6 实际角度 deg |
| BOOL | Kuka_ArcOn | 焊机起弧成功反馈 |
| BOOL | Kuka_Collision | 碰撞或力矩超限 |
| BOOL | Kuka_DownloadOk | 作业/焊缝/焊道/轨迹下载完成 |
| BOOL | Kuka_JobDone | 全部焊缝焊完并已回 Home |
| FLOAT32 | Kuka_Progress | 当前作业进度 % |
| BOOL | Laser_Ready | 激光寻缝器就绪 |
| BOOL | Laser_Finding | 正在寻缝 |
| BOOL | Laser_StartValid | 已找到焊缝起点，Found_Start* 有效 |
| BOOL | Laser_EndValid | 已找到焊缝终点，Found_End* 有效 |
| BOOL | Laser_Lost | 寻缝丢失或跟踪丢失 |
| INT32 | Laser_ErrId | 激光故障码，0 表示正常 |
| FLOAT32 | Found_StartX | 激光找到的焊缝起点 X mm |
| FLOAT32 | Found_StartY | 激光找到的焊缝起点 Y mm |
| FLOAT32 | Found_StartZ | 激光找到的焊缝起点 Z mm |
| FLOAT32 | Found_StartA | 到位用焊枪姿态 A（通常回传 Torch_A） |
| FLOAT32 | Found_StartB | 到位用焊枪姿态 B |
| FLOAT32 | Found_StartC | 到位用焊枪姿态 C |
| FLOAT32 | Found_EndX | 激光找到的焊缝终点 X mm |
| FLOAT32 | Found_EndY | 激光找到的焊缝终点 Y mm |
| FLOAT32 | Found_EndZ | 激光找到的焊缝终点 Z mm |
| FLOAT32 | Found_EndA | 终点焊枪姿态 A |
| FLOAT32 | Found_EndB | 终点焊枪姿态 B |
| FLOAT32 | Found_EndC | 终点焊枪姿态 C |
| FLOAT32 | Laser_dY | RSI 横向纠偏监视值 mm（闭环不走本表） |
| FLOAT32 | Laser_dZ | RSI 高度纠偏监视值 mm（闭环不走本表） |

## 上位机→KUKA 报文示例

```xml
<Host><Extern_Speed>0</Extern_Speed><Extern_Acc>0</Extern_Acc><Robot_Speed>10</Robot_Speed><Robot_Acc>0</Robot_Acc><Extern_E1>0</Extern_E1><Extern_E2>0</Extern_E2><Extern_E3>0</Extern_E3><Robot_X>0</Robot_X><Robot_Y>0</Robot_Y><Robot_Z>0</Robot_Z><Robot_A>0</Robot_A><Robot_B>90</Robot_B><Robot_C>180</Robot_C><Robot_J1>0</Robot_J1><Robot_J2>0</Robot_J2><Robot_J3>0</Robot_J3><Robot_J4>0</Robot_J4><Robot_J5>0</Robot_J5><Robot_J6>0</Robot_J6><Host_Heartbeat>1</Host_Heartbeat><Host_Cmd>3</Host_Cmd><Host_CmdSeq>1</Host_CmdSeq><Job_Id>1</Job_Id><Job_SeamCount>1</Job_SeamCount><Job_PassCount>1</Job_PassCount><Job_ToolNo>1</Job_ToolNo><Job_BaseNo>0</Job_BaseNo><Job_Override>100</Job_Override><Job_Approach>50</Job_Approach><Job_Retract>50</Job_Retract><Seam_Id>1</Seam_Id><Seam_StartX>0</Seam_StartX><Seam_StartY>0</Seam_StartY><Seam_StartZ>0</Seam_StartZ><Seam_StartA>0</Seam_StartA><Seam_StartB>90</Seam_StartB><Seam_StartC>180</Seam_StartC><Seam_EndX>100</Seam_EndX><Seam_EndY>0</Seam_EndY><Seam_EndZ>0</Seam_EndZ><Seam_EndA>0</Seam_EndA><Seam_EndB>90</Seam_EndB><Seam_EndC>180</Seam_EndC><Torch_A>0</Torch_A><Torch_B>90</Torch_B><Torch_C>180</Torch_C><Seam_Inset>0</Seam_Inset><Seam_WeldSpeed>10</Seam_WeldSpeed><Seam_WeaveMode>1</Seam_WeaveMode><Seam_WeaveType>0</Seam_WeaveType><Seam_Amplitude>5</Seam_Amplitude><Seam_Chord>20</Seam_Chord><Seam_MultiMode>0</Seam_MultiMode><Seam_Thickness>0</Seam_Thickness><Seam_Groove>0</Seam_Groove><Seam_FitUpGap>0</Seam_FitUpGap><Seam_Penetration>0</Seam_Penetration><Pass_SeamId>1</Pass_SeamId><Pass_Layer>1</Pass_Layer><Pass_Local>1</Pass_Local><Pass_Seq>1</Pass_Seq><Pass_Kind>0</Pass_Kind><Pass_StartX>0</Pass_StartX><Pass_StartY>0</Pass_StartY><Pass_StartZ>0</Pass_StartZ><Pass_StartA>0</Pass_StartA><Pass_StartB>0</Pass_StartB><Pass_StartC>0</Pass_StartC><Pass_EndX>0</Pass_EndX><Pass_EndY>0</Pass_EndY><Pass_EndZ>0</Pass_EndZ><Pass_EndA>0</Pass_EndA><Pass_EndB>0</Pass_EndB><Pass_EndC>0</Pass_EndC><Pass_Speed>10</Pass_Speed><Traj_Index>0</Traj_Index><Traj_Count>2</Traj_Count><Traj_Speed>10</Traj_Speed><Traj_Flag>1</Traj_Flag><Weld_ArcEnable>1</Weld_ArcEnable><Weld_GasEnable>1</Weld_GasEnable><Weld_Current>0</Weld_Current><Weld_Voltage>0</Weld_Voltage><Weld_WireSpeed>0</Weld_WireSpeed><Weld_GasPreflow>200</Weld_GasPreflow><Weld_GasPostflow>400</Weld_GasPostflow><Weld_Crater>200</Weld_Crater><Laser_Mode>1</Laser_Mode><Laser_FindEnable>0</Laser_FindEnable><Laser_TrackEnable>0</Laser_TrackEnable><Laser_LookAhead>30</Laser_LookAhead><Laser_SearchRadius>20</Laser_SearchRadius><Laser_SearchSpeed>20</Laser_SearchSpeed><Laser_Timeout>5000</Laser_Timeout></Host>
```

## KUKA→上位机 报文示例

```xml
<Kuka><Kuka_Heartbeat>1</Kuka_Heartbeat><Kuka_CmdAck>0</Kuka_CmdAck><Kuka_Phase>4</Kuka_Phase><Kuka_OpMode>3</Kuka_OpMode><Kuka_ProActive>1</Kuka_ProActive><Kuka_DrivesOn>1</Kuka_DrivesOn><Kuka_EStop>0</Kuka_EStop><Kuka_MsgId>0</Kuka_MsgId><Kuka_SeamId>0</Kuka_SeamId><Kuka_Layer>0</Kuka_Layer><Kuka_PassSeq>0</Kuka_PassSeq><Kuka_TrajIndex>0</Kuka_TrajIndex><Act_Extern_Speed>0</Act_Extern_Speed><Act_Extern_Acc>0</Act_Extern_Acc><Act_Robot_Speed>0</Act_Robot_Speed><Act_Robot_Acc>0</Act_Robot_Acc><Act_Extern_E1>0</Act_Extern_E1><Act_Extern_E2>0</Act_Extern_E2><Act_Extern_E3>0</Act_Extern_E3><Act_Robot_X>0</Act_Robot_X><Act_Robot_Y>0</Act_Robot_Y><Act_Robot_Z>0</Act_Robot_Z><Act_Robot_A>0</Act_Robot_A><Act_Robot_B>0</Act_Robot_B><Act_Robot_C>0</Act_Robot_C><Act_Robot_J1>0</Act_Robot_J1><Act_Robot_J2>0</Act_Robot_J2><Act_Robot_J3>0</Act_Robot_J3><Act_Robot_J4>0</Act_Robot_J4><Act_Robot_J5>0</Act_Robot_J5><Act_Robot_J6>0</Act_Robot_J6><Kuka_ArcOn>0</Kuka_ArcOn><Kuka_Collision>0</Kuka_Collision><Kuka_DownloadOk>1</Kuka_DownloadOk><Kuka_JobDone>0</Kuka_JobDone><Kuka_Progress>0</Kuka_Progress><Laser_Ready>0</Laser_Ready><Laser_Finding>0</Laser_Finding><Laser_StartValid>0</Laser_StartValid><Laser_EndValid>0</Laser_EndValid><Laser_Lost>0</Laser_Lost><Laser_ErrId>0</Laser_ErrId><Found_StartX>0</Found_StartX><Found_StartY>0</Found_StartY><Found_StartZ>0</Found_StartZ><Found_StartA>0</Found_StartA><Found_StartB>0</Found_StartB><Found_StartC>0</Found_StartC><Found_EndX>0</Found_EndX><Found_EndY>0</Found_EndY><Found_EndZ>0</Found_EndZ><Found_EndA>0</Found_EndA><Found_EndB>0</Found_EndB><Found_EndC>0</Found_EndC><Laser_dY>0</Laser_dY><Laser_dZ>0</Laser_dZ></Kuka>
```
