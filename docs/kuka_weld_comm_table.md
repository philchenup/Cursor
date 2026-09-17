# 上位机 ↔ KUKA 焊接通讯数据结构表

共 120 个信号。命令 Start=6，阶段 Welding=10。

## 上位机 → KUKA

| 数据类型 | 信号名 |
| --- | --- |
| FLOAT32 | Extern_Speed |
| FLOAT32 | Extern_Acc |
| FLOAT32 | Robot_Speed |
| FLOAT32 | Robot_Acc |
| FLOAT32 | Extern_E1 |
| FLOAT32 | Extern_E2 |
| FLOAT32 | Extern_E3 |
| FLOAT32 | Robot_X |
| FLOAT32 | Robot_Y |
| FLOAT32 | Robot_Z |
| FLOAT32 | Robot_A |
| FLOAT32 | Robot_B |
| FLOAT32 | Robot_C |
| FLOAT32 | Robot_J1 |
| FLOAT32 | Robot_J2 |
| FLOAT32 | Robot_J3 |
| FLOAT32 | Robot_J4 |
| FLOAT32 | Robot_J5 |
| FLOAT32 | Robot_J6 |
| INT32 | Host_Heartbeat |
| INT32 | Host_Cmd |
| INT32 | Host_CmdSeq |
| INT32 | Job_Id |
| INT32 | Job_SeamCount |
| INT32 | Job_PassCount |
| INT32 | Job_ToolNo |
| INT32 | Job_BaseNo |
| FLOAT32 | Job_Override |
| FLOAT32 | Job_Approach |
| FLOAT32 | Job_Retract |
| INT32 | Seam_Id |
| FLOAT32 | Seam_StartX |
| FLOAT32 | Seam_StartY |
| FLOAT32 | Seam_StartZ |
| FLOAT32 | Seam_StartA |
| FLOAT32 | Seam_StartB |
| FLOAT32 | Seam_StartC |
| FLOAT32 | Seam_EndX |
| FLOAT32 | Seam_EndY |
| FLOAT32 | Seam_EndZ |
| FLOAT32 | Seam_EndA |
| FLOAT32 | Seam_EndB |
| FLOAT32 | Seam_EndC |
| FLOAT32 | Seam_Inset |
| FLOAT32 | Seam_WeldSpeed |
| INT32 | Seam_WeaveMode |
| INT32 | Seam_WeaveType |
| FLOAT32 | Seam_Amplitude |
| FLOAT32 | Seam_Chord |
| INT32 | Seam_MultiMode |
| FLOAT32 | Seam_Thickness |
| FLOAT32 | Seam_Groove |
| FLOAT32 | Seam_FitUpGap |
| FLOAT32 | Seam_Penetration |
| INT32 | Pass_SeamId |
| INT32 | Pass_Layer |
| INT32 | Pass_Local |
| INT32 | Pass_Seq |
| INT32 | Pass_Kind |
| FLOAT32 | Pass_StartX |
| FLOAT32 | Pass_StartY |
| FLOAT32 | Pass_StartZ |
| FLOAT32 | Pass_StartA |
| FLOAT32 | Pass_StartB |
| FLOAT32 | Pass_StartC |
| FLOAT32 | Pass_EndX |
| FLOAT32 | Pass_EndY |
| FLOAT32 | Pass_EndZ |
| FLOAT32 | Pass_EndA |
| FLOAT32 | Pass_EndB |
| FLOAT32 | Pass_EndC |
| FLOAT32 | Pass_Speed |
| INT32 | Traj_Index |
| INT32 | Traj_Count |
| FLOAT32 | Traj_Speed |
| INT32 | Traj_Flag |
| BOOL | Weld_ArcEnable |
| BOOL | Weld_GasEnable |
| FLOAT32 | Weld_Current |
| FLOAT32 | Weld_Voltage |
| FLOAT32 | Weld_WireSpeed |
| INT32 | Weld_GasPreflow |
| INT32 | Weld_GasPostflow |
| INT32 | Weld_Crater |

## KUKA → 上位机

| 数据类型 | 信号名 |
| --- | --- |
| INT32 | Kuka_Heartbeat |
| INT32 | Kuka_CmdAck |
| INT32 | Kuka_Phase |
| INT32 | Kuka_OpMode |
| BOOL | Kuka_ProActive |
| BOOL | Kuka_DrivesOn |
| BOOL | Kuka_EStop |
| INT32 | Kuka_MsgId |
| INT32 | Kuka_SeamId |
| INT32 | Kuka_Layer |
| INT32 | Kuka_PassSeq |
| INT32 | Kuka_TrajIndex |
| FLOAT32 | Act_Extern_Speed |
| FLOAT32 | Act_Extern_Acc |
| FLOAT32 | Act_Robot_Speed |
| FLOAT32 | Act_Robot_Acc |
| FLOAT32 | Act_Extern_E1 |
| FLOAT32 | Act_Extern_E2 |
| FLOAT32 | Act_Extern_E3 |
| FLOAT32 | Act_Robot_X |
| FLOAT32 | Act_Robot_Y |
| FLOAT32 | Act_Robot_Z |
| FLOAT32 | Act_Robot_A |
| FLOAT32 | Act_Robot_B |
| FLOAT32 | Act_Robot_C |
| FLOAT32 | Act_Robot_J1 |
| FLOAT32 | Act_Robot_J2 |
| FLOAT32 | Act_Robot_J3 |
| FLOAT32 | Act_Robot_J4 |
| FLOAT32 | Act_Robot_J5 |
| FLOAT32 | Act_Robot_J6 |
| BOOL | Kuka_ArcOn |
| BOOL | Kuka_Collision |
| BOOL | Kuka_DownloadOk |
| BOOL | Kuka_JobDone |
| FLOAT32 | Kuka_Progress |

## 上位机→KUKA 报文示例

```xml
<Host><Extern_Speed>0</Extern_Speed><Extern_Acc>0</Extern_Acc><Robot_Speed>10</Robot_Speed><Robot_Acc>0</Robot_Acc><Extern_E1>0</Extern_E1><Extern_E2>0</Extern_E2><Extern_E3>0</Extern_E3><Robot_X>0</Robot_X><Robot_Y>0</Robot_Y><Robot_Z>0</Robot_Z><Robot_A>0</Robot_A><Robot_B>90</Robot_B><Robot_C>180</Robot_C><Robot_J1>0</Robot_J1><Robot_J2>0</Robot_J2><Robot_J3>0</Robot_J3><Robot_J4>0</Robot_J4><Robot_J5>0</Robot_J5><Robot_J6>0</Robot_J6><Host_Heartbeat>1</Host_Heartbeat><Host_Cmd>3</Host_Cmd><Host_CmdSeq>1</Host_CmdSeq><Job_Id>1</Job_Id><Job_SeamCount>1</Job_SeamCount><Job_PassCount>1</Job_PassCount><Job_ToolNo>1</Job_ToolNo><Job_BaseNo>0</Job_BaseNo><Job_Override>100</Job_Override><Job_Approach>50</Job_Approach><Job_Retract>50</Job_Retract><Seam_Id>1</Seam_Id><Seam_StartX>0</Seam_StartX><Seam_StartY>0</Seam_StartY><Seam_StartZ>0</Seam_StartZ><Seam_StartA>0</Seam_StartA><Seam_StartB>90</Seam_StartB><Seam_StartC>180</Seam_StartC><Seam_EndX>100</Seam_EndX><Seam_EndY>0</Seam_EndY><Seam_EndZ>0</Seam_EndZ><Seam_EndA>0</Seam_EndA><Seam_EndB>90</Seam_EndB><Seam_EndC>180</Seam_EndC><Seam_Inset>0</Seam_Inset><Seam_WeldSpeed>10</Seam_WeldSpeed><Seam_WeaveMode>1</Seam_WeaveMode><Seam_WeaveType>0</Seam_WeaveType><Seam_Amplitude>5</Seam_Amplitude><Seam_Chord>20</Seam_Chord><Seam_MultiMode>0</Seam_MultiMode><Seam_Thickness>0</Seam_Thickness><Seam_Groove>0</Seam_Groove><Seam_FitUpGap>0</Seam_FitUpGap><Seam_Penetration>0</Seam_Penetration><Pass_SeamId>1</Pass_SeamId><Pass_Layer>1</Pass_Layer><Pass_Local>1</Pass_Local><Pass_Seq>1</Pass_Seq><Pass_Kind>0</Pass_Kind><Pass_StartX>0</Pass_StartX><Pass_StartY>0</Pass_StartY><Pass_StartZ>0</Pass_StartZ><Pass_StartA>0</Pass_StartA><Pass_StartB>0</Pass_StartB><Pass_StartC>0</Pass_StartC><Pass_EndX>0</Pass_EndX><Pass_EndY>0</Pass_EndY><Pass_EndZ>0</Pass_EndZ><Pass_EndA>0</Pass_EndA><Pass_EndB>0</Pass_EndB><Pass_EndC>0</Pass_EndC><Pass_Speed>10</Pass_Speed><Traj_Index>0</Traj_Index><Traj_Count>2</Traj_Count><Traj_Speed>10</Traj_Speed><Traj_Flag>1</Traj_Flag><Weld_ArcEnable>1</Weld_ArcEnable><Weld_GasEnable>1</Weld_GasEnable><Weld_Current>0</Weld_Current><Weld_Voltage>0</Weld_Voltage><Weld_WireSpeed>0</Weld_WireSpeed><Weld_GasPreflow>200</Weld_GasPreflow><Weld_GasPostflow>400</Weld_GasPostflow><Weld_Crater>200</Weld_Crater></Host>
```

## KUKA→上位机 报文示例

```xml
<Kuka><Kuka_Heartbeat>1</Kuka_Heartbeat><Kuka_CmdAck>0</Kuka_CmdAck><Kuka_Phase>4</Kuka_Phase><Kuka_OpMode>3</Kuka_OpMode><Kuka_ProActive>1</Kuka_ProActive><Kuka_DrivesOn>1</Kuka_DrivesOn><Kuka_EStop>0</Kuka_EStop><Kuka_MsgId>0</Kuka_MsgId><Kuka_SeamId>0</Kuka_SeamId><Kuka_Layer>0</Kuka_Layer><Kuka_PassSeq>0</Kuka_PassSeq><Kuka_TrajIndex>0</Kuka_TrajIndex><Act_Extern_Speed>0</Act_Extern_Speed><Act_Extern_Acc>0</Act_Extern_Acc><Act_Robot_Speed>0</Act_Robot_Speed><Act_Robot_Acc>0</Act_Robot_Acc><Act_Extern_E1>0</Act_Extern_E1><Act_Extern_E2>0</Act_Extern_E2><Act_Extern_E3>0</Act_Extern_E3><Act_Robot_X>0</Act_Robot_X><Act_Robot_Y>0</Act_Robot_Y><Act_Robot_Z>0</Act_Robot_Z><Act_Robot_A>0</Act_Robot_A><Act_Robot_B>0</Act_Robot_B><Act_Robot_C>0</Act_Robot_C><Act_Robot_J1>0</Act_Robot_J1><Act_Robot_J2>0</Act_Robot_J2><Act_Robot_J3>0</Act_Robot_J3><Act_Robot_J4>0</Act_Robot_J4><Act_Robot_J5>0</Act_Robot_J5><Act_Robot_J6>0</Act_Robot_J6><Kuka_ArcOn>0</Kuka_ArcOn><Kuka_Collision>0</Kuka_Collision><Kuka_DownloadOk>1</Kuka_DownloadOk><Kuka_JobDone>0</Kuka_JobDone><Kuka_Progress>0</Kuka_Progress></Kuka>
```
