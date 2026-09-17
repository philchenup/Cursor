# 上位机 ↔ KUKA 焊接通讯数据结构表

共 101 个信号。命令 Start=6，阶段 Welding=10。

| 序号 | 分组 | 信号名 | XPath | 方向 | KRL类型 | 字节 | 单位 | 取值 | 工艺步骤 | 说明 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | 握手控制 | Heartbeat | `Host/Heartbeat` | 上位机→KUKA | INT | 4 | - | 0..2^31-1 | 全程 | 上位机心跳，周期递增，KUKA 用于判断通讯中断 |
| 2 | 握手控制 | Cmd | `Host/Cmd` | 上位机→KUKA | INT | 4 | - | 0..12 | 作业控制 | 命令字 Idle Reset Download Start Pause Resume Stop ArcOff GoHome AckFault |
| 3 | 握手控制 | CmdSeq | `Host/CmdSeq` | 上位机→KUKA | INT | 4 | - | 1..2^31-1 | 作业控制 | 命令序号，KUKA 仅在序号变化时执行一次 |
| 4 | 握手控制 | JobId | `Host/JobId` | 上位机→KUKA | INT | 4 | - | 1..9999 | 下载作业 | 焊接作业号 |
| 5 | 握手控制 | SeamCount | `Host/SeamCount` | 上位机→KUKA | INT | 4 | - | 1..256 | 下载作业 | 本作业焊缝条数 |
| 6 | 握手控制 | PassCount | `Host/PassCount` | 上位机→KUKA | INT | 4 | - | 1..1024 | 下载作业 | 本作业焊道总数（单层单道时等于焊缝数） |
| 7 | 握手控制 | ToolNo | `Host/ToolNo` | 上位机→KUKA | INT | 4 | - | 1..16 | 下载作业 | 焊枪 TCP 对应 $TOOL 编号 |
| 8 | 握手控制 | BaseNo | `Host/BaseNo` | 上位机→KUKA | INT | 4 | - | 0..32 | 下载作业 | 工件坐标系 $BASE 编号 |
| 9 | 握手控制 | OverridePct | `Host/OverridePct` | 上位机→KUKA | REAL | 4 | % | 1..100 | 运行 | 建议速度倍率；实际仍受示教器倍率限制 |
| 10 | 握手控制 | ApproachMm | `Host/ApproachMm` | 上位机→KUKA | REAL | 4 | mm | 0..500 | 接近 | 焊枪沿 TCP -Z 的接近高度 |
| 11 | 握手控制 | RetractMm | `Host/RetractMm` | 上位机→KUKA | REAL | 4 | mm | 0..500 | 回撤 | 收弧后沿 TCP -Z 的回撤高度 |
| 12 | 焊缝工艺 | SeamId | `Host/Seam/Id` | 上位机→KUKA | INT | 4 | - | 1..256 | 下载焊缝 | 焊缝序号，对应工艺表「序号」 |
| 13 | 焊缝工艺 | StartX | `Host/Seam/Start/X` | 上位机→KUKA | REAL | 4 | mm | ±10000 | 下载焊缝 | 原始起点 X（内缩前），对应「起点」 |
| 14 | 焊缝工艺 | StartY | `Host/Seam/Start/Y` | 上位机→KUKA | REAL | 4 | mm | ±10000 | 下载焊缝 | 原始起点 Y |
| 15 | 焊缝工艺 | StartZ | `Host/Seam/Start/Z` | 上位机→KUKA | REAL | 4 | mm | ±10000 | 下载焊缝 | 原始起点 Z |
| 16 | 焊缝工艺 | StartA | `Host/Seam/Start/A` | 上位机→KUKA | REAL | 4 | deg | ±180 | 下载焊缝 | 起点姿态 A（绕 Z） |
| 17 | 焊缝工艺 | StartB | `Host/Seam/Start/B` | 上位机→KUKA | REAL | 4 | deg | ±180 | 下载焊缝 | 起点姿态 B（绕 Y） |
| 18 | 焊缝工艺 | StartC | `Host/Seam/Start/C` | 上位机→KUKA | REAL | 4 | deg | ±180 | 下载焊缝 | 起点姿态 C（绕 X） |
| 19 | 焊缝工艺 | StartE1 | `Host/Seam/Start/E1` | 上位机→KUKA | REAL | 4 | mm | 地轨行程 | 下载焊缝 | 地轨：先对齐焊点 Y，再手臂到位 |
| 20 | 焊缝工艺 | EndX | `Host/Seam/End/X` | 上位机→KUKA | REAL | 4 | mm | ±10000 | 下载焊缝 | 原始终点 X，对应「终点」 |
| 21 | 焊缝工艺 | EndY | `Host/Seam/End/Y` | 上位机→KUKA | REAL | 4 | mm | ±10000 | 下载焊缝 | 原始终点 Y |
| 22 | 焊缝工艺 | EndZ | `Host/Seam/End/Z` | 上位机→KUKA | REAL | 4 | mm | ±10000 | 下载焊缝 | 原始终点 Z |
| 23 | 焊缝工艺 | EndA | `Host/Seam/End/A` | 上位机→KUKA | REAL | 4 | deg | ±180 | 下载焊缝 | 终点姿态 A |
| 24 | 焊缝工艺 | EndB | `Host/Seam/End/B` | 上位机→KUKA | REAL | 4 | deg | ±180 | 下载焊缝 | 终点姿态 B |
| 25 | 焊缝工艺 | EndC | `Host/Seam/End/C` | 上位机→KUKA | REAL | 4 | deg | ±180 | 下载焊缝 | 终点姿态 C |
| 26 | 焊缝工艺 | EndE1 | `Host/Seam/End/E1` | 上位机→KUKA | REAL | 4 | mm | 地轨行程 | 下载焊缝 | 终点地轨位置，通常与起点 E1 相同 |
| 27 | 焊缝工艺 | InsetMm | `Host/Seam/InsetMm` | 上位机→KUKA | REAL | 4 | mm | 0..半缝长 | 下载焊缝 | 沿焊缝从起终点各收回，对应「内缩」 |
| 28 | 焊缝工艺 | SpeedMmS | `Host/Seam/SpeedMmS` | 上位机→KUKA | REAL | 4 | mm/s | 0..1e5 | 焊接 | 焊接速度，对应「焊接速度」，默认 10 mm/s |
| 29 | 焊缝工艺 | WeaveMode | `Host/Seam/WeaveMode` | 上位机→KUKA | INT | 4 | - | 0直线 1摆动 | 焊接 | 对应「摆动方式」 |
| 30 | 焊缝工艺 | WeaveType | `Host/Seam/WeaveType` | 上位机→KUKA | INT | 4 | - | 0正弦 1三角 | 摆动焊 | 对应「摆动类型」；直线焊时忽略 |
| 31 | 焊缝工艺 | AmplitudeMm | `Host/Seam/AmplitudeMm` | 上位机→KUKA | REAL | 4 | mm | 0..1e4 | 摆动焊 | 侧向峰值偏移，对应「幅度」，默认 5 mm |
| 32 | 焊缝工艺 | ChordMm | `Host/Seam/ChordMm` | 上位机→KUKA | REAL | 4 | mm | >0 | 摆动焊 | 一个摆动周期沿焊缝的长度，对应「弦长」，默认 20 mm |
| 33 | 焊缝工艺 | MultiMode | `Host/Seam/MultiMode` | 上位机→KUKA | INT | 4 | - | 0单层单道 1多层多道 | 焊接 | 对应「多层多道」 |
| 34 | 焊缝工艺 | ThicknessMm | `Host/Seam/ThicknessMm` | 上位机→KUKA | REAL | 4 | mm | 0..1e4 | 多层多道 | 板厚，对应「板厚」 |
| 35 | 焊缝工艺 | GrooveDeg | `Host/Seam/GrooveDeg` | 上位机→KUKA | REAL | 4 | deg | 0..90 | 多层多道 | 坡口角度，对应「坡口角度」 |
| 36 | 焊缝工艺 | FitUpGapMm | `Host/Seam/FitUpGapMm` | 上位机→KUKA | REAL | 4 | mm | 0..1e3 | 多层多道 | 装配间隙，对应「装配间隙」 |
| 37 | 焊缝工艺 | PenetrationMm | `Host/Seam/PenetrationMm` | 上位机→KUKA | REAL | 4 | mm | 0..1e4 | 多层多道 | 熔深，对应「熔深」 |
| 38 | 焊道 | PassSeamId | `Host/Pass/SeamId` | 上位机→KUKA | INT | 4 | - | 1..256 | 下载焊道 | 本焊道所属焊缝 |
| 39 | 焊道 | PassLayer | `Host/Pass/Layer` | 上位机→KUKA | INT | 4 | - | 1..64 | 下载焊道 | 层号，打底层为 1 |
| 40 | 焊道 | PassLocal | `Host/Pass/LocalIndex` | 上位机→KUKA | INT | 4 | - | 1..32 | 下载焊道 | 层内道号 |
| 41 | 焊道 | PassSeq | `Host/Pass/Sequence` | 上位机→KUKA | INT | 4 | - | 1..1024 | 下载焊道 | 全局焊接顺序 |
| 42 | 焊道 | PassKind | `Host/Pass/Kind` | 上位机→KUKA | INT | 4 | - | 0打底 1填充 2盖面 | 下载焊道 | 焊道类型，决定焊枪倾角与规范 |
| 43 | 焊道 | PassStartX | `Host/Pass/Start/X` | 上位机→KUKA | REAL | 4 | mm | ±10000 | 下载焊道 | 本焊道起点 X（已含层/道偏移） |
| 44 | 焊道 | PassStartY | `Host/Pass/Start/Y` | 上位机→KUKA | REAL | 4 | mm | ±10000 | 下载焊道 | 本焊道起点 Y |
| 45 | 焊道 | PassStartZ | `Host/Pass/Start/Z` | 上位机→KUKA | REAL | 4 | mm | ±10000 | 下载焊道 | 本焊道起点 Z |
| 46 | 焊道 | PassStartA | `Host/Pass/Start/A` | 上位机→KUKA | REAL | 4 | deg | ±180 | 下载焊道 | 本焊道焊枪姿态 A |
| 47 | 焊道 | PassStartB | `Host/Pass/Start/B` | 上位机→KUKA | REAL | 4 | deg | ±180 | 下载焊道 | 本焊道焊枪姿态 B |
| 48 | 焊道 | PassStartC | `Host/Pass/Start/C` | 上位机→KUKA | REAL | 4 | deg | ±180 | 下载焊道 | 本焊道焊枪姿态 C |
| 49 | 焊道 | PassStartE1 | `Host/Pass/Start/E1` | 上位机→KUKA | REAL | 4 | mm | 地轨行程 | 下载焊道 | 本焊道地轨 |
| 50 | 焊道 | PassEndX | `Host/Pass/End/X` | 上位机→KUKA | REAL | 4 | mm | ±10000 | 下载焊道 | 本焊道终点 X |
| 51 | 焊道 | PassEndY | `Host/Pass/End/Y` | 上位机→KUKA | REAL | 4 | mm | ±10000 | 下载焊道 | 本焊道终点 Y |
| 52 | 焊道 | PassEndZ | `Host/Pass/End/Z` | 上位机→KUKA | REAL | 4 | mm | ±10000 | 下载焊道 | 本焊道终点 Z |
| 53 | 焊道 | PassEndA | `Host/Pass/End/A` | 上位机→KUKA | REAL | 4 | deg | ±180 | 下载焊道 | 本焊道终点姿态 A |
| 54 | 焊道 | PassEndB | `Host/Pass/End/B` | 上位机→KUKA | REAL | 4 | deg | ±180 | 下载焊道 | 本焊道终点姿态 B |
| 55 | 焊道 | PassEndC | `Host/Pass/End/C` | 上位机→KUKA | REAL | 4 | deg | ±180 | 下载焊道 | 本焊道终点姿态 C |
| 56 | 焊道 | PassEndE1 | `Host/Pass/End/E1` | 上位机→KUKA | REAL | 4 | mm | 地轨行程 | 下载焊道 | 本焊道终点地轨 |
| 57 | 焊道 | PassSpeedMmS | `Host/Pass/SpeedMmS` | 上位机→KUKA | REAL | 4 | mm/s | 0..1e5 | 焊接 | 本焊道焊接速度，可覆盖焊缝默认速度 |
| 58 | 轨迹 | TrajIndex | `Host/Traj/Index` | 上位机→KUKA | INT | 4 | - | 0..N-1 | 下载轨迹 | 当前轨迹点序号 |
| 59 | 轨迹 | TrajCount | `Host/Traj/Count` | 上位机→KUKA | INT | 4 | - | 2..4096 | 下载轨迹 | 本焊道轨迹点总数 |
| 60 | 轨迹 | TrajX | `Host/Traj/X` | 上位机→KUKA | REAL | 4 | mm | ±10000 | 下载轨迹 | 插值点 X（摆动后 TCP） |
| 61 | 轨迹 | TrajY | `Host/Traj/Y` | 上位机→KUKA | REAL | 4 | mm | ±10000 | 下载轨迹 | 插值点 Y |
| 62 | 轨迹 | TrajZ | `Host/Traj/Z` | 上位机→KUKA | REAL | 4 | mm | ±10000 | 下载轨迹 | 插值点 Z |
| 63 | 轨迹 | TrajA | `Host/Traj/A` | 上位机→KUKA | REAL | 4 | deg | ±180 | 下载轨迹 | 插值点姿态 A |
| 64 | 轨迹 | TrajB | `Host/Traj/B` | 上位机→KUKA | REAL | 4 | deg | ±180 | 下载轨迹 | 插值点姿态 B |
| 65 | 轨迹 | TrajC | `Host/Traj/C` | 上位机→KUKA | REAL | 4 | deg | ±180 | 下载轨迹 | 插值点姿态 C |
| 66 | 轨迹 | TrajE1 | `Host/Traj/E1` | 上位机→KUKA | REAL | 4 | mm | 地轨行程 | 下载轨迹 | 插值点地轨 |
| 67 | 轨迹 | TrajSpeedMmS | `Host/Traj/SpeedMmS` | 上位机→KUKA | REAL | 4 | mm/s | 0..1e5 | 焊接 | 该点进给速度 |
| 68 | 轨迹 | TrajFlag | `Host/Traj/Flag` | 上位机→KUKA | INT | 4 | - | bit 掩码 | 起弧/收弧 | bit0=起弧点 bit1=收弧点 bit2=本焊道末点 |
| 69 | 焊机 | ArcEnable | `Host/Welder/ArcEnable` | 上位机→KUKA | BOOL | 4 | - | 0/1 | 起弧 | 允许起弧；KUKA 在 AtStart 后置位焊机起弧输出 |
| 70 | 焊机 | GasEnable | `Host/Welder/GasEnable` | 上位机→KUKA | BOOL | 4 | - | 0/1 | 气体 | 允许送气 |
| 71 | 焊机 | CurrentA | `Host/Welder/CurrentA` | 上位机→KUKA | REAL | 4 | A | 0..500 | 焊接 | 焊接电流设定 |
| 72 | 焊机 | VoltageV | `Host/Welder/VoltageV` | 上位机→KUKA | REAL | 4 | V | 0..50 | 焊接 | 电弧电压设定 |
| 73 | 焊机 | WireMMin | `Host/Welder/WireMMin` | 上位机→KUKA | REAL | 4 | m/min | 0..25 | 焊接 | 送丝速度 |
| 74 | 焊机 | GasPreflowMs | `Host/Welder/GasPreflowMs` | 上位机→KUKA | INT | 4 | ms | 0..5000 | 气体预吹 | 起弧前保护气时间 |
| 75 | 焊机 | GasPostflowMs | `Host/Welder/GasPostflowMs` | 上位机→KUKA | INT | 4 | ms | 0..5000 | 气体滞后 | 收弧后保护气时间 |
| 76 | 焊机 | CraterMs | `Host/Welder/CraterMs` | 上位机→KUKA | INT | 4 | ms | 0..2000 | 收弧 | 填弧坑时间 |
| 77 | 状态回传 | KukaHeartbeat | `Kuka/Heartbeat` | KUKA→上位机 | INT | 4 | - | 0..2^31-1 | 全程 | 机器人心跳 |
| 78 | 状态回传 | CmdAckSeq | `Kuka/CmdAckSeq` | KUKA→上位机 | INT | 4 | - | 0..2^31-1 | 作业控制 | 已接受/已执行的命令序号 |
| 79 | 状态回传 | Phase | `Kuka/Phase` | KUKA→上位机 | INT | 4 | - | 0..18 | 全程 | 工艺阶段：地轨/接近/到位/预吹/起弧/焊接/收弧/回撤/回Home/故障 |
| 80 | 状态回传 | OpMode | `Kuka/OpMode` | KUKA→上位机 | INT | 4 | - | 0T1 1T2 2AUT 3EXT | 联机 | 运行模式，正式焊接应为 EXT |
| 81 | 状态回传 | ProActive | `Kuka/ProActive` | KUKA→上位机 | BOOL | 4 | - | 0/1 | 联机 | 解释器程序正在运行 |
| 82 | 状态回传 | DrivesOn | `Kuka/DrivesOn` | KUKA→上位机 | BOOL | 4 | - | 0/1 | 联机 | 驱动使能 |
| 83 | 状态回传 | EStop | `Kuka/EStop` | KUKA→上位机 | BOOL | 4 | - | 0/1 | 安全 | 急停 |
| 84 | 状态回传 | MsgId | `Kuka/MsgId` | KUKA→上位机 | INT | 4 | - | 0=无 | 故障 | KUKA 报警号或自定义故障码 |
| 85 | 状态回传 | FbSeamId | `Kuka/SeamId` | KUKA→上位机 | INT | 4 | - | 0..256 | 焊接 | 当前焊缝 |
| 86 | 状态回传 | FbLayer | `Kuka/Layer` | KUKA→上位机 | INT | 4 | - | 0..64 | 多层多道 | 当前层 |
| 87 | 状态回传 | FbPassSeq | `Kuka/PassSeq` | KUKA→上位机 | INT | 4 | - | 0..1024 | 多层多道 | 当前焊道全局序号 |
| 88 | 状态回传 | FbTrajIndex | `Kuka/TrajIndex` | KUKA→上位机 | INT | 4 | - | 0..N | 焊接 | 当前轨迹点 |
| 89 | 状态回传 | TcpX | `Kuka/Tcp/X` | KUKA→上位机 | REAL | 4 | mm | ±10000 | 全程 | 实际 TCP X |
| 90 | 状态回传 | TcpY | `Kuka/Tcp/Y` | KUKA→上位机 | REAL | 4 | mm | ±10000 | 全程 | 实际 TCP Y |
| 91 | 状态回传 | TcpZ | `Kuka/Tcp/Z` | KUKA→上位机 | REAL | 4 | mm | ±10000 | 全程 | 实际 TCP Z |
| 92 | 状态回传 | TcpA | `Kuka/Tcp/A` | KUKA→上位机 | REAL | 4 | deg | ±180 | 全程 | 实际姿态 A |
| 93 | 状态回传 | TcpB | `Kuka/Tcp/B` | KUKA→上位机 | REAL | 4 | deg | ±180 | 全程 | 实际姿态 B |
| 94 | 状态回传 | TcpC | `Kuka/Tcp/C` | KUKA→上位机 | REAL | 4 | deg | ±180 | 全程 | 实际姿态 C |
| 95 | 状态回传 | TcpE1 | `Kuka/Tcp/E1` | KUKA→上位机 | REAL | 4 | mm | 地轨行程 | 全程 | 实际地轨 |
| 96 | 状态回传 | ActSpeedMmS | `Kuka/ActSpeedMmS` | KUKA→上位机 | REAL | 4 | mm/s | ≥0 | 焊接 | 实际焊接速度 |
| 97 | 状态回传 | ArcOn | `Kuka/ArcOn` | KUKA→上位机 | BOOL | 4 | - | 0/1 | 起弧 | 焊机起弧成功反馈 |
| 98 | 状态回传 | Collision | `Kuka/Collision` | KUKA→上位机 | BOOL | 4 | - | 0/1 | 安全 | 碰撞/力矩超限 |
| 99 | 状态回传 | DownloadOk | `Kuka/DownloadOk` | KUKA→上位机 | BOOL | 4 | - | 0/1 | 下载作业 | 作业/焊缝/焊道/轨迹下载完成 |
| 100 | 状态回传 | JobDone | `Kuka/JobDone` | KUKA→上位机 | BOOL | 4 | - | 0/1 | 作业完成 | 全部焊缝焊完并已回 Home |
| 101 | 状态回传 | ProgressPct | `Kuka/ProgressPct` | KUKA→上位机 | REAL | 4 | % | 0..100 | 全程 | 当前作业进度 |

## 上位机→KUKA 报文示例

```xml
<Host><Heartbeat>1</Heartbeat><Cmd>3</Cmd><CmdSeq>1</CmdSeq><JobId>1</JobId><SeamCount>1</SeamCount><PassCount>1</PassCount><ToolNo>1</ToolNo><BaseNo>0</BaseNo><OverridePct>100</OverridePct><ApproachMm>50</ApproachMm><RetractMm>50</RetractMm><Seam><Id>1</Id><Start><X>0</X><Y>0</Y><Z>0</Z><A>0</A><B>90</B><C>180</C><E1>0</E1></Start><End><X>100</X><Y>0</Y><Z>0</Z><A>0</A><B>90</B><C>180</C><E1>0</E1></End><InsetMm>0</InsetMm><SpeedMmS>10</SpeedMmS><WeaveMode>1</WeaveMode><WeaveType>0</WeaveType><AmplitudeMm>5</AmplitudeMm><ChordMm>20</ChordMm><MultiMode>0</MultiMode><ThicknessMm>0</ThicknessMm><GrooveDeg>0</GrooveDeg><FitUpGapMm>0</FitUpGapMm><PenetrationMm>0</PenetrationMm></Seam><Pass><SeamId>1</SeamId><Layer>1</Layer><LocalIndex>1</LocalIndex><Sequence>1</Sequence><Kind>0</Kind><Start><X>0</X><Y>0</Y><Z>0</Z><A>0</A><B>0</B><C>0</C><E1>0</E1></Start><End><X>0</X><Y>0</Y><Z>0</Z><A>0</A><B>0</B><C>0</C><E1>0</E1></End><SpeedMmS>10</SpeedMmS></Pass><Traj><Index>0</Index><Count>2</Count><X>0</X><Y>0</Y><Z>0</Z><A>0</A><B>0</B><C>0</C><E1>0</E1><SpeedMmS>10</SpeedMmS><Flag>1</Flag></Traj><Welder><ArcEnable>1</ArcEnable><GasEnable>1</GasEnable><CurrentA>0</CurrentA><VoltageV>0</VoltageV><WireMMin>0</WireMMin><GasPreflowMs>200</GasPreflowMs><GasPostflowMs>400</GasPostflowMs><CraterMs>200</CraterMs></Welder></Host>
```

## KUKA→上位机 报文示例

```xml
<Kuka><Heartbeat>1</Heartbeat><CmdAckSeq>0</CmdAckSeq><Phase>4</Phase><OpMode>3</OpMode><ProActive>1</ProActive><DrivesOn>1</DrivesOn><EStop>0</EStop><MsgId>0</MsgId><SeamId>0</SeamId><Layer>0</Layer><PassSeq>0</PassSeq><TrajIndex>0</TrajIndex><Tcp><X>0</X><Y>0</Y><Z>0</Z><A>0</A><B>0</B><C>0</C><E1>0</E1></Tcp><ActSpeedMmS>0</ActSpeedMmS><ArcOn>0</ArcOn><Collision>0</Collision><DownloadOk>1</DownloadOk><JobDone>0</JobDone><ProgressPct>0</ProgressPct></Kuka>
```
