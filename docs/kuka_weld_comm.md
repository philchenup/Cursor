# 上位机 ↔ KUKA 焊接通讯规范

上位机按焊缝工艺表规划轨迹，通过 **KUKA.Ethernet KRL (EKI)** 与 KR C 控制器交换作业、工艺和状态。机器人作为 TCP **Client** 连接上位机 Server，报文为嵌套 XML，XPath 与通讯表一致。

完整 101 行信号表见 [`kuka_weld_comm_table.csv`](kuka_weld_comm_table.csv)，EKI 配置见 [`../kuka/EthernetKRL/WeldHost.xml`](../kuka/EthernetKRL/WeldHost.xml)。C++ 结构在 `include/KukaWeldComm.h`。

## 1. 焊接流程与通讯阶段

与现有工艺一致：焊缝表给出起终点、内缩、速度、摆动（正弦/三角、幅度、弦长）、多层多道（板厚、坡口角、装配间隙、熔深）；V 坡口规划展开为层/道；运动为地轨对齐焊点 Y → 接近 → 焊接 → 回撤 → 回 Home。

```
上位机                              KUKA (EXT)
  |  心跳 Heartbeat / Heartbeat        |
  |  DownloadJob → CmdAckSeq           |
  |  DownloadSeam（工艺表一行）        |
  |  DownloadPass（层/道，多层时）     |
  |  DownloadTraj（摆动插值点）        |
  |  Start                             |
  |                    RailMove 只动 E1
  |                    Approach 沿 TCP-Z
  |                    AtStart 内缩后起点
  |                    GasPreflow
  |                    ArcStarting / ArcOn=1
  |                    Welding 直线或摆动
  |                    Crater → GasPostflow
  |                    Retract
  |                    BetweenPass 下一道/层
  |                    ReturnHome
  |                    JobDone=1
```

异常：`Stop` 立即停运动；`ArcOff` 先收弧再停；急停/碰撞时 `Phase=EStop/Fault`，上位机发 `AckFault` 后才能继续。

## 2. 链路与帧格式

| 项 | 约定 |
| --- | --- |
| 物理/协议 | 以太网 TCP，KUKA.Ethernet KRL |
| 角色 | 上位机 Server `192.168.1.100:54600`，KUKA Client |
| 循环周期 | 状态/命令 20–50 ms；轨迹可在 Start 前一次性下载 |
| 单位 | mm、deg（KUKA ABC）、mm/s、A、V、m/min、ms |
| 位姿 | `E6POS`：X Y Z A B C + 地轨 E1 |
| 轨迹 CSV | 组内 `,` 组间 `;` 结尾 `.`，每点 `X,Y,Z,A,B,C,E1,Speed,Flag` |

命令靠 **CmdSeq 边沿** 触发，避免心跳刷新导致重复执行。KUKA 用 `CmdAckSeq` 回显已接受序号。

### 命令字 `Host/Cmd`

| 值 | 名称 | 含义 |
| --- | --- | --- |
| 0 | Idle | 无新命令 |
| 1 | Reset | 清作业缓冲、关弧、回 Idle |
| 2 | DownloadJob | 写入作业头（焊缝数、焊道数、TOOL/BASE、接近/回撤） |
| 3 | DownloadSeam | 写入当前焊缝工艺（对应工艺表一行） |
| 4 | DownloadPass | 写入当前焊道（层、道、打底/填充/盖面） |
| 5 | DownloadTraj | 写入当前轨迹点（或配合 CSV 批量） |
| 6 | Start | 从当前作业第一条未焊焊缝开始 |
| 7 | Pause | 暂停插补，电弧策略由安全规程决定 |
| 8 | Resume | 继续 |
| 9 | Stop | 停止运动 |
| 10 | ArcOff | 强制收弧 |
| 11 | GoHome | 回 Home（先手臂后地轨） |
| 12 | AckFault | 上位机确认故障已处理 |

### 工艺阶段 `Kuka/Phase`

Disconnected → Connected → Idle → Downloading → Ready → RailMove → Approach → AtStart → GasPreflow → ArcStarting → Welding → Crater → GasPostflow → Retract → BetweenPass（循环）→ ReturnHome → JobDone。Fault / EStop 可从任意阶段进入。

## 3. 与焊缝工艺表的字段映射

| 工艺表列 | 通讯信号 | 默认 |
| --- | --- | --- |
| 序号 | `Host/Seam/Id` | — |
| 起点 | `Host/Seam/Start/{X,Y,Z,A,B,C,E1}` | 内缩前原始点 |
| 终点 | `Host/Seam/End/{X,Y,Z,A,B,C,E1}` | 内缩前原始点 |
| 内缩 | `Host/Seam/InsetMm` | 0 mm，沿焊缝两端各收回 |
| 焊接速度 | `Host/Seam/SpeedMmS` | 10 mm/s |
| 摆动方式 | `Host/Seam/WeaveMode` | 0 直线焊 / 1 摆动焊 |
| 摆动类型 | `Host/Seam/WeaveType` | 0 正弦 / 1 三角 |
| 幅度 | `Host/Seam/AmplitudeMm` | 5 mm |
| 弦长 | `Host/Seam/ChordMm` | 20 mm（一个摆动周期沿焊缝长度） |
| 多层多道 | `Host/Seam/MultiMode` | 0 单层单道 / 1 多层多道 |
| 板厚 | `Host/Seam/ThicknessMm` | 规划层数用 |
| 坡口角度 | `Host/Seam/GrooveDeg` | 0–90° |
| 装配间隙 | `Host/Seam/FitUpGapMm` | — |
| 熔深 | `Host/Seam/PenetrationMm` | — |

单层单道：一条焊缝对应一条焊道，轨迹由起终点（内缩后）直线或摆动插值。多层多道：上位机按板厚/坡口/间隙/熔深展开焊道，再逐条 `DownloadPass` + 轨迹。

`TrajFlag`：bit0 起弧点，bit1 收弧点，bit2 本焊道末点。

## 4. 数据结构（C++）

循环报文打包为 `HostCyclic` / `KukaCyclic`，内含作业头、当前焊缝、当前焊道、当前轨迹点、焊机规范。下载阶段通过切换 `Cmd` 与对应子结构刷新同一 XML 骨架，避免 EKI 多套配置。

焊机电流/电压/送丝经 KUKA 数字量/模拟量转发焊机；上位机只写设定，起弧成功看 `Kuka/ArcOn`。

## 5. 完整通讯表

见 [`kuka_weld_comm_table.md`](kuka_weld_comm_table.md)（Markdown）或 [`kuka_weld_comm_table.csv`](kuka_weld_comm_table.csv)（Excel）。信号分为六组：**握手控制、焊缝工艺、焊道、轨迹、焊机、状态回传**。
