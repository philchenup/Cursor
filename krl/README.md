# KUKA EthernetKRL 优雅断开说明

## 问题

上位机原先在断开时直接 `QTcpSocket::abort()`，且未发送 `IsOut=TRUE`，示教器会报通信断开错误。

## 正确时序

1. PC: `gracefulStop()` → 发送 `<IsOut>TRUE</IsOut>`
2. `kukarec`: `EKI_Close("RobotMotion")`（不 `EXIT`，继续 LOOP）
3. `$FLAG[7]` → FALSE → `kukasend` 停发；`pcservice` 再次 `EKI_Open`
4. PC: `disconnectFromHost()`

## 文件

| 文件 | 作用 |
|------|------|
| `RobotMotion.xml` | EKI 配置（ALIVE→FLAG[7]，Sensor→FLAG[8]） |
| `pcservice.src` | Submit：Init/Open/等连/等断/再 Open |
| `kukasend.src` | 仅 `$FLAG[7]` 时发送状态（不用 FLAG[9]） |
| `kukarec.src` | 收指令；`IsOut` 时 `EKI_Close`，不退出程序 |

## 上位机对应改动

- `device_robot/kukacommunicator.*`：新增 `gracefulStop()`；`stop()` 改为 `disconnectFromHost`；报文增加 `<Velocity>`（`setPtpVelocity` / `stepMove(..., vel)`）
- `MainWindow.*`：断开按钮 / `closeEvent` / 析构调用 `shutdownRobotComm()` → `gracefulStop`

## 速度下发

1. XML：`Sensor/Velocity`（REAL，PTP 百分比 1~100）
2. PC：默认 30%，可用 `setPtpVelocity(50)` 或 `stepMove(mode, xp2, 50)`
3. `kukarec`：`EKI_GetReal(...Velocity...)` → `BAS(#VEL_PTP/ACC_PTP, Velocity)` → `PTP`
