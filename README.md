# Cursor

KUKA EthernetKRL 优雅断开相关更新见分支 PR，主要文件：

- `device_robot/kukacommunicator.*` — `gracefulStop` / 非 abort 关 TCP
- `MainWindow.*` — 关机与断开按钮走协议退出
- `krl/` — `kukasend` / `kukarec` / `pcservice` / `RobotMotion.xml`
