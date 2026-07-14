# 虚拟示教器 + KUKA Router + 上位机客户端

## 角色
- 机器人 EKI = TCP Server（Xml_motion16: TYPE=Client, PORT=54600）
- 上位机 = TCP Client（connectToHost）
- Router = 把宿主机端口转发到 VxWorks 上的 EKI

## 关键：两个 IP 不是一回事
| 地址 | 是什么 | 用在哪 |
|---|---|---|
| 192.168.1.147 | 虚拟示教器 Windows/界面显示 IP | 不是 EKI 真正监听地址 |
| 192.168.0.1 | VxWorks/RTOS（EKI 实际所在） | Xml INTERNAL/IP、Router Target Host |

因此 Xml_motion16.xml 建议：
```xml
<INTERNAL>
  <IP>192.168.0.1</IP>
  <PORT>54600</PORT>
  ...
</INTERNAL>
```
（不要把 INTERNAL/IP 写成 192.168.1.147）

## Router 怎么填
| 项 | 填什么 |
|---|---|
| Target Host | `192.168.0.1` |
| Source Port | `54600`（外部可连的端口，可与 Target 相同） |
| Target Port | `54600`（必须等于 Xml INTERNAL/PORT） |
| Protocol | TCP |

即：**Router 端口号填 54600**（与 Xml_motion16 一致）。

Network card interface index：从 0 起试，直到 Router 显示监听在你要用的网卡 IP 上。

## 上位机（外部连接）怎么填
| 项 | 填什么 |
|---|---|
| IP | **运行 Router 的那台电脑/网卡 IP**（不是 192.168.0.1，也通常不是 192.168.1.147） |
| Port | `54600`（= Router 的 Source Port） |

例：Router 与上位机在同一台 Windows 上 →
```text
IP   = 127.0.0.1   （或 Router 显示的监听 IP）
Port = 54600
comm->start("127.0.0.1", 54600);
```

若上位机在另一台电脑 → 填 **Router 所在主机** 在那张网卡上的 IP，端口仍为 54600。

## 启动顺序
1. 启动虚拟示教器 / OfficeLite
2. 启动 Router（路由已添加）
3. Submit(SPS) 执行 EKI_Init + EKI_Open（开始监听）
4. 上位机 connect
5. 再运行 motion16()

## 自检
- Router 状态应变绿 / Connected（不要一直黄 Connecting）
- 示教器 `$FLAG[2]`（ALIVE）应变 TRUE
- 若连不上：查防火墙是否放行 54600，以及 Interface Index 是否选对
