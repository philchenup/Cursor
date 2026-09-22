# 用 PROFINET 读写 KUKA 外部轴 / 关节 / TCP

上位机走赫优讯 CIFX（你现有的 `xChannelIORead` / `xChannelIOWrite`），KUKA 作 PROFINET Device。PROFINET **没有** `$AXIS_ACT` 这种符号地址，只有过程数据字节。机器人要把 REAL 拷到 32 位 `$OUT`/`$IN`，上位机按字节偏移解 FLOAT32。

## 读什么、多大

| 内容 | 个数 | 类型 | 字节 |
| --- | --- | --- | --- |
| 外部轴 E1 E2 E3 | 3 | FLOAT32 | 12 |
| 关节角 A1–A6 | 6 | FLOAT32 | 24 |
| TCP 位姿 X Y Z A B C | 6 | FLOAT32 | 24 |
| **合计** | **15** | | **60** |
| CIFX 帧（`DATALENGTH`） | | | **64**（末尾 4 字节保留） |

这和 CIFX 示例里的 `DATALENGTH 64` 对齐。有效数据是前 60 字节。

## 字节布局（读、写同一顺序）

| 字节偏移 | 信号 | KRL | 单位 | 机器人发出 `$OUT` | 上位机下发 `$IN` |
| --- | --- | --- | --- | --- | --- |
| 0 | E1 | `$AXIS_ACT.E1` | mm/deg | 1..32 | 1..32 |
| 4 | E2 | `$AXIS_ACT.E2` | mm/deg | 33..64 | 33..64 |
| 8 | E3 | `$AXIS_ACT.E3` | mm/deg | 65..96 | 65..96 |
| 12 | A1 | `$AXIS_ACT.A1` | deg | 97..128 | 97..128 |
| 16 | A2 | `$AXIS_ACT.A2` | deg | 129..160 | 129..160 |
| 20 | A3 | `$AXIS_ACT.A3` | deg | 161..192 | 161..192 |
| 24 | A4 | `$AXIS_ACT.A4` | deg | 193..224 | 193..224 |
| 28 | A5 | `$AXIS_ACT.A5` | deg | 225..256 | 225..256 |
| 32 | A6 | `$AXIS_ACT.A6` | deg | 257..288 | 257..288 |
| 36 | X | `$POS_ACT.X` | mm | 289..320 | 289..320 |
| 40 | Y | `$POS_ACT.Y` | mm | 321..352 | 321..352 |
| 44 | Z | `$POS_ACT.Z` | mm | 353..384 | 353..384 |
| 48 | A | `$POS_ACT.A` | deg | 385..416 | 385..416 |
| 52 | B | `$POS_ACT.B` | deg | 417..448 | 417..448 |
| 56 | C | `$POS_ACT.C` | deg | 449..480 | 449..480 |

方向（KUKA = Device，CIFX = Controller）：

- **读**机器人：`xChannelIORead` ← 机器人 `$OUT[1..480]`
- **写**机器人：`xChannelIOWrite` → 机器人 `$IN[1..480]`

## WorkVisual

1. 给 KUKA PN Device 加一块至少 **64 字节** 的用户 IO 模块（输入、输出各一块）。
2. I/O Mapping：`$OUT[1]..$OUT[480]` 映到输出模块字节 0..59；`$IN[1]..$IN[480]` 映到输入模块字节 0..59。
3. 部署到控制器。CIFX/Sycon.NET 侧 IO 长度同样 ≥ 64。

KRL 不能把 `REAL` 直接接到 `$IN/$OUT`。每个值必须是 32 位 `SIGNAL`，用 `CAST_TO`（REAL→INT 位型）发出、`CAST_FROM`（INT→REAL）收回。

把 [`kuka/profinet/kuka_pn_axis_pose.dat`](../kuka/profinet/kuka_pn_axis_pose.dat) 和 [`.src`](../kuka/profinet/kuka_pn_axis_pose.src) 拷到 `C:\KRC\ROBOTER\KRC\R1\`，在 `sps.sub` 的循环里调用：

```
kuka_pn_axis_pose()
```

Submit 里只做 IO 拷贝，不要 PTP。运动程序读取 `CAST_FROM` 得到的 `cmd_*` 再插补。

## 上位机 CIFX（改你的 ChannelDemo）

`DATALENGTH` 保持 64。把原来只 `DumpData` 的循环换成解码/编码：

```cpp
#include "KukaProfinetIo.h"

unsigned char abSendData[kKukaPnFrameBytes] = {0};
unsigned char abRecvData[kKukaPnFrameBytes] = {0};

lRet = xChannelIORead(hChannel, 0, 0, sizeof(abRecvData), abRecvData, IO_WAIT_TIMEOUT);
if (lRet == CIFX_NO_ERROR) {
    KukaAxisPose fb;
    if (decodeKukaPnAxisPose(abRecvData, sizeof(abRecvData), &fb)) {
        qDebug() << "E1 E2 E3" << fb.e1 << fb.e2 << fb.e3;
        qDebug() << "A1..A6" << fb.a1 << fb.a2 << fb.a3 << fb.a4 << fb.a5 << fb.a6;
        qDebug() << "TCP XYZABC" << fb.x << fb.y << fb.z << fb.a << fb.b << fb.c;
    }

    KukaAxisPose cmd = fb;   // 示例：把当前值写回；改 cmd.x 等即下发目标
    encodeKukaPnAxisPose(cmd, abSendData, sizeof(abSendData));
    xChannelIOWrite(hChannel, 0, 0, sizeof(abSendData), abSendData, IO_WAIT_TIMEOUT);
}
```

单字段：`fb.a1` 在缓冲偏移 `kukaPnFieldOffset(KukaPnField::A1)`（12）。

若解码出 NaN 或数量级不对，把最后参数改成 `KukaPnEndian::Big`（WorkVisual 按大端 DWORD 映射时）。

`xChannelIORead/Write` 的第二个参数是 area（0 = 过程数据），第三个是 **字节偏移**。也可以只读关节：

```cpp
float a1;
xChannelIORead(hChannel, 0, 12, 4, &a1, IO_WAIT_TIMEOUT);  // 仅 A1，需本机小端
```

推荐整帧 64 字节一次读写，再用 `decodeKukaPnAxisPose`。

## 和 EKI 64 字节表的区别

焊接通讯表的 64 字节是心跳/命令/寻缝点。PROFINET 这 64 字节是 **E1–E3 + A1–A6 + XYZABC**。两套地址不要混在同一 IO 模块里，除非重新排表。
