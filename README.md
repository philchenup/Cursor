# 法兰位姿 → TCP 位姿

`getOperationalPosition()` 给出的是法兰在基座下的位姿 `T_base_flange`。
TCP 旋转矩阵 `R_flange_tcp` 表示 TCP 坐标轴在法兰坐标系中的方向：

```
T_base_tcp = T_base_flange * T_flange_tcp
```

只有旋转、原点仍在法兰中心时，XYZ 不变，ABC 会变。若 TCP 相对法兰还有平移，把偏移传入 `t_flange_tcp`。

## 显示（OperationalModel::data）

```cpp
#include "tcp_pose.h"

const rl::math::Transform& T_flange =
    MainWindow::instance()->mdl->getOperationalPosition(index.column());
rl::math::Transform T_tcp = flangeToTcp(T_flange, R_tcp); // 用户提供的 3x3
// 有刀尖偏移时：flangeToTcp(T_flange, R_tcp, t_tcp);

rl::math::Transform::ConstTranslationPart position = T_tcp.translation();
rl::math::Vector3 orientation = T_tcp.rotation().eulerAngles(2, 1, 0).reverse();
```

## 编辑后再做 IK（OperationalModel::setData）

表格里改的是 TCP，IK 仍要法兰目标：

```cpp
rl::math::Transform T_tcp = /* 按原逻辑用表格 XYZABC 拼出的位姿 */;
rl::math::Transform T_flange = tcpToFlange(T_tcp, R_tcp);
ik->addGoal(T_flange, i);
```
