# 法兰 / TCP 坐标显示

`OperationalModel` 默认仍显示法兰。外部可切换 TCP，并传入法兰→TCP 齐次变换 `T`：

```cpp
rl::math::Transform T = rl::math::Transform::Identity();
T.linear() = R_tcp;          // 法兰系下 TCP 姿态
T.translation() = t_tcp;     // 法兰系下 TCP 原点，无可省略（0）

operationalModel->setToolTransform(T);
operationalModel->setDisplayTcp(true);   // 表格显示 TCP
operationalModel->setDisplayTcp(false);  // 回到法兰
```

显示：`T_base_tcp = T_base_flange * T`（`OperationalModel::flangeToTcp`）
编辑/点动：表格改的是当前显示坐标系，IK 前再变回法兰 `T_base_tcp * T⁻¹`（`OperationalModel::tcpToFlange`）。
