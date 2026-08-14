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

用 `QRadioButton` 切换时，接 `toggled(bool)`（不要用 `clicked`，改选另一项时当前按钮不会 clicked）：

```cpp
connect(ui->radioButtonTcp, &QRadioButton::toggled,
        operationalModel, &OperationalModel::setDisplayTcp);
ui->radioButtonFlange->setChecked(true);  // 默认法兰
```

选中 TCP → `setDisplayTcp(true)`；选中法兰 → TCP 被取消选中 → `setDisplayTcp(false)`。完整片段见 `snippets/MainWindow_tcpRadio.cpp`。

显示：`T_base_tcp = T_base_flange * T`（`OperationalModel::flangeToTcp`）
编辑/点动：增量沿**当前坐标系**施加（TCP 模式沿 TCP 轴，否则沿法兰轴），IK 前再变回法兰 `T_display * offset` → `tcpToFlange`。
