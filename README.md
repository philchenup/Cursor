# Open3D 直线 / 圆点云拟合示例

用 Python + Open3D 生成带噪声的直线点云与圆点云，分别拟合直线和圆，并可视化结果。

## 环境

```bash
pip install -r requirements.txt
```

## 运行

交互可视化（需要图形界面）：

```bash
python fit_line_circle.py
```

无界面环境（仅拟合并保存结果图）：

```bash
python fit_line_circle.py --no-show
```

结果图默认保存到 `output/line_circle_fit.png`。

## 方法说明

- **直线拟合**：对点云做 PCA，取第一主成分作为直线方向，质心作为直线上一点。
- **圆拟合**：先估计点云平面法向，投影到平面后做代数最小二乘圆拟合，再变换回 3D。
- **可视化**：Open3D 中蓝色为线点云、绿色为拟合直线；橙色为圆点云、红色为拟合圆与圆心。
