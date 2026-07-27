# Open3D 直线 / 圆 / 平面点云拟合示例

用 Python + Open3D 生成带噪声的直线、圆、平面点云，分别拟合并可视化结果。

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

平面拟合可选 PCA（默认）或 Open3D RANSAC：

```bash
python fit_line_circle.py --no-show --plane-method pca
python fit_line_circle.py --no-show --plane-method ransac
```

结果图默认保存到 `output/line_circle_plane_fit.png`。

## 方法说明

- **直线拟合**：对点云做 PCA，取第一主成分作为直线方向，质心作为直线上一点。
- **圆拟合**：先估计点云平面法向，投影到平面后做代数最小二乘圆拟合，再变换回 3D。
- **平面拟合**：
  - `pca`：SVD 取最小奇异向量为法向，质心为平面上一点。
  - `ransac`：调用 `PointCloud.segment_plane`。
- **可视化**：
  - 蓝色：线点云；绿色：拟合直线
  - 橙色：圆点云；红色：拟合圆与圆心
  - 紫色：平面点云；绿色网格：拟合平面
