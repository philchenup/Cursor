# T 型板焊缝定位

截图就是正交俯视点云。背景以外的每个像素是一个回波点，一共 1,539,126 个点，都在 z = 0 的扫描平面上。黑色区域没有点，那就是坡口。

中间焊缝是无点缺口的中线，两侧焊趾是缺口的两壁。板内的小孔和边缘毛刺不够长，不会被当成焊缝。缝宽不稳定的两端尖角会被丢掉，只保留缝宽平稳的那一段。

坐标原点在截图左上角，x 向右，y 向下，单位是截图上的 1 像素。

## 这张点云上的结果

缝宽沿整条缝的波动不到 1 个像素。

### 左下坡口，方向 49.04°

| | 起点 (x, y) | 终点 (x, y) |
| --- | --- | --- |
| 中间焊缝 | 304.23, 574.24 | 639.84, 960.92 |
| 黄板侧焊趾 | 332.06, 550.09 | 667.67, 936.76 |
| 紫板侧焊趾 | 276.40, 598.40 | 612.01, 985.07 |

缝宽 73.70 px，长度 512.00 px，缝宽 MAD 0.71 px。

### 右上坡口，方向 90.41°

| | 起点 (x, y) | 终点 (x, y) |
| --- | --- | --- |
| 中间焊缝 | 1028.38, 179.96 | 1024.66, 699.95 |
| 浅绿侧焊趾 | 1053.08, 180.14 | 1049.36, 700.12 |
| 蓝板侧焊趾 | 1003.68, 179.79 | 999.97, 699.77 |

缝宽 49.40 px，长度 520.00 px，缝宽 MAD 0.93 px。

标注图：`weld_seam/examples/t_plates_seams.png`。红线是中间焊缝，橙线和青线是两侧焊趾。

## 做法

1. 与四角同色的像素没有回波，其余像素收成点 `(x, y, 0)`。
2. 从点云外轮廓灌进无点区域，留下来的内部空洞才是候选坡口。
3. 只保留又长又细的空洞。圆孔和一两像素宽的划痕直接丢掉。
4. 沿空洞方向分箱，每箱取 2% 和 98% 分位作为两壁，中线取两壁的平均。
5. 缝宽用中位数。偏离过大的箱子用 MAD 丢掉，端点落在最长的一段稳定缝宽上。

立板有点时用 `tjoint_seams_3d`：RANSAC 抽出翼板，把立板投影到翼板上，再做同一套边缘包络。一面点多、一面点少时，中心线仍在两条边缘中间。

## 用法

```bash
python -m weld_seam weld_seam/examples/t_plates.png \
  -o weld_seam/examples/t_plates_seams.png \
  --json weld_seam/examples/t_plates_seams.json

python -m unittest tests.test_weld_seam
```

依赖见 `weld_seam/requirements.txt`。

## 补全同一平面上的缺口

黄、绿两块是同一平面的点云，中间没有点，所以被欧式聚类分成两个连通域。`fillPlanarGap` 只在两块正对、缝宽稳定的那一段里补点，新点落在两块质心连线所在的平面上，不把外轮廓往外撑。

```bash
g++ -std=c++17 -O2 -Iinclude src/FillPlanarGap.cpp src/FillPlanarGapDemo.cpp \
  -o fill_planar_gap $(pkg-config --cflags --libs pcl_segmentation pcl_search pcl_kdtree pcl_common)

python weld_seam/fill_screenshot_gap.py weld_seam/examples/yg_plates.png ./fill_planar_gap \
  -o weld_seam/examples/yg_gap_filled.png
```

这张截图补了 39199 个点，都在 z = 0，落在两块之间的黑色缺口里。下图里红色是补上的点。
