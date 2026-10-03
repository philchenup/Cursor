# Cursor

## ScaleAISShapeBy1000

将 OpenCASCADE 的 `AIS_Shape*` 缩小 1000 倍，并返回一个新的 `AIS_Shape*`。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
AIS_Shape* scaled = ScaleAISShapeBy1000(ais);

// 推荐用 Handle 接管返回值，避免泄漏
Handle(AIS_Shape) scaledHandle = ScaleAISShapeBy1000(ais);
```

## SceneSeamExtractor

整场点云（约 3 m × 18 m，乱序摆放多个组立工件）的初始焊缝提取。输入为相机垂直向下拍摄并拼接后的
`pcl::PointCloud<pcl::PointXYZ>`（单位 mm），输出每个工件的底板平面、立板中心线、以及平角焊缝 / 立角焊缝
的起止点、接近方向和置信度。流程全部为确定性算法（高度图 + 直方图 + 最小二乘平面 + 形态学 + Hough），
不依赖随机采样。

```cpp
#include "SceneSeamExtractor.h"

SceneSeamParams params;              // 默认参数适用于 mm 单位、底板 ≥ 0.2 m 的工件
SceneSeamResult result = ExtractSceneSeams(cloud, params);
for (const Workpiece& piece : result.workpieces)
    for (const InitialSeam& seam : piece.seams)
        /* seam.start / seam.end / seam.approachSide / seam.confidence */;
```

编译与测试（已安装 PCL）：

```bash
cmake -S . -B build -DCMAKE_CXX_COMPILER=g++ -DCMAKE_C_COMPILER=gcc
cmake --build build -j
./build/scene_seam_test                # 合成 19 个工件的场景，通过检查后打开可视化窗口
./build/scene_seam_test --no-viewer    # 仅运行检查
```

可视化窗口上半部分为整场点云（按高度着色、工件外接框和编号、焊缝叠加），下半部分为单个工件细节，
按 `n` / `p` 切换工件；红色为平角焊缝，橙色为立角焊缝，青色为立板中心线。
