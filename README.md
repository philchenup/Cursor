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
`pcl::PointCloud<pcl::PointXYZ>`（单位 mm），输出每个工件的底板平面、顶部轮廓直线，以及平角焊缝 / 立角焊缝
的起止点、接近方向和置信度。

平角焊缝是立板顶面点云投影到底板上的轮廓直线段，立角焊缝是该轮廓凹拐角处从底板到顶部的垂线段。
流程为确定性算法（高度图 + 直方图 + 最小二乘平面 + 形态学 + 轮廓直线拟合），不依赖随机采样，也不估计板厚。

默认按相机坐标系处理：Z 轴指向地面（地面 Z 最大，工件越高 Z 越小）。内部翻转为 Z 向上计算，输出再翻转回
输入坐标系，因此焊缝坐标可直接与点云对应。点云已经是 Z 向上时，把 `zAxisDown` 设为 `false`。

```cpp
#include "SceneSeamExtractor.h"

SceneSeamParams params;              // 默认参数适用于 mm 单位、底板 ≥ 0.2 m 的工件
params.zAxisDown = true;             // 相机坐标系（默认）；世界坐标系 Z 向上时设为 false
SceneSeamResult result = ExtractSceneSeams(cloud, params);
for (const Workpiece& piece : result.workpieces)
    for (const InitialSeam& seam : piece.seams)
        /* seam.start / seam.end / seam.approachSide / seam.confidence */;
```

编译与测试（已安装 PCL）：

```bash
cmake -S . -B build -DCMAKE_CXX_COMPILER=g++ -DCMAKE_C_COMPILER=gcc
cmake --build build -j
./build/scene_seam_test                # 合成 19 个工件的相机坐标系场景（Z 指向地面），通过检查后打开可视化窗口
./build/scene_seam_test --no-viewer    # 仅运行检查
```

直接处理拼接后的整场 PLY 点云（跳过合成场景和真值检查）：

```bash
./build/scene_seam_test --ply scene.ply                        # 相机坐标系（Z 指向地面），单位 mm
./build/scene_seam_test --ply scene.ply --z-up                 # 点云 Z 轴向上
./build/scene_seam_test --ply scene.ply --scale 1000           # 点云单位 m，内部换算成 mm
./build/scene_seam_test --ply scene.ply --export seams.csv     # 焊缝写入 CSV（输入坐标系）
./build/scene_seam_test --ply scene.ply --no-viewer            # 只打印结果不开窗口
./build/scene_seam_test --no-viewer --save-ply scene.ply       # 把合成场景按相机坐标系另存为 PLY，用于验证流程
```

Z 轴方向设错的典型症状：只分割出一个覆盖整个视场的“工件”，真实工件在其点云中表现为空洞，焊缝为 0 条。

常用参数覆盖（单位 mm）：`--voxel 4`、`--rib-min-height 20`、`--min-seam-length 30`、`--min-area 50000`。
`ribMinHeight` 要高于底板厚度、低于最矮立板；工件外形变小时再调 `minWorkpieceArea` / `minWorkpieceSize`。

可视化窗口上半部分为整场点云（按高度着色、工件外接框和编号、焊缝叠加），下半部分为单个工件细节，
按 `n` / `p` 切换工件；红色为平角焊缝，橙色为立角焊缝，青色为顶部轮廓直线。
