# 堆叠过滤（去压叠）— PCL / C++

抓取场景下，配准会给出多个目标位姿。本模块用 **模板点云 + 配准位姿** 还原每个实例在场景中的占用，去掉被上层压住的目标，只保留最上层可抓取实例。

默认按 **毫米点云、相机 +Z 朝下拍摄**（眼在手 / 俯视料框）：离相机越近 Z 越小，越算「在上面」。

核心实现：`stack_filter/stacked_object_filter.*`  
命令行：`apps/stack_filter/`

## 参考项目 / 算法出处

| 来源 | 步骤 | 本模块对应 |
|------|------|------------|
| [Mech-Vision 去除被压叠的物体 V2](https://docs.mech-mind.net/en/suite-software-manual/1.8.3/vision-steps/remove-overlapped-objects-v2.html) | Projection (2D) / Bounding Box (3D) | `projection_2d` / `bounding_box_3d` |
| [Mech-Vision 去除被压叠的物体（易用版）](https://docs.mech-mind.net/zh/suite-software-manual/latest/vision-steps/remove-overlapped-objects-lite.html) | 压叠比例 = 上方投影面积 / 模板投影面积 | 默认公式 |
| [TransferTech Extract Top-layer Point Clouds](https://docs.transfertech.cn/tf_docs/en/atom/1.3.1.1/operator/point_cloud/filter/PointsUpJudge.html) | Z 轴朝下 / 最高层高度 | `camera_z_down` + `highest_layer` |
| PCL `transformPointCloud` / `VoxelGrid` | 位姿变换与下采样 | 实现路径 |

## 算法

对每个实例：`world = T_pose * template`（单位与点云一致，默认 mm）。

### `projection_2d`（默认，法兰/面模板）

1. 「朝上」方向默认 **-Z**（相机 +Z 朝下）。高度 `h = p · up`，因此 **Z 更小的点更高**。
2. 在 XY 上按 `pixel_size` 正交投影（默认约模板 XY 对角线的 2%，法兰约 2.5 mm）。
3. 每个格子记录该实例的最大高度；再对各实例占用做 1 格膨胀，避免圆环孔隙漏检。
4. 若全局最高高度比本实例高出 `height_margin`（默认 3 mm），该格视为被压住。
5. `overlap_ratio = 被压住格数 / 本实例占用格数`，超过 0.30 则丢弃。

并排接触、互不在上方 → 都保留。  
完全叠放 → 只留离相机更近的那个。  
两堆高度不同 → **每堆顶层都保留**。

### `bounding_box_3d` / `highest_layer`

包围盒填实体素后再做同样的「上方占用」判断；或再加一层全局最高层高度门控。

输出按 `mean_height`（朝相机）从高到低排序，便于从上往下抓。

## 构建与运行

```bash
cd filterreg_rigid
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=g++
cmake --build build --target stack_filter -j

./build/stack_filter --self-test
./build/stack_filter --demo --out-dir .
./build/stack_filter --template flange.pcd --poses poses.json --out-dir ./out
```

`--demo` 生成 5 个 φ110 mm 法兰（中心孔 + 4 螺栓孔 + 缺口），相机 Z 朝下、单位 mm。期望保留 **top_left / bot_left / top_right**，丢掉被压住的 **center / bot_right**。

## JSON 位姿

```json
{
  "method": "projection_2d",
  "units": "mm",
  "camera_z_down": true,
  "overlap_ratio_threshold": 0.3,
  "targets": [
    {"id": "A", "translation": [0, 0, 345], "quaternion_wxyz": [1, 0, 0, 0]}
  ]
}
```

`translation` 与点云同单位。`camera_z_down: true` 时不要再把 `up_axis` 设成 `[0,0,1]`。

C++：

```cpp
poser::StackFilterParams p;          // mm + camera Z down
p.overlap_ratio_threshold = 0.30f;
poser::StackedObjectFilter filter(p);
auto out = filter.Filter(instances); // model + Eigen::Matrix4f pose
```

米制、重力 +Z 向上时：`--units m --camera-z-up`。
