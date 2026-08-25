# 堆叠过滤（去压叠）— PCL / C++

抓取场景下，配准会给出多个目标位姿。本模块用 **模板点云 + 配准位姿** 还原每个实例在场景中的占用，去掉被上层压住的目标，只保留最上层可抓取实例。

核心实现：`stack_filter/stacked_object_filter.*`  
命令行：`apps/stack_filter/`

## 参考项目 / 算法出处

公开的工业视觉软件里，这一步通常接在 3D 精匹配之后：

| 来源 | 步骤 | 本模块对应 |
|------|------|------------|
| [Mech-Vision 去除被压叠的物体 V2](https://docs.mech-mind.net/en/suite-software-manual/1.8.3/vision-steps/remove-overlapped-objects-v2.html) | Projection (2D) / Bounding Box (3D) | `projection_2d` / `bounding_box_3d` |
| [Mech-Vision 去除被压叠的物体（易用版）](https://docs.mech-mind.net/zh/suite-software-manual/latest/vision-steps/remove-overlapped-objects-lite.html) | 压叠比例 = 上方投影面积 / 模板投影面积 | 默认公式 |
| [TransferTech Extract Top-layer Point Clouds](https://docs.transfertech.cn/tf_docs/en/atom/1.3.1.1/operator/point_cloud/filter/PointsUpJudge.html) | 最高层高度 / ROI 上方点数 | `highest_layer` |
| [TransferTech Match Result NMS](https://docs.transfertech.cn/tf_docs/en/atom/1.3.1.1/operator/grasp/filter_and_sort/PoseListNMS.html) | 模板按位姿变换后体素 IoU | 体素占用思路（去重，不是去压叠） |
| [PCL VoxelGrid tutorial](https://pcl.readthedocs.io/projects/tutorials/en/pcl-1.13.0/voxel_grid.html) | `pcl::VoxelGrid` 下采样 | 变换后点云抽稀 |
| [ritwikrohan/CloudGrasp](https://github.com/ritwikrohan/CloudGrasp) | PCL 平面 + 欧氏聚类抓取流水线 | 场景分割参考（输入不同：这里已有实例位姿） |

没有现成的开源库直接实现 Mech-Vision 的「去压叠」。因此按上述文档，用 PCL（`transformPointCloud`、`VoxelGrid`、`getMinMax3D`、`compute3DCentroid`、`loadPCDFile`）实现了同等输入输出。

## 算法

输入：

- 模板点云（模型坐标系，一种工件可共用一份；不同工件可各给一份）
- 每个检测的 4×4 配准位姿

对每个实例：`world = T_pose * template`。

### `projection_2d`（默认，面模板推荐）

1. 沿 `up_axis`（默认 +Z）做正交投影，格子边长 `pixel_size`（默认 2% 点云对角线，对应 Mech-Vision 2%）。
2. 每个格子记录该实例的最大高度。
3. 若其它实例在同一格子且更高（超过 `height_margin`），该格视为被压住。
4. `overlap_ratio = 被压住格数 / 本实例占用格数`。超过阈值（默认 0.30）则丢弃。

并排接触但互不在上方 → 比例约 0，两者都保留。  
完全叠在一起 → 下层比例接近 1，只留下层顶上的那个。  
两堆高度不同 → **每堆的顶层都会保留**（这是抓取里更合理的行为，不同于全局最高层裁切）。

### `bounding_box_3d`

把位姿对齐的 AABB 填实体素再走同一套「上方占用」判断。适合边缘模板 / 点云很稀的情况。

### `highest_layer`

在投影去压叠之外，再按平均高度裁切：只留距最高实例 `layer_thickness/2` 以内的目标（TransferTech PointsUpJudge）。

输出按 `mean_height` 从高到低排序，便于从上往下抓。

长度单位：**米**。`pixel_size = 0.0025` 即 2.5 mm。

## 构建

依赖：`libpcl-dev`（Ubuntu: `sudo apt-get install libpcl-dev`）。不装 PCL 时可用 `-DBUILD_STACK_FILTER=OFF`。

```bash
cd filterreg_rigid
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=g++
cmake --build . --target stack_filter -j

./stack_filter --self-test
./stack_filter --demo --out-dir .
```

`--demo` 会生成五块 100×100×50 mm 的盒子：A 被 C 压住、B 被 D 部分压住、E 单独在桌面上。期望保留 **C、D、E**。

## 用自己的模板和位姿

```bash
./stack_filter --template widget.pcd --poses poses.json --out-dir ./out \
    --method projection_2d --threshold 0.30 --pixel-size 0.0025
```

`poses.json` 示例见 `apps/stack_filter/sample_poses.json`。每位姿可以是：

- `translation` + `quaternion_wxyz`（wxyz）
- 或 16 个数的行优先 `matrix`

输出：

- `stack_filter_result.json` — 每个实例的压叠比例、是否保留
- `stack_filter_result.ply` — 绿色保留 / 红色丢弃
- `stack_filter_topdown.svg` — XY 俯视框

C++ 调用：

```cpp
poser::RegisteredInstance inst;
inst.id = "obj_0";
inst.model = template_cloud;       // pcl::PointCloud<pcl::PointXYZ>::Ptr
inst.pose = T_model_to_world;      // Eigen::Matrix4f
poser::StackedObjectFilter filter;
auto out = filter.Filter({inst, ...});
// out.kept_indices, out.instances[i].overlap_ratio
```

从 FilterReg 的 `poser::mat34` 转换：`Eigen::Matrix4f T = poser::to_eigen(kinematic.GetRigidTransform());`
