# FilterReg Rigid (pt2pt / pt2pl) — Standalone Package

从 [bhsphd/FilterReg](https://github.com/bhsphd/FilterReg) 提取并整理的 **刚性点云配准** 最小实现，对应原仓库：

- `apps/rigid_pt2pt` — FilterReg point-to-point（GMM + Kabsch）
- `apps/rigid_pt2pl` — FilterReg point-to-plane（GMM + twist）

本目录可独立编译运行，**刚性配准核心不依赖 PCL / OpenCV / CUDA Toolkit**。
抓取堆叠过滤（`stack_filter`）是可选模块，需要系统 PCL（`libpcl-dev`）。

## 依赖

| 依赖 | 说明 |
|------|------|
| CMake ≥ 3.10 | 构建 |
| C++14 编译器 | 推荐 `g++` |
| glog | `sudo apt-get install libgoogle-glog-dev` |
| Eigen3 | 已内置 `external/eigen3` |
| nlohmann/json | 已内置 `external/nlohmann` |
| CUDA stubs | 已内置 `external/cuda_stub`（仅提供 `float3/float4` 等类型，不跑 GPU） |
| PCL 1.8+（可选） | 仅 `stack_filter` 需要：`sudo apt-get install libpcl-dev` |

## 构建与运行

```bash
cd filterreg_rigid
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=g++
cmake --build . -j

# point-to-point（Stanford bunny）
./rigid_pt2pt ./bunny.pcd

# point-to-plane
./rigid_pt2pl ./cloud_0.pcd ./cloud_1.pcd

# 堆叠过滤（需 PCL）：模板点云 + 配准位姿 → 保留未被压住的最上层目标
./stack_filter --self-test
./stack_filter --demo --out-dir .
# 详见 apps/stack_filter/README.md
```

运行结束后会写出 `matched_live.ply` / `matched_observation.ply`（替代原 PCL 可视化窗口）。

## 相对原仓库的改动

1. **去掉 PCL**：自研轻量 PCD 读写（`io/pcd_io.*`），支持 `ascii` / `binary` / `binary_compressed`。
2. **去掉可视化依赖**：`DebugVisualizer::DrawMatchedCloudPair` 改为导出 PLY。
3. **去掉 CUDA / OpenCV / imgproc / cloudproc / nn_search**：这两个 app 的 CPU 算法路径不需要它们。
4. **修复原仓库 bug**：`GMMPermutohedralUpdatedSigma::ComputeSigmaValue` 缺少 `return`，会导致未定义行为。

## 目录结构

见 [FILE_LIST.md](./FILE_LIST.md)。核心模块：

```
filterreg_rigid/
├── apps/rigid_pt2pt/          # pt2pt demo + bunny.pcd
├── apps/rigid_pt2pl/          # pt2pl demo + cloud_*.pcd
├── apps/stack_filter/         # 抓取去压叠 demo（PCL）
├── stack_filter/              # 模板点云 + 位姿 → 最上层未压叠目标
├── common/                    # FeatureMap / TensorBlob 等基础设施
├── geometry_utils/            # mat33/mat34、permutohedral
├── kinematic/rigid/           # 刚性运动学 + Kabsch / pt2pl assembler
├── corr_search/gmm/           # FilterReg GMM 对应搜索
├── io/                        # 无 PCL 的 PCD 读取
├── visualizer/                # PLY 导出
└── external/                  # eigen3 / nlohmann / cuda_stub
```

## 算法出处

Wei Gao and Russ Tedrake, *FilterReg: Robust and Efficient Probabilistic Point-Set Registration using Gaussian Filter and Twist Parameterization*, CVPR 2019.
