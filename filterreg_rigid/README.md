# FilterReg Rigid (pt2pt / pt2pl) — Standalone Package

从 [bhsphd/FilterReg](https://github.com/bhsphd/FilterReg) 提取并整理的 **刚性点云配准** 最小实现，对应原仓库：

- `apps/rigid_pt2pt` — FilterReg point-to-point（GMM + Kabsch）
- `apps/rigid_pt2pl` — FilterReg point-to-plane（GMM + twist）
- `apps/robust_align` — small_gicp 风格的 ICP / P2L / GICP（体素金字塔、自适应 `max_corr_dist`、Cauchy 核）

本目录可独立编译运行，**不依赖 PCL / OpenCV / CUDA Toolkit**。

室内平面场景下 P2P/P2L 对不齐、GICP 对 `maxCorrespondenceDistance` 过敏的原因与用法见 [docs/icp_gicp_gap_analysis.md](docs/icp_gicp_gap_analysis.md)。算法对照 [koide3/small_gicp](https://github.com/koide3/small_gicp)。

## 依赖

| 依赖 | 说明 |
|------|------|
| CMake ≥ 3.10 | 构建 |
| C++14 编译器 | 推荐 `g++` |
| glog | `sudo apt-get install libgoogle-glog-dev` |
| Eigen3 | 已内置 `external/eigen3` |
| nlohmann/json | 已内置 `external/nlohmann` |
| CUDA stubs | 已内置 `external/cuda_stub`（仅提供 `float3/float4` 等类型，不跑 GPU） |

## 构建与运行

```bash
cd filterreg_rigid
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=g++
cmake --build . -j

# point-to-point（Stanford bunny）
./rigid_pt2pt ./bunny.pcd

# point-to-plane（带宽退火，避免平行面混叠）
./rigid_pt2pl ./cloud_0.pcd ./cloud_1.pcd

# small_gicp-style ICP / P2L / GICP（推荐用于室内平面）
./robust_align --demo
./robust_align ./cloud_1.pcd ./cloud_0.pcd --method gicp --voxel 0.03 --max-dist 0.4
```

运行结束后会写出 `matched_live.ply` / `matched_observation.ply`（替代原 PCL 可视化窗口）。`robust_align` 写出 `aligned_source.ply` / `aligned_target.ply`。

## 相对原仓库的改动

1. **去掉 PCL**：自研轻量 PCD 读写（`io/pcd_io.*`），支持 `ascii` / `binary` / `binary_compressed`。
2. **去掉可视化依赖**：`DebugVisualizer::DrawMatchedCloudPair` 改为导出 PLY。
3. **去掉 CUDA / OpenCV / imgproc / cloudproc / nn_search**：这两个 app 的 CPU 算法路径不需要它们。
4. **修复原仓库 bug**：`GMMPermutohedralUpdatedSigma::ComputeSigmaValue` 缺少 `return`，会导致未定义行为。
5. **P2L 间隙**：GMM 平均法向重新单位化，并对 \(\sigma\) 做粗到细退火，避免平行面被核函数混成“中间平面”。
6. **robust_align**：参考 small_gicp 的 ICP/P2L/GICP 因子，加上自适应对应距离、Cauchy 核与法向一致性剔除。

## 目录结构

见 [FILE_LIST.md](./FILE_LIST.md)。核心模块：

```
filterreg_rigid/
├── apps/rigid_pt2pt/          # pt2pt demo + bunny.pcd
├── apps/rigid_pt2pl/          # pt2pl demo + cloud_*.pcd
├── apps/robust_align/         # ICP / P2L / GICP CLI + --demo
├── registration/              # small_gicp-style NN ICP/GICP
├── common/                    # FeatureMap / TensorBlob 等基础设施
├── geometry_utils/            # mat33/mat34、permutohedral
├── kinematic/rigid/           # 刚性运动学 + Kabsch / pt2pl assembler
├── corr_search/gmm/           # FilterReg GMM 对应搜索
├── io/                        # 无 PCL 的 PCD 读取
├── visualizer/                # PLY 导出
├── docs/                      # P2P/P2L/GICP 间隙与 max_dist 分析
└── external/                  # eigen3 / nlohmann / cuda_stub
```

## 算法出处

Wei Gao and Russ Tedrake, *FilterReg: Robust and Efficient Probabilistic Point-Set Registration using Gaussian Filter and Twist Parameterization*, CVPR 2019.
