# FilterReg (C++ port of neka-nat/probreg)

只保留 [probreg](https://github.com/neka-nat/probreg) 中的 **Rigid FilterReg**（`pt2pt` / `pt2pl`）。
不包含 CPD、GMMReg、SVR、GMMTree、BCPD 或可变形 FilterReg。

算法对应 `probreg/filterreg.py` 的 `RigidFilterReg`：

1. **E-step**：permutohedral lattice 高斯滤波估计对应（`m0/m1/m2/nx`）
2. **M-step**：`pt2pt` 用加权 Kabsch，`pt2pl` 用 twist 线性化

论文：Gao & Tedrake, *FilterReg*, CVPR 2019.

## 文件

```
filterreg/
  filterreg.h / filterreg.cpp     # Rigid FilterReg
  permutohedral.h / .cpp          # 高维高斯滤波
  pcd_io.h / pcd_io.cpp           # 读 PCD、写 PLY
  main.cpp                        # demo + 合成测试
  data/                           # bunny / cloud_0 / cloud_1
```

依赖：CMake ≥ 3.10、C++14、Eigen3（`libeigen3-dev`）。

## 构建与运行

```bash
cd filterreg
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=g++
cmake --build build -j

./build/filterreg_demo --test
./build/filterreg_demo pt2pt ./build/bunny.pcd
./build/filterreg_demo pt2pl ./build/cloud_0.pcd ./build/cloud_1.pcd
```

配准结果写出 `matched_source.ply` / `matched_target.ply`。
