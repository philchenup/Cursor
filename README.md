# 点云算法性能对比（PCL / Open3D / 海康 VM3D）

对本仓库 `benchmark/` 目录运行统一数据集上的算法耗时测试，并按类汇总。

## 快速开始

```bash
# 依赖: Python3 + open3d, libpcl-dev, cmake, g++, nlohmann-json3-dev
pip3 install open3d numpy
cd benchmark
python3 scripts/generate_data.py
python3 scripts/bench_open3d.py
cmake -S . -B build && cmake --build build -j
./build/bench_pcl data results/pcl_timings.json
python3 scripts/make_report.py
```

结果见 [`REPORT.md`](REPORT.md) 与 `benchmark/results/`。

## 说明

- 同一合成点云（默认 100,000 点）对比处理耗时。
- 某库不存在的算法不输出耗时。
- 海康 VisionMaster 3D 为 Windows 商业软件，本环境无法实测耗时，仅根据[官方模块文档](https://pinfo.hikrobotics.com/hkws/unzip/20240919112534_17943_doc/)标注能力是否存在。
