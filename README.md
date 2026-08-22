# FilterReg Rigid Standalone

独立可编译的 FilterReg 刚性配准实现（`rigid_pt2pt` / `rigid_pt2pl` / `robust_align`），源码整理自 [bhsphd/FilterReg](https://github.com/bhsphd/FilterReg)，**无需配置 PCL**。

室内平面场景下 P2P/P2L 对不齐、GICP 对 `maxCorrespondenceDistance` 过敏的原因见 [`filterreg_rigid/docs/icp_gicp_gap_analysis.md`](filterreg_rigid/docs/icp_gicp_gap_analysis.md)。用法见 [`filterreg_rigid/README.md`](filterreg_rigid/README.md)。
