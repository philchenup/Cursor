# FilterReg Rigid Standalone

独立可编译的 FilterReg 刚性配准实现（`rigid_pt2pt` / `rigid_pt2pl`），源码整理自 [bhsphd/FilterReg](https://github.com/bhsphd/FilterReg)，**无需配置 PCL**。

详见 [`filterreg_rigid/README.md`](filterreg_rigid/README.md)。

轨迹规划：[`trajectory_planning/`](trajectory_planning/) 更新了 rlPlanDemo `Thread`，以目标点法兰位姿 `Eigen::Affine3f` 为输入做 IK + RRT。
