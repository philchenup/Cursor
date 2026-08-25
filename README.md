# FilterReg Rigid Standalone

独立可编译的 FilterReg 刚性配准实现（`rigid_pt2pt` / `rigid_pt2pl`），源码整理自 [bhsphd/FilterReg](https://github.com/bhsphd/FilterReg)，**无需配置 PCL**。

配准得到多个抓取目标后，可用 PCL 模块 [`filterreg_rigid/stack_filter`](filterreg_rigid/stack_filter) 做堆叠过滤。视角从 −Z 朝 +Z 看（Z 越小越靠上），只保留最上层且未被压住的目标。提供 2D 投影与 3D 包围盒两种方法。

详见 [`filterreg_rigid/README.md`](filterreg_rigid/README.md)。
