# 点云 Surface Matching：可复现、效果最好的算法

本文结论针对工业视觉里的 **surface matching**：用 **模型点云**（CAD 采样或扫描模板）在 **场景点云** 中找回物体的 6D 位姿。这与「两片扫描互配准」或「从点云重建网格」不是同一件事。

## 结论

**在「只使用点云 / 深度、不训练、可公开复现」的约束下，效果最好的 surface matching 算法是：**

> **改进 Point Pair Feature（PPF）全局投票 + 点到面 ICP 精修**
>
> 代表工作：Drost et al., CVPR 2010 → Hinterstoisser et al., ECCV 2016 → **Vidal et al., Sensors 2018（Vidal-Sensors18）**
>
> 最容易复现的工程入口：**OpenCV `ppf_match_3d`**（Drost PPF + Birdal 多分辨率 ICP）
>
> 本仓库提供同一管线的纯 NumPy 参考实现，见 `surface_matching/`。

原因可以压成三句话：

1. **BOP 官方评测里，纯深度 / 点云方法长期由 PPF 家族占据第一档。** Vidal-Sensors18 在 BOP 2018 平均召回 **74.6%**（15 个方法第一），BOP 2019 核心七数据集 Average Recall **0.569**（当时总榜第一，也是深度-only 第一），明显高于原始 Drost-CVPR10-3D-Only 的 0.487。
2. **它天然就是 surface matching，而不是扫到扫配准。** 离线用模型点对建哈希表，在线对场景点对投票，输出多个 6D 假设，再 ICP。不需要 RGB、不需要标注、不需要训练，CAD 采样即可。
3. **可复现性最好。** 论文、BOP 提交、OpenCV 官方模块、LINEMOD / T-LESS / ITODD 都是公开的；深度学习配准（GeoTransformer 等）在 3DMatch 上分数更高，但解决的是另一类问题，而且依赖训练数据和权重。

如果任务其实是 **两片点云互配准**（室内重建、SLAM），不要用 PPF，改用下面「相邻任务」一节的 GeoTransformer / MAC。

## 问题定义

| 任务 | 输入 | 输出 | 典型场景 |
| --- | --- | --- | --- |
| **Surface matching（本文）** | 已知模型点云 + 场景点云 | 模型在场景中的 \(R,t\)，可多实例 | 工业抓取、CAD 定位、检测 |
| Pairwise registration | 两片部分重叠扫描 | 相对位姿 | 三维重建、SLAM |
| Surface reconstruction | 一张点云 | 网格 / 隐式曲面 | 扫描后建模 |

Surface matching 的难点是：场景里有杂物、遮挡、只看到模型的一部分、没有可靠初值。ICP 单独做不到，必须先全局粗匹配。

## 算法主干

PPF 描述一对带法向的点 \((p_1,n_1),(p_2,n_2)\)：

\[
F = \big(\|d\|,\; \angle(n_1,d),\; \angle(n_2,d),\; \angle(n_1,n_2)\big),\quad d=p_2-p_1
\]

1. **离线（模型）**：采样模型点与法向，枚举点对，量化 \(F\) 后写入哈希表，并记下把参考点法向对齐到 \(x\) 轴后的绕轴角 \(\alpha_m\)。
2. **在线（场景）**：对场景参考点同样算点对，查表，在 \((\text{模型参考点},\;\alpha)\) 上霍夫投票。
3. **假设生成**：票数峰值还原为 \(T_{\text{model}\to\text{scene}}=T_s^{-1}\,R_x(\alpha)\,T_m\)。
4. **聚类**：相近位姿合并（Hinterstoisser / Vidal 的凝聚聚类）。
5. **精修**：点到面 ICP（OpenCV 用的是 Birdal 的多分辨率 ICP）。
6. **Vidal 额外步骤**：法向聚类预处理、邻域 bin 查表、视点相关重打分、场景一致性检验、NMS。

这些步骤解决的是 ICP 做不到的事：无初值、部分可见、杂波。ICP 只负责最后几个毫米 / 几度。

## 公开数字（越可核对越好）

### BOP：模型到场景，只用深度

| 方法 | 模态 | BOP 2018 平均召回 | BOP 2019 AR | 备注 |
| --- | --- | --- | --- | --- |
| **Vidal-Sensors18** | D | **74.6%** | **0.569** | 改进 PPF，当时总榜第一 |
| Drost-CVPR10-3D-Edges | D | 71.7%（edge 变体） | 0.500 | 原作者加边缘 |
| Drost-CVPR10-3D-Only | D | 68.1% | 0.487 | 经典 PPF |
| Drost-CVPR10-3D-Only-Faster | D | — | 0.454 | 更快、略掉点 |
| ZTE_PPF（2022） | D | — | T-LESS 0.374 / ITODD 0.470 | 优化版 Drost |
| XYZ-SurfaceMatching（2022） | D | — | ITODD 0.471 | 工业实现 |
| 多视角 PPF（Meas. Sci. Technol. 2025） | D | — | ITODD **0.696** | 传统方法新高，**未见公开代码** |

来源：

- BOP ECCV 2018 论文，[arXiv:1808.08319](https://arxiv.org/abs/1808.08319)
- [BOP Challenge 2019 结果](https://bop.felk.cvut.cz/media/bop_challenge_2019_results.pdf)
- [Vidal-Sensors18 方法页](https://bop.felk.cvut.cz/method_info/45/)
- Vidal 原文：Sensors 2018, 18(8):2678，[doi:10.3390/s18082678](https://doi.org/10.3390/s18082678)

BOP 2019 上 Vidal 分数据集：LM-O 0.582，T-LESS 0.538，TUD-L 0.876，IC-BIN 0.393，ITODD 0.435，HB 0.706，YCB-V 0.450。

**注意：** 2024 年以后 RGB / RGB-D 学习方法（FoundationPose、GPose、HccePose 等）在 BOP 上已经明显高于纯点云 PPF。它们 **不是**「基于点云」的 surface matching，也通常不能只靠 CAD 点云、零训练就部署。

### 若任务其实是点云配准（3DMatch）

| 方法 | 3DMatch RR | 3DLoMatch RR | 复现 |
| --- | --- | --- | --- |
| **GeoTransformer** (CVPR 2022) | 92.5% | 74.2% | [官方代码 + 权重](https://github.com/qinzheng93/GeoTransformer) |
| MAC + 学习描述子 (TPAMI 2024) | 95.7% | 78.9% | [官方代码](https://github.com/zhangxy0517/3D-Registration-with-Maximal-Cliques) |
| PEAL (CVPR 2023) | 94.4% | ~79% | 需要重叠先验 |
| CoFF (ISPRS 2025) | 95.9% | 81.6% | 需要 RGB，不是纯点云 |
| TEASER++ + FPFH | 低于上表 | 低于上表 | [官方 C++/Python](https://github.com/MIT-SPARK/TEASER-plusplus)，可证明鲁棒 |

配准 SOTA 不能直接当 surface matching 用：它假设两片云大致是同一场景的两部分，而不是「小模型埋在杂乱场景里」。

## 可复现实现怎么选

按「能独立跑通、参数公开、结果能对上公开基准」排序：

1. **OpenCV `cv::ppf_match_3d::PPF3DDetector` + `ICP`**
   - 官方维护，C++，输入 `Nx6`（XYZ + 法向）
   - 文档：[Surface Matching](https://docs.opencv.org/4.x/d9/d25/group__surface__matching.html)
   - 这是工业开源落地里复现成本最低的一条路
   - 精度对应 Drost + 改进 ICP，一般低于完整 Vidal 管线
2. **本仓库 `surface_matching`**
   - Drost 投票 + Hinterstoisser/Vidal 邻域 bin + 位姿聚类 + 点到面 ICP
   - 依赖只有 NumPy，测试是合成位姿恢复（可重复、无外部数据）
   - 用于读懂算法和接到 OpenCASCADE 采样点云，不是 BOP 完整复现
3. **hengguan/ppf_matcher**
   - 宣称在 OpenCV 上吸收了 Vidal + Hinterstoisser，并用 LINEMOD 测试
   - 个人仓库，维护停在 2022，适合对照，不适合当唯一依据
4. **qq456cvb/GeometricProcessing3D**
   - GPU PPF，引用 Drost + Vidal
   - 需要 CUDA，评测完整性一般
5. **Vidal 原作者实现**
   - 只作为 BOP 提交存在，**没有官方 GitHub**
   - 「完整 Vidal」目前不能无歧义地复现到论文数字

因此：**要「效果最好且能复现」——算法选 Vidal 管线，工程选 OpenCV PPF + ICP，研究/对接 CAD 用本仓库参考实现。**

## 什么时候不要用 PPF

- **两片扫描拼起来**（重叠 30%–70%，尺度接近）：GeoTransformer 或 FPFH + TEASER++ + ICP。
- **有 RGB 且可以训练 / 用现成权重**：BOP 现在的第一档是 RGB-D 学习方法，不是 PPF。
- **平面、高对称、几乎没有曲率变化的零件**：PPF 投票会糊。加边缘点、曲率加权，或上多视角 / 接触约束。
- **只要局部精修、已有初值**：直接点到面 ICP，不要跑全局投票。

## 推荐部署顺序

1. 从 CAD（OpenCASCADE `BRepMesh` / 表面采样）得到带法向的模型点云，单位与扫描一致。
2. 法向朝外或朝向传感器，模型与场景取向一致。
3. OpenCV `PPF3DDetector(relativeSamplingStep=0.03~0.05, relativeDistanceStep=0.05)` 训练模型。
4. `match()` 得到假设，再用 `ppf_match_3d::ICP` 精修。
5. 用模型直径的 10% 作为内点阈值做一次点到面误差验收。
6. 若杂波多、平面多：加上 Hinterstoisser 的「同一量化特征只投一票」、边缘/曲率采样、Vidal 的邻域 bin 与假设重打分。

本仓库的 `surface_matching.match_surface()` 按同一顺序实现，便于先在合成数据上确认几何，再换 OpenCV 要速度。
