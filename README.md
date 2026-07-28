# STEP 模型 Ø8mm 孔识别方案

通过开源几何内核 **OpenCASCADE**（本仓库用 CadQuery 自带的 **OCP** Python 绑定）读取 STEP，遍历圆柱面，区分孔与凸台，并按直径筛选出 **8mm** 孔。

## 推荐技术栈

| 方案 | 库 | 说明 |
|------|-----|------|
| **本仓库采用** | [CadQuery](https://github.com/CadQuery/cadquery) + OCP | `pip install cadquery` 即可，内含 OpenCASCADE |
| 等价方案 | [pythonocc-core](https://github.com/tpaviot/pythonocc-core) | 需 conda：`conda install -c conda-forge pythonocc-core`，API 几乎相同 |

核心思路与 pythonOCC 官方示例一致：对每个面做 `BRepAdaptor_Surface`，判断 `GeomAbs_Cylinder`，再取 `Radius`。

## 算法流程

1. `STEPControl_Reader` 读入 STEP → B-Rep
2. 遍历所有 `FACE`，保留圆柱面
3. 用实体分类器区分「孔」与「凸台」：取圆柱面中部对应的轴线上一点，若在实体外则为孔腔，在实体内则为凸台
4. 将共轴、半径相近的圆柱面合并为同一个逻辑孔
5. 按目标直径过滤（默认 8mm，容差 ±0.05mm）

## 快速开始

```bash
pip install -r requirements.txt

# 生成带混合孔径的示例零件
python3 -m step_hole_finder.generate_sample

# 查找直径 8mm 的孔
python3 -m step_hole_finder step_hole_finder/samples/plate_with_holes.step

# JSON 输出 / 列出全部孔 / 放宽容差
python3 -m step_hole_finder your.step --json
python3 -m step_hole_finder your.step --diameter 0
python3 -m step_hole_finder your.step --diameter 8 --tolerance 0.1
```

作为库调用：

```python
from step_hole_finder import load_step, find_cylindrical_holes

shape = load_step("part.step")
holes = find_cylindrical_holes(shape, target_diameter_mm=8.0)
for h in holes:
    print(h.diameter_mm, h.axis_point, h.axis_direction, h.depth_mm)
```

## 示例结果预期

`generate_sample.py` 生成的板件含：2×Ø8 通孔、1×Ø6、1×Ø10，以及 1 个 Ø8 圆柱凸台。检测 Ø8 孔时应得到 **2** 个，凸台不计入。

```bash
python -m unittest discover -s step_hole_finder/tests -v
```

## 局限与注意

- 依赖 B-Rep 圆柱面；若 STEP 把孔离散成 BSpline/网格，需先用 CAD 导出精确几何，或改用拟合算法。
- 锥孔、沉头孔、螺纹孔需扩展圆锥面 / 特征识别逻辑。
- 单位按 STEP 中的几何单位处理；机械零件通常为毫米。
- 通孔深度为圆柱面轴向跨度估计，复杂特征可能需再结合相邻平面求交精化。
