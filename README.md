# Cursor

## ScaleAISShapeBy1000

将 OpenCASCADE 的 `AIS_Shape*` 缩小 1000 倍，并返回一个新的 `AIS_Shape*`。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
AIS_Shape* scaled = ScaleAISShapeBy1000(ais);

// 推荐用 Handle 接管返回值，避免泄漏
Handle(AIS_Shape) scaledHandle = ScaleAISShapeBy1000(ais);
```

## V-groove weld planning (3D)

`vgroove_weld_planning.py` converts the original MATLAB 2D V-groove weld planner to Python and extrudes the plan along the weld length for 3D display.

```bash
pip install -r requirements.txt
python3 vgroove_weld_planning.py
python3 vgroove_weld_planning.py --show
python3 vgroove_weld_planning.py --h 16 --beta 30 --g 1 --weld-length 40
```

Figures are written to `figures/vgroove_weld_3d_overview.png` and `figures/vgroove_weld_3d_detail.png`.
