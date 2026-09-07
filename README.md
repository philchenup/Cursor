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

## V-groove weld planning (Open3D)

`vgroove_weld_planning.py` converts the original MATLAB 2D V-groove weld planner to Python and displays the plan with **Open3D** in a right-handed frame:

- **Y** (green): weld / seam direction
- **Z** (blue): height / plate thickness
- **X** (red): groove width, from the right-hand rule `X = Y × Z`

```bash
pip install -r requirements.txt
python3 vgroove_weld_planning.py
python3 vgroove_weld_planning.py --show
python3 vgroove_weld_planning.py --gui
python3 vgroove_weld_planning.py --h 16 --beta 30 --g 1 --weld-length 40 --view end --show

# transparency (0 = hidden, 1 = opaque). Lower these to see torch arrows.
python3 vgroove_weld_planning.py --bead-alpha 0.2 --plate-alpha 0 --layer-alpha 0
# restore solid workpiece / full-length beads if needed:
python3 vgroove_weld_planning.py --plate-alpha 0.25 --bead-depth 0 --bead-alpha 0.15
```

Screenshots: `figures/vgroove_open3d_perspective.png`, `figures/vgroove_open3d_endview.png`.
