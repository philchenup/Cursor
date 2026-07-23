# Cursor

## ReverseEdgeByY

对 `const TopoDS_Edge`：若起点 Y 小于终点 Y，则首尾反转该边。

### 用法

```cpp
#include "ReverseEdgeByY.h"

TopoDS_Edge edge = /* ... */;
TopoDS_Edge oriented = ReverseEdgeIfStartYLessThanEndY(edge);
```

### 依赖

Open CASCADE Technology (OCCT)：`TopoDS`、`TopExp`、`BRep_Tool`。
