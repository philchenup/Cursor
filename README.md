# Cursor

## PlaceConfig JSON 读写

用 [nlohmann/json](https://github.com/nlohmann/json) 将 `PlaceConfig` 保存到 JSON 文件，再读回来。

`ArrayConfig` 是匿名嵌套结构体，没有独立类型名，不能用 `NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE`。`Joint` 可以用宏；`PlaceConfig` 需要手写 `to_json` / `from_json`。

```cpp
#include "PlaceConfigJson.h"

PlaceConfig cfg;
cfg.ArrayConfig.layerX = 2;
cfg.ArrayConfig.layerY = 3;
cfg.ArrayConfig.obj_length = 120.5;
cfg.PlaceJoint = {1.1, 1.2, 1.3, 1.4, 1.5, 1.6};

SavePlaceConfig("place_config.json", cfg);
PlaceConfig loaded = LoadPlaceConfig("place_config.json");
```

生成的 JSON 形如：

```json
{
    "ArrayConfig": {
        "layerX": 2,
        "layerY": 3,
        "obj_length": 120.5,
        "obj_width": 0.0,
        "obj_height": 0.0
    },
    "PassJoint": { "j1": 0.0, "j2": 0.0, "j3": 0.0, "j4": 0.0, "j5": 0.0, "j6": 0.0 },
    "PlaceJoint": { "j1": 1.1, "j2": 1.2, "j3": 1.3, "j4": 1.4, "j5": 1.5, "j6": 1.6 },
    "WaitJoint": { "j1": 0.0, "j2": 0.0, "j3": 0.0, "j4": 0.0, "j5": 0.0, "j6": 0.0 }
}
```

编译示例（仓库已自带 `third_party/nlohmann/json.hpp`）：

```bash
make test
```

## ScaleAISShapeBy1000

将 OpenCASCADE 的 `AIS_Shape*` 缩小 1000 倍，并返回一个新的 `AIS_Shape*`。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
AIS_Shape* scaled = ScaleAISShapeBy1000(ais);

// 推荐用 Handle 接管返回值，避免泄漏
Handle(AIS_Shape) scaledHandle = ScaleAISShapeBy1000(ais);
```
