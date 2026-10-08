# 六轴机械臂的视觉实时避障

每个控制周期用深度图更新机械臂周围的符号距离场，再把六轴臂的连杆包络推离障碍，同时把末端送向笛卡尔目标。

## 控制周期

视觉更新和关节伺服分开。距离场按相机帧率重建，伺服周期只查询已经算好的场。

1. 用内参把深度图反投影成基座坐标系点云，再用当前关节角上的连杆球去掉机械臂本体。
2. 点云写入局部占据栅格，做一次可分离的欧氏距离变换，得到符号距离：障碍外为正，内部为负，梯度指向远离障碍的方向。
3. 末端生成朝向目标的吸引速度，速度大小不超过设定上限。
4. 若末端间隙小于影响距离，就沿距离梯度削掉过快的靠近分量。剩余间隙会在一个制动时间里被刹停；已经小于最小间隙时，速度改为推离障碍。吸引速度几乎正对障碍、切向分量消失时，补一个朝目标一侧的切向速度，避免停在障碍正前方。
5. 用阻尼最小二乘把末端速度映射成六轴关节速度。
6. 对每个连杆控制球重复速度阻尼：间隙变化率不得低于 `(最小间隙 - 当前间隙) / 制动时间`。
7. 靠近关节限位时加排斥速度，并在积分后夹紧到限位内。
8. 向前积分一步。如果这一步会把任一控制球的间隙压到最小间隙以下，就把步长减半再试。

连杆用一串球近似：每个连杆在关节原点以及连杆长度的 1/3、2/3 处各放一个球。球的位置雅可比和正运动学用的是同一套经典 DH。

默认臂不是某一台商业机器人的标定参数，只保留常见六轴结构：腰、肩、肘和三轴手腕。长度单位是米，角度是弧度。

| 关节 | a (m) | α | d (m) |
| --- | --- | --- | --- |
| 1 | 0 | π/2 | 0.30 |
| 2 | 0.40 | 0 | 0 |
| 3 | 0.30 | 0 | 0 |
| 4 | 0 | π/2 | 0.10 |
| 5 | 0 | −π/2 | 0.10 |
| 6 | 0 | 0 | 0.05 |

零位末端位置是 `(0.70, -0.15, 0.20)`。

## 构建与验证

```bash
CXX=g++ cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
./build/avoidance_demo
```

`avoidance_tests` 覆盖正运动学、雅可比、距离变换、深度反投影、本体剔除、自由空间到达、障碍绕行和关节限位。障碍场景里，关掉避障的直线跟踪会穿过球体；打开避障后末端会偏离直线，几何间隙保持为正，并到达目标。`avoidance_demo` 用虚拟相机渲染同一个球体，走完整的深度图到关节指令链路。

## 调用

```cpp
#include "arm_avoidance/ReactiveAvoidance.h"

arm_avoidance::SixAxisArm arm = arm_avoidance::SixAxisArm::DefaultIndustrial();
arm_avoidance::EsdfGrid grid({-0.2, -0.8, -0.3}, 0.02, 80, 80, 60);
arm_avoidance::VisionAvoidancePipeline pipeline(arm, grid);

arm_avoidance::CameraModel camera;
// 填写 fx、fy、cx、cy、分辨率，以及相机到基座的刚体变换
pipeline.setCamera(camera);

pipeline.updateFromDepth(depth, width, height, state.q);
arm_avoidance::AvoidanceResult result = pipeline.step(state);
// result.q_command 是下一拍的关节位置
```

栅格外的点视为未知，不参与约束。距离场只在写入的体素范围内可信。

## ScaleAISShapeBy1000

仓库里保留了把 OpenCASCADE `AIS_Shape` 缩小 1000 倍的函数。它依赖 OpenCASCADE 头文件，不参与上面的避障构建。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
Handle(AIS_Shape) scaled = ScaleAISShapeBy1000(ais);
```
