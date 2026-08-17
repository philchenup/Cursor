#ifndef AIS_RL_BODY_BINDING_H
#define AIS_RL_BODY_BINDING_H

#include <AIS_Shape.hxx>
#include <Standard_Real.hxx>
#include <gp_Trsf.hxx>

#include <rl/math/Transform.h>
#include <rl/sg/Body.h>
#include <rl/sg/Model.h>

/**
 * 把 OpenCASCADE 的 AIS_Shape 网格化后绑定为 Robotics Library 场景里的一个 Body。
 *
 * 网格写在 Body 局部坐标系（AIS_Shape::Shape()，不含显示位姿）。
 * OCCT 里拖动/SetLocalTransformation 之后，调用 syncAisPoseToRlBody()
 * 把当前显示位姿写到 body->setFrame()，碰撞模型会跟着动。
 *
 * @param ais           已 Display 的 AIS_Shape（STEP 读入后的显示对象）
 * @param sgModel       RL 场景中的环境模型（不要用机器人那一组 Model 0）
 * @param linearDeflection  网格线偏差；<=0 时按包围盒对角线的 1% 自动取
 * @return              新创建的 rl::sg::Body*，由 sgModel 持有寿命
 */
rl::sg::Body* bindAisShapeToRlBody(
	AIS_Shape* ais,
	rl::sg::Model* sgModel,
	Standard_Real linearDeflection = -1.0);

/// 把 AIS 当前世界位姿写到已绑定的 RL Body（拖动后 / 定时器里调用）。
void syncAisPoseToRlBody(const AIS_Shape* ais, rl::sg::Body* body);

/**
 * gp_Trsf → rl::math::Transform（只取刚体：单位四元数旋转 + 平移）。
 *
 * 不用 VectorialPart()：它含 scale，3x3 不是旋转，RL setFrame 会崩。
 * 平移非有限或 OCCT 异常时返回 Identity；旋转无法恢复时保留平移。
 * 不把异常抛给调用方。
 */
rl::math::Transform gpTrsfToRl(const gp_Trsf& trsf);

#endif
