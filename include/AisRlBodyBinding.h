#ifndef AIS_RL_BODY_BINDING_H
#define AIS_RL_BODY_BINDING_H

#include <AIS_Shape.hxx>
#include <Standard_Real.hxx>
#include <gp_Trsf.hxx>
#include <string>

#include <rl/math/Transform.h>
#include <rl/math/Vector.h>
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

/// 从 AIS 成员 LocalTransformation 按值拷贝。不要写 const gp_Trsf& t = ais->Transformation()。
gp_Trsf copyAisLocalTrsf(const AIS_Shape* ais);

/// AIS 显示坐标系原点的世界坐标（Transformation 作用在 (0,0,0)）。
rl::math::Vector3 aisWorldTranslation(const AIS_Shape* ais);

/// RL Body 当前 frame 的平移（getFrame().translation() 的拷贝）。
rl::math::Vector3 rlBodyTranslation(const rl::sg::Body* body);

/// 在 sgModel 里按名字找 Body；找不到返回 nullptr。
rl::sg::Body* findRlBodyByName(rl::sg::Model* sgModel, const std::string& name);

/// gp_Trsf 是否可用来做位姿：比例有限且非 0，Form 合法，原点变换结果有限。
/// 引用本身无法判断悬空，悬空引用仍是未定义行为。
bool isValidGpTrsf(const gp_Trsf& trsf);

/// gp_Trsf → rl::math::Transform。无效时返回 Identity。
rl::math::Transform gpTrsfToRl(const gp_Trsf& trsf);

#endif
