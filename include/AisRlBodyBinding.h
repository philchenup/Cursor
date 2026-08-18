#ifndef AIS_RL_BODY_BINDING_H
#define AIS_RL_BODY_BINDING_H

#include <AIS_Shape.hxx>
#include <Standard_Real.hxx>
#include <gp_Trsf.hxx>
#include <string>
#include <vector>

#include <rl/math/Transform.h>
#include <rl/math/Vector.h>
#include <rl/sg/Body.h>
#include <rl/sg/Model.h>

/**
 * AIS_Shape ↔ rl::sg::Body 一对一绑定（不含 OCC 选中 / 拖动 / Remove）。
 *
 * 网格写在 Body 局部坐标系（AIS_Shape::Shape()，不含显示位姿）。
 * 每个 bind 生成独立 Body，名称 occ_ais_body_<序号>，互不影响。
 * 拖动后只对这一对调用 syncAisPoseToRlBody(loadShape, rlWorkpiece)。
 *
 * 选中、拖动、从场景删除放在 MainWindow，见 snippets/MainWindow_bindAisToRlBody.cpp。
 */

/// MainWindow 持有的一对一表：A_step ↔ A_body、B_step ↔ B_body。
struct AisRlPair
{
	Handle(AIS_Shape) ais;
	rl::sg::Body* body = nullptr;
};

/// bind 生成的 Body 名称前缀。场景 XML 里原有的环境 Body 不要用这个前缀。
const char* occAisBodyNamePrefix();

bool isOccAisBoundBody(const rl::sg::Body* body);

/**
 * 把 AIS_Shape 网格化后创建一颗独立的 RL Body。
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

/**
 * 只把这一对 AIS 的世界位姿写到对应 Body。其它 Body 的 frame 不变。
 * ais 或 body 为空时返回 Identity。
 */
rl::math::Transform syncAisPoseToRlBody(const AIS_Shape* ais, rl::sg::Body* body);

/**
 * 删除这一颗 bind 创建的 Body（FCL 注销并从 sgModel 列表移除）。
 * body 置为 nullptr。不要对场景 XML 里原有的环境 Body 调用。
 */
void unbindAisShapeFromRlBody(rl::sg::Body*& body);

/**
 * 删除 sgModel 上所有 occ_ais_body_* 。不碰场景 XML 里原有的环境 Body。
 * 单模型删除请用 unbindAisShapeFromRlBody，不要调用这个。
 */
void unbindAisShapesFromRlModel(rl::sg::Model* sgModel);

AisRlPair* findAisRlPair(std::vector<AisRlPair>& pairs, const AIS_Shape* ais);
const AisRlPair* findAisRlPair(const std::vector<AisRlPair>& pairs, const AIS_Shape* ais);

rl::sg::Body* findRlBodyForAis(const std::vector<AisRlPair>& pairs, const AIS_Shape* ais);

/// 从表里解绑并移除这一对。没找到则什么也不做。OCC Remove 仍由 MainWindow 做。
void eraseAisRlPair(std::vector<AisRlPair>& pairs, const AIS_Shape* ais);

/// 从 AIS 成员 LocalTransformation 按值拷贝。不要写 const gp_Trsf& t = ais->Transformation()。
gp_Trsf copyAisLocalTrsf(const AIS_Shape* ais);

/// AIS 在世界坐标系中的位姿：父节点链 × LocalTransformation，按值返回。
gp_Trsf copyAisWorldTrsf(const AIS_Shape* ais);

/// 同 copyAisWorldTrsf。网格已含 Shape().Location()，此处不再乘一次。
gp_Trsf GetAisShapeWorldTrsf(const AIS_Shape* ais);

rl::math::Transform GetAisShapeWorldRl(const AIS_Shape* ais);

/// AIS 显示坐标系原点的世界坐标。
rl::math::Vector3 aisWorldTranslation(const AIS_Shape* ais);

/// RL Body 当前 frame 的平移。
rl::math::Vector3 rlBodyTranslation(const rl::sg::Body* body);

/// 在 sgModel 里按名字找 Body；找不到返回 nullptr。
rl::sg::Body* findRlBodyByName(rl::sg::Model* sgModel, const std::string& name);

/// gp_Trsf 是否可用来做位姿：比例有限且非 0，Form 合法，原点变换结果有限。
bool isValidGpTrsf(const gp_Trsf& trsf);

/// gp_Trsf → rl::math::Transform。用 GetMat4 取 4×4；无效时返回 Identity。
rl::math::Transform gpTrsfToRl(const gp_Trsf& trsf);

#endif
