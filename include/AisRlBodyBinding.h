#ifndef AIS_RL_BODY_BINDING_H
#define AIS_RL_BODY_BINDING_H

#include <AIS_InteractiveObject.hxx>
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

/**
 * 从 RL 场景删除 bindAisShapeToRlBody 创建的 Body。
 * delete 会走 Body 析构：注销 FCL 碰撞对象并从 sgModel 列表移除。
 * body 置为 nullptr，避免后续 sync / 规划继续用悬空指针。
 */
void unbindAisShapeFromRlBody(rl::sg::Body*& body);

/**
 * 删除 sgModel 上所有名称以 "occ_ais_body" 开头的 Body。
 * 不碰场景 XML 里原有的环境 Body。
 */
void unbindAisShapesFromRlModel(rl::sg::Model* sgModel);

/// 底层同步：把 AIS 世界位姿写到指定 Body。多模型请用 AisRlBodyBinder::sync / syncAll。
void syncAisPoseToRlBody(const AIS_Shape* ais, rl::sg::Body* body);

/**
 * 多模型 AIS↔RL 对照表。MainWindow 持有一份。
 * 拖动时用 sync(当前 AIS)，删除时用 unbind(选中 AIS)。不要对拖动调用 syncAll。
 */
class AisRlBodyBinder
{
public:
	void setModel(rl::sg::Model* sgModel) { m_model = sgModel; }
	rl::sg::Model* model() const { return m_model; }

	rl::sg::Body* bind(AIS_Shape* ais, Standard_Real linearDeflection = -1.0);
	rl::sg::Body* bind(AIS_Shape* ais, rl::sg::Model* sgModel, Standard_Real linearDeflection = -1.0);

	void unbind(AIS_Shape* ais);
	void unbind(const Handle(AIS_InteractiveObject)& obj);
	void unbindAll();

	/// 只同步这一个 AIS 对应的 Body。拖动时必须用这个，不要 syncAll。
	rl::math::Transform sync(const AIS_Shape* ais);
	/// 同步全部绑定；AIS 已失效的条目会删掉对应 Body。
	void syncAll();

	rl::sg::Body* bodyOf(const AIS_Shape* ais) const;
	std::size_t size() const { return m_entries.size(); }

private:
	struct Entry
	{
		Handle(AIS_Shape) ais;
		rl::sg::Body* body = nullptr;
	};

	std::vector<Entry>::iterator findEntry(const AIS_Shape* ais);
	std::vector<Entry>::const_iterator findEntry(const AIS_Shape* ais) const;

	std::vector<Entry> m_entries;
	rl::sg::Model* m_model = nullptr;
};

/// 从 AIS 成员 LocalTransformation 按值拷贝。不要写 const gp_Trsf& t = ais->Transformation()。
gp_Trsf copyAisLocalTrsf(const AIS_Shape* ais);

/// AIS 在世界坐标系中的位姿：父节点链 × LocalTransformation，按值返回。
gp_Trsf copyAisWorldTrsf(const AIS_Shape* ais);

/// AIS 显示坐标系原点的世界坐标（Transformation 作用在 (0,0,0)）。
rl::math::Vector3 aisWorldTranslation(const AIS_Shape* ais);

/// RL Body 当前 frame 的平移（getFrame().translation() 的拷贝）。
rl::math::Vector3 rlBodyTranslation(const rl::sg::Body* body);

/// 在 sgModel 里按名字找 Body；找不到返回 nullptr。
rl::sg::Body* findRlBodyByName(rl::sg::Model* sgModel, const std::string& name);

/// gp_Trsf 是否可用来做位姿：比例有限且非 0，Form 合法，原点变换结果有限。
/// 引用本身无法判断悬空，悬空引用仍是未定义行为。
bool isValidGpTrsf(const gp_Trsf& trsf);

/// gp_Trsf → rl::math::Transform。用 GetMat4 取 4×4；无效时返回 Identity。
rl::math::Transform gpTrsfToRl(const gp_Trsf& trsf);

#endif
