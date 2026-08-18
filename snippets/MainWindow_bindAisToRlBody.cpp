// 加载 STEP → AIS_Shape 后，绑定到 RL 环境模型，并在 OCCT 刷新里同步位姿。
//
// 1) 加入工程：AisRlBodyBinding.h / AisRlBodyBinding.cpp
// 2) 环境模型用 scene 里机器人之外的那一组（常见为 getModel(1)），不要绑到 Model 0。
// 3) 在已有 occtUpdate / 50 ms 定时器里调用 sync。

#include "AisRlBodyBinding.h"

Handle(AIS_Shape) aisWorkpiece;   // 成员，即 loadShape
rl::sg::Body* rlWorkpiece = nullptr;

void MainWindow::loadStepAsRlObstacle(const QString& path)
{
	removeLoadShape();

	auto loaded = ReadModel::loadStepModel(path.toLocal8Bit().constData());
	if (loaded.shape.IsNull())
		return;

	aisWorkpiece = ReadModel::makeDisplayShape(loaded);
	this->myOccView->getContext()->Display(aisWorkpiece, Standard_True);

	rl::sg::Model* env = nullptr;
	if (this->scene->getNumModels() > 1)
		env = this->scene->getModel(1);
	else
		env = this->scene->create();

	// 新建的碰撞体名字是 "occ_ais_body"。场景里原有的 "body" 用 findRlBodyByName 取。
	rlWorkpiece = bindAisShapeToRlBody(aisWorkpiece.get(), env);
}

// 从 OCC 删除 loadShape 时，同步删除 getModel(1) 上绑定的 RL Body，避免规划仍碰到旧工件。
void MainWindow::removeLoadShape()
{
	if (this->myOccView != nullptr && !aisWorkpiece.IsNull())
	{
		this->myOccView->getContext()->Remove(aisWorkpiece, Standard_True);
		this->myOccView->Redraw();
	}
	aisWorkpiece.Nullify();

	unbindAisShapeFromRlBody(rlWorkpiece);
	if (this->scene != nullptr && this->scene->getNumModels() > 1)
		unbindAisShapesFromRlModel(this->scene->getModel(1));
}

// 已有的场景刷新 / 50 ms 定时器：先同步，再读两边原点
void MainWindow::occtUpdate()
{
	AIS_Shape* loadShape = aisWorkpiece.get();
	syncAisPoseToRlBody(loadShape, rlWorkpiece);

	const rl::math::Vector3 aisP = aisWorldTranslation(loadShape);

	rl::sg::Body* namedBody = findRlBodyByName(this->scene->getModel(1), "body");
	const rl::math::Vector3 bodyP = rlBodyTranslation(namedBody);

	this->statusBar()->showMessage(
		QString("AIS xyz=(%1, %2, %3)    body xyz=(%4, %5, %6)")
			.arg(aisP.x(), 0, 'f', 4)
			.arg(aisP.y(), 0, 'f', 4)
			.arg(aisP.z(), 0, 'f', 4)
			.arg(bodyP.x(), 0, 'f', 4)
			.arg(bodyP.y(), 0, 'f', 4)
			.arg(bodyP.z(), 0, 'f', 4));
}
