// 加载 STEP → AIS_Shape 后，绑定到 RL 环境模型，并在 OCCT 刷新里同步位姿。
//
// 1) 加入工程：AisRlBodyBinding.h / AisRlBodyBinding.cpp
// 2) 环境模型用 scene 里机器人之外的那一组（常见为 getModel(1)），不要绑到 Model 0。
// 3) 在已有 occtUpdate / 50 ms 定时器里调用 sync。

#include "AisRlBodyBinding.h"

Handle(AIS_Shape) aisWorkpiece;   // 成员
rl::sg::Body* rlWorkpiece = nullptr;

void MainWindow::loadStepAsRlObstacle(const QString& path)
{
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

	rlWorkpiece = bindAisShapeToRlBody(aisWorkpiece.get(), env);
}

// 已有的场景刷新里加一行（拖动 AIS 后 RL 碰撞体跟着动）
void MainWindow::occtUpdate()
{
	syncAisPoseToRlBody(aisWorkpiece.get(), rlWorkpiece);
	// ... 原来的机器人 link 刷新
}
