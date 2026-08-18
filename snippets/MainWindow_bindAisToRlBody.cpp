// 多模型：MainWindow 持有一份 AisRlBodyBinder。
// 绑定/解绑按 AIS 对象查表；occtUpdate 只调 syncAll()，不必再传 (ais, body)。

#include "AisRlBodyBinding.h"

AisRlBodyBinder aisRl;   // 成员

rl::sg::Model* MainWindow::environmentModel()
{
	if (this->scene == nullptr)
		return nullptr;
	if (this->scene->getNumModels() > 1)
		return this->scene->getModel(1);
	return this->scene->create();
}

Handle(AIS_Shape) MainWindow::addStepAsRlObstacle(const QString& path)
{
	auto loaded = ReadModel::loadStepModel(path.toLocal8Bit().constData());
	if (loaded.shape.IsNull())
		return Handle(AIS_Shape)();

	Handle(AIS_Shape) ais = ReadModel::makeDisplayShape(loaded);
	this->myOccView->getContext()->Display(ais, Standard_True);

	aisRl.bind(ais.get(), environmentModel());
	return ais;
}

// 从 OCC 删除某一个模型时，同步删掉对应 RL Body
void MainWindow::removeAisObstacle(const Handle(AIS_Shape)& ais)
{
	if (ais.IsNull())
		return;

	aisRl.unbind(ais.get());

	if (this->myOccView != nullptr)
	{
		this->myOccView->getContext()->Remove(ais, Standard_True);
		this->myOccView->Redraw();
	}
}

void MainWindow::removeAllAisObstacles()
{
	aisRl.unbindAll();
	// 如还要清 OCC 显示，对每个 AIS 再 Remove；对照表已不再持有 Handle。
}

// 拖动 / 50 ms 定时器：一次同步全部绑定，不必再写 syncAisPoseToRlBody(ais, body)
void MainWindow::occtUpdate()
{
	aisRl.syncAll();
	// 只刷新某一个：aisRl.sync(ais.get());
	// ... 原来的机器人 link 刷新
}
