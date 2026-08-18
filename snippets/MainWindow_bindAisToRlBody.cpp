// 多模型：MainWindow 持有一份 AisRlBodyBinder。
// 删除走 unbind(选中的 AIS)；拖动走 sync(当前 AIS)，不要 syncAll。

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
