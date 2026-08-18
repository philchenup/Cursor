// MainWindow：多 STEP 一对一绑定、拖动同步、选中删除。
// AisRlBodyBinding 只提供 bind / sync(ais, body) / unbind(body)，不含 OCC 选中与 Remove。
//
// MainWindow.h：
//   #include "AisRlBodyBinding.h"
//   std::vector<AisRlPair> m_aisRlPairs;
//   rl::sg::Model* environmentModel();
//   Handle(AIS_Shape) loadStepAsRlObstacle(const QString& path);
//   void onAisPoseChanged(AIS_Shape* loadShape);
//   void onAisDragFinished(AIS_Shape* loadShape);
//   void removeModel();
//
// 构造里：
//   connect(myOccView, &OccView::aisPoseChanged, this, &MainWindow::onAisPoseChanged);
//   connect(myOccView, &OccView::aisDragFinished, this, &MainWindow::onAisDragFinished);

#include "AisRlBodyBinding.h"

#include <vector>

#include <AIS_InteractiveObject.hxx>
#include <AIS_Point.hxx>
#include <AIS_Shape.hxx>
#include <AIS_Trihedron.hxx>
#include <AIS_ViewCube.hxx>

rl::sg::Model* MainWindow::environmentModel()
{
	if (this->scene == nullptr)
		return nullptr;
	if (this->scene->getNumModels() > 1)
		return this->scene->getModel(1);
	return this->scene->create();
}

Handle(AIS_Shape) MainWindow::loadStepAsRlObstacle(const QString& path)
{
	auto loaded = ReadModel::loadStepModel(path.toLocal8Bit().constData());
	if (loaded.shape.IsNull())
		return Handle(AIS_Shape)();

	Handle(AIS_Shape) loadShape = ReadModel::makeDisplayShape(loaded);
	this->myOccView->getContext()->Display(loadShape, Standard_True);

	// A_step → A_body；再 load 一次则 B_step → B_body。各自身份独立。
	rl::sg::Body* rlWorkpiece = bindAisShapeToRlBody(
		loadShape.get(), environmentModel());
	if (rlWorkpiece == nullptr)
		return loadShape;

	AisRlPair pair;
	pair.ais = loadShape;
	pair.body = rlWorkpiece;
	m_aisRlPairs.push_back(pair);
	return loadShape;
}

void MainWindow::onAisPoseChanged(AIS_Shape* loadShape)
{
	rl::sg::Body* rlWorkpiece = findRlBodyForAis(m_aisRlPairs, loadShape);
	if (rlWorkpiece == nullptr)
		return;
	// 只更新这一对，其它 Body 的 frame 不变。
	syncAisPoseToRlBody(loadShape, rlWorkpiece);
}

void MainWindow::onAisDragFinished(AIS_Shape* loadShape)
{
	if (loadShape == nullptr)
		return;

	AisRlPair* pair = findAisRlPair(m_aisRlPairs, loadShape);
	if (pair == nullptr)
		return;

	// 松开后若几何已烘焙进 Shape() 且 LocalTransformation 复位，需要重网格这一颗 Body。
	unbindAisShapeFromRlBody(pair->body);
	pair->body = bindAisShapeToRlBody(loadShape, environmentModel());
}

void MainWindow::removeModel()
{
	std::vector<Handle(AIS_InteractiveObject)> selectedObjects;
	myOccView->getContext()->InitSelected();

	while (myOccView->getContext()->MoreSelected())
	{
		Handle(AIS_InteractiveObject) aAis = myOccView->getContext()->SelectedInteractive();
		selectedObjects.push_back(aAis);
		myOccView->getContext()->NextSelected();
	}

	for (auto aAis : selectedObjects)
	{
		if (aAis.IsNull())
			continue;
		if (aAis->IsKind(STANDARD_TYPE(AIS_ViewCube))
			|| aAis->IsKind(STANDARD_TYPE(AIS_Manipulator))
			|| aAis->IsKind(STANDARD_TYPE(AIS_Trihedron))
			|| aAis->IsKind(STANDARD_TYPE(AIS_Point)))
			continue;

		Handle(AIS_Shape) loadShape = Handle(AIS_Shape)::DownCast(aAis);
		if (!loadShape.IsNull())
			eraseAisRlPair(m_aisRlPairs, loadShape.get());

		myOccView->getContext()->Remove(aAis, false);
	}
	myOccView->getContext()->UpdateCurrentViewer();
}

void MainWindow::occtUpdate()
{
	// 不要对全部工件 sync：拖 A 时不得改 B 的 Body。
	// 工件位姿只在 onAisPoseChanged / onAisDragFinished 里按当前 AIS 更新。
	// 无 manipulator 信号时，可只同步当前选中的那一对：
	myOccView->getContext()->InitSelected();
	while (myOccView->getContext()->MoreSelected())
	{
		Handle(AIS_Shape) loadShape =
			Handle(AIS_Shape)::DownCast(myOccView->getContext()->SelectedInteractive());
		if (!loadShape.IsNull())
		{
			rl::sg::Body* rlWorkpiece = findRlBodyForAis(m_aisRlPairs, loadShape.get());
			syncAisPoseToRlBody(loadShape.get(), rlWorkpiece);
		}
		myOccView->getContext()->NextSelected();
	}
	// ... 原来的机器人 link 刷新
}
