#include "MovableWrl.h"

#include <algorithm>
#include <cstdlib>
#include <stdexcept>

#include <QApplication>
#include <QCoreApplication>
#include <QDebug>
#include <QEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QObject>
#include <QString>
#include <QtGlobal>
#include <QWidget>

#include <Inventor/SoDB.h>
#include <Inventor/SoFullPath.h>
#include <Inventor/SoInput.h>
#include <Inventor/SoInteraction.h>
#include <Inventor/SoOutput.h>
#include <Inventor/SoPickedPoint.h>
#include <Inventor/SoPath.h>
#include <Inventor/VRMLnodes/SoVRMLGroup.h>
#include <Inventor/SbViewportRegion.h>
#include <Inventor/actions/SoGetMatrixAction.h>
#include <Inventor/actions/SoRayPickAction.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/actions/SoToVRML2Action.h>
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/VRMLnodes/SoVRMLShape.h>
#include <Inventor/manips/SoTransformManip.h>
#include <Inventor/manips/SoTransformerManip.h>
#include <Inventor/nodes/SoCamera.h>
#include <Inventor/SbViewVolume.h>
#include <Inventor/sensors/SoFieldSensor.h>
#include <Inventor/sensors/SoSensor.h>

#include <rl/math/Rotation.h>
#include <rl/sg/Exception.h>
#include <rl/sg/Model.h>
#include <rl/sg/Shape.h>

namespace
{
	void rlToSo(const rl::math::Transform& T, SoTransform* t)
	{
		t->translation.setValue(
			static_cast<float>(T.translation().x()),
			static_cast<float>(T.translation().y()),
			static_cast<float>(T.translation().z())
		);

		const rl::math::AngleAxis aa(T.linear());
		const rl::math::Vector3 a = aa.axis();
		t->rotation.setValue(
			SbVec3f(static_cast<float>(a.x()), static_cast<float>(a.y()), static_cast<float>(a.z())),
			static_cast<float>(aa.angle())
		);
		t->scaleFactor.setValue(1.0f, 1.0f, 1.0f);
	}

	SoNode* readWrl(const std::string& file)
	{
		SoInput in;
		if (!in.openFile(file.c_str()))
		{
			throw std::runtime_error("无法打开 WRL: " + file);
		}

		SoSeparator* node = SoDB::readAll(&in);
		in.closeFile();
		if (!node)
		{
			throw std::runtime_error("无法解析 WRL: " + file);
		}
		return node;
	}

	void addCoinDraggerSearchPaths()
	{
		SoInteraction::init();

		if (const char* coinDir = std::getenv("COINDIR"))
		{
			SoInput::addDirectoryFirst((std::string(coinDir) + "/share/Coin/draggerDefaults").c_str());
			SoInput::addDirectoryFirst((std::string(coinDir) + "/data/draggerDefaults").c_str());
		}

		if (const char* draggerDir = std::getenv("COIN_DRAGGER_DIR"))
		{
			SoInput::addDirectoryFirst(draggerDir);
		}
	}

	QPoint mousePos(const QMouseEvent* event)
	{
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
		return event->position().toPoint();
#else
		return event->pos();
#endif
	}

	qreal widgetDpr(const QWidget* widget)
	{
#if QT_VERSION >= QT_VERSION_CHECK(5, 6, 0)
		return widget->devicePixelRatioF();
#else
		Q_UNUSED(widget);
		return 1.0;
#endif
	}

	void copyTransformFields(const SoTransform* src, SoTransform* dst)
	{
		dst->translation = src->translation;
		dst->rotation = src->rotation;
		dst->scaleFactor = src->scaleFactor;
		dst->scaleOrientation = src->scaleOrientation;
		dst->center = src->center;
	}

	rl::math::Transform sbMatrixToRl(const SbMatrix& matrix)
	{
		rl::math::Transform T = rl::math::Transform::Identity();
		for (int row = 0; row < 4; ++row)
		{
			for (int col = 0; col < 4; ++col)
			{
				T(row, col) = matrix[col][row];
			}
		}
		return T;
	}

	std::size_t addShapesFromNode(rl::sg::Body* body, SoNode* root)
	{
		if (!body || !root)
		{
			return 0;
		}

		SoSearchAction search;
		search.setType(SoVRMLShape::getClassTypeId());
		search.setInterest(SoSearchAction::ALL);
		search.setSearchingAll(TRUE);
		search.apply(root);

		SbViewportRegion viewport;
		std::size_t count = 0;
		const SoPathList& paths = search.getPaths();
		for (int i = 0; i < paths.getLength(); ++i)
		{
			SoFullPath* path = static_cast<SoFullPath*>(paths[i]);
			if (!path || path->getLength() < 1)
			{
				continue;
			}

			SoNode* tail = path->getTail();
			if (!tail || !tail->isOfType(SoVRMLShape::getClassTypeId()))
			{
				continue;
			}

			auto* vrmlShape = static_cast<SoVRMLShape*>(tail);
			if (!vrmlShape->geometry.getValue())
			{
				continue;
			}

			SoGetMatrixAction matrixAction(viewport);
			matrixAction.apply(path);

			try
			{
				rl::sg::Shape* shape = body->create(vrmlShape);
				if (!shape)
				{
					continue;
				}
				shape->setTransform(sbMatrixToRl(matrixAction.getMatrix()));
				++count;
			}
			catch (const rl::sg::Exception&)
			{
				qWarning() << "Skip unsupported VRML geometry while building planner collision shape";
			}
			catch (const std::exception& e)
			{
				qWarning() << "Skip VRML geometry:" << e.what();
			}
		}
		return count;
	}

	bool writeNodeToWrl(SoNode* node, const std::string& file)
	{
		if (!node || file.empty())
		{
			return false;
		}

		SoToVRML2Action toVrml;
		toVrml.apply(node);
		SoVRMLGroup* vrmlRoot = toVrml.getVRML2SceneGraph();

		SoOutput out;
		if (!out.openFile(file.c_str()))
		{
			qWarning() << "无法写入 WRL:" << QString::fromStdString(file);
			return false;
		}

		SoNode* toWrite = vrmlRoot ? static_cast<SoNode*>(vrmlRoot) : node;
		toWrite->ref();
		out.setHeaderString(vrmlRoot ? "#VRML V2.0 utf8" : "#Inventor V2.1 ascii");
		SoWriteAction writer(&out);
		writer.apply(toWrite);
		out.closeFile();
		toWrite->unref();
		return true;
	}

	MovableWrlManager* g_manager = nullptr;
}

class MovableWrlEventFilter : public QObject
{
public:
	explicit MovableWrlEventFilter(MovableWrlManager* manager, QObject* parent = nullptr) :
		QObject(parent),
		manager_(manager)
	{
	}

	bool eventFilter(QObject* watched, QEvent* event) override
	{
		if (event->type() == QEvent::KeyPress)
		{
			if (manager_->handleKey(static_cast<QKeyEvent*>(event), watched))
			{
				return true;
			}
		}

		if (event->type() == QEvent::MouseButtonPress)
		{
			const auto* mouseEvent = static_cast<QMouseEvent*>(event);
			if (mouseEvent->button() == Qt::LeftButton &&
				(mouseEvent->modifiers() & Qt::ControlModifier) &&
				manager_->isUnderExaminer(watched))
			{
				auto* widget = qobject_cast<QWidget*>(watched);
				if (widget)
				{
					manager_->pickFromWidget(widget, mousePos(mouseEvent));
					return true;
				}
			}
		}

		return QObject::eventFilter(watched, event);
	}

private:
	MovableWrlManager* manager_;
};

MovableWrl::MovableWrl() = default;

MovableWrl::~MovableWrl()
{
	collisionBody_ = nullptr;
	onPoseChanged = nullptr;

	if (translationSensor_)
	{
		translationSensor_->detach();
		delete translationSensor_;
	}
	if (rotationSensor_)
	{
		rotationSensor_->detach();
		delete rotationSensor_;
	}
	if (centerSensor_)
	{
		centerSensor_->detach();
		delete centerSensor_;
	}
	if (root)
	{
		root->unref();
	}
}

SoTransform* MovableWrl::poseNode() const
{
	if (!root || root->getNumChildren() < 1)
	{
		return nullptr;
	}
	return static_cast<SoTransform*>(root->getChild(0));
}

SoNode* MovableWrl::geometryNode() const
{
	if (!root || root->getNumChildren() < 2)
	{
		return nullptr;
	}
	return root->getChild(1);
}

rl::math::Transform MovableWrl::getPose() const
{
	SoTransform* t = poseNode();
	if (!t)
	{
		return rl::math::Transform::Identity();
	}

	SbMatrix matrix;
	matrix.setTransform(
		t->translation.getValue(),
		t->rotation.getValue(),
		t->scaleFactor.getValue(),
		t->scaleOrientation.getValue(),
		t->center.getValue()
	);
	return sbMatrixToRl(matrix);
}

void MovableWrl::setPose(const rl::math::Transform& T)
{
	if (SoTransform* t = poseNode())
	{
		rlToSo(T, t);
		syncCollisionBody();
	}
}

void MovableWrl::bindCollisionBody(rl::sg::Body* body)
{
	collisionBody_ = body;
	onPoseChanged = [this](const rl::math::Transform& T)
		{
			if (collisionBody_)
			{
				collisionBody_->setFrame(T);
			}
		};
	syncCollisionBody();
}

rl::sg::Body* MovableWrl::collisionBody() const
{
	return collisionBody_;
}

void MovableWrl::unbindCollisionBody()
{
	collisionBody_ = nullptr;
	onPoseChanged = nullptr;
}

void MovableWrl::destroyCollisionBody()
{
	rl::sg::Body* body = collisionBody_;
	collisionBody_ = nullptr;
	onPoseChanged = nullptr;
	if (!body)
	{
		return;
	}

	if (rl::sg::Model* model = body->getModel())
	{
		model->remove(body);
	}
	delete body;
}

void MovableWrl::syncCollisionBody() const
{
	if (onPoseChanged)
	{
		onPoseChanged(getPose());
	}
}

std::size_t MovableWrl::addCollisionShapes(rl::sg::Body* body) const
{
	SoNode* geometry = geometryNode();
	if (!body || !geometry)
	{
		return 0;
	}

	geometry->ref();
	std::size_t count = addShapesFromNode(body, geometry);
	if (count == 0)
	{
		SoToVRML2Action toVrml;
		toVrml.apply(geometry);
		if (SoVRMLGroup* vrmlRoot = toVrml.getVRML2SceneGraph())
		{
			vrmlRoot->ref();
			count = addShapesFromNode(body, vrmlRoot);
			vrmlRoot->unref();
		}
	}
	geometry->unref();
	return count;
}

void MovableWrl::attachPoseSensors()
{
	SoTransform* t = poseNode();
	if (!t)
	{
		return;
	}

	if (!translationSensor_)
	{
		translationSensor_ = new SoFieldSensor(&MovableWrlManager::onPoseFieldChanged, this);
	}
	if (!rotationSensor_)
	{
		rotationSensor_ = new SoFieldSensor(&MovableWrlManager::onPoseFieldChanged, this);
	}
	if (!centerSensor_)
	{
		centerSensor_ = new SoFieldSensor(&MovableWrlManager::onPoseFieldChanged, this);
	}

	translationSensor_->detach();
	rotationSensor_->detach();
	centerSensor_->detach();
	translationSensor_->attach(&t->translation);
	rotationSensor_->attach(&t->rotation);
	centerSensor_->attach(&t->center);
}

bool MovableWrl::saveCurrentPoseWrl(const std::string& file) const
{
	SoTransform* pose = poseNode();
	if (!root || !pose)
	{
		qWarning() << "没有可保存的障碍物模型";
		return false;
	}

	SoSeparator* exportRoot = new SoSeparator();
	exportRoot->ref();

	SoTransform* exportPose = new SoTransform();
	copyTransformFields(pose, exportPose);
	exportRoot->addChild(exportPose);

	for (int i = 1; i < root->getNumChildren(); ++i)
	{
		exportRoot->addChild(root->getChild(i));
	}

	const bool ok = writeNodeToWrl(exportRoot, file);
	exportRoot->unref();
	if (ok)
	{
		qDebug() << "已保存当前位姿 WRL:" << QString::fromStdString(file);
	}
	return ok;
}

MovableWrlManager* MovableWrlManager::install(SoGroup* sceneParent, SoQtExaminerViewer* examiner)
{
	if (!g_manager)
	{
		g_manager = new MovableWrlManager(sceneParent, examiner);
	}
	return g_manager;
}

MovableWrlManager* MovableWrlManager::instance()
{
	return g_manager;
}

void MovableWrlManager::addDraggerDefaultsDir(const std::string& dir)
{
	if (!dir.empty())
	{
		SoInput::addDirectoryFirst(dir.c_str());
	}
}

MovableWrlManager::MovableWrlManager(SoGroup* sceneParent, SoQtExaminerViewer* examiner) :
	examiner_(examiner)
{
	addCoinDraggerSearchPaths();

	movableRoot_ = new SoSeparator();
	movableRoot_->ref();
	movableRoot_->setName("movableWrlRoot");
	sceneParent->addChild(movableRoot_);

	examiner_->setEventCallback(&MovableWrlManager::onViewerEvent, this);
	examiner_->setViewing(TRUE);
	installEventFilter();
}

void MovableWrlManager::installEventFilter()
{
	eventFilter_ = new MovableWrlEventFilter(this);

	if (QWidget* widget = examiner_->getWidget())
	{
		widget->installEventFilter(eventFilter_);
		const QList<QWidget*> children = widget->findChildren<QWidget*>();
		for (QWidget* child : children)
		{
			child->installEventFilter(eventFilter_);
		}
	}

	if (QCoreApplication::instance())
	{
		QCoreApplication::instance()->installEventFilter(eventFilter_);
	}
}

bool MovableWrlManager::isUnderExaminer(QObject* watched) const
{
	QWidget* root = examiner_ ? examiner_->getWidget() : nullptr;
	auto* widget = qobject_cast<QWidget*>(watched);
	while (widget)
	{
		if (widget == root)
		{
			return true;
		}
		widget = widget->parentWidget();
	}
	return false;
}

MovableWrl* MovableWrlManager::addWrl(const std::string& wrlFile, const rl::math::Transform& initialPose)
{
	SoNode* wrl = readWrl(wrlFile);

	SoTransform* transform = new SoTransform();
	rlToSo(initialPose, transform);

	MovableWrl* item = new MovableWrl();
	item->root = new SoSeparator();
	item->root->ref();
	item->root->setName("movableWrl");
	item->root->addChild(transform);
	item->root->addChild(wrl);
	item->sourceFile = wrlFile;
	item->attachPoseSensors();

	movableRoot_->addChild(item->root);
	items_.push_back(item);

	return item;
}

void MovableWrlManager::setEditMode(bool on)
{
	examiner_->setViewing(on ? FALSE : TRUE);
}

bool MovableWrlManager::isEditMode() const
{
	return !examiner_->isViewing();
}

void MovableWrlManager::exitManipulator()
{
	if (selected_)
	{
		detachManip(selected_);
		selected_ = nullptr;
	}
	examiner_->setViewing(TRUE);
}

MovableWrl* MovableWrlManager::selectedItem() const
{
	return selected_;
}

const std::vector<MovableWrl*>& MovableWrlManager::items() const
{
	return items_;
}

bool MovableWrlManager::saveWrl(MovableWrl* item, const std::string& file) const
{
	return item ? item->saveCurrentPoseWrl(file) : false;
}

bool MovableWrlManager::saveSelectedWrl(const std::string& file) const
{
	if (!selected_)
	{
		return false;
	}
	return selected_->saveCurrentPoseWrl(file);
}

bool MovableWrlManager::saveSelectedWrl()
{
	if (!selected_)
	{
		return false;
	}

	QString defaultPath;
	if (!selected_->sourceFile.empty())
	{
		const QFileInfo info(QString::fromStdString(selected_->sourceFile));
		defaultPath = info.absolutePath() + "/" + info.completeBaseName() + "_posed.wrl";
	}
	else
	{
		defaultPath = "obstacle_posed.wrl";
	}

	const QString path = QFileDialog::getSaveFileName(
		examiner_ ? examiner_->getWidget() : nullptr,
		QString::fromUtf8("保存障碍物当前位姿 WRL"),
		defaultPath,
		QString::fromUtf8("VRML (*.wrl);;All files (*)")
	);
	if (path.isEmpty())
	{
		return false;
	}
	return selected_->saveCurrentPoseWrl(path.toStdString());
}

bool MovableWrlManager::removeItem(MovableWrl* item)
{
	if (!item)
	{
		return false;
	}

	const auto it = std::find(items_.begin(), items_.end(), item);
	if (it == items_.end())
	{
		return false;
	}

	if (selected_ == item)
	{
		detachManip(item);
		selected_ = nullptr;
		examiner_->setViewing(TRUE);
	}

	if (item->root && movableRoot_)
	{
		const int index = movableRoot_->findChild(item->root);
		if (index >= 0)
		{
			movableRoot_->removeChild(index);
		}
	}

	item->destroyCollisionBody();
	items_.erase(it);
	delete item;
	return true;
}

bool MovableWrlManager::removeSelected()
{
	if (!selected_)
	{
		return false;
	}
	return removeItem(selected_);
}

bool MovableWrlManager::handleKey(const QKeyEvent* event, QObject* watched)
{
	if (!event)
	{
		return false;
	}

	if (QApplication::activeModalWidget())
	{
		return false;
	}

	if (event->key() == Qt::Key_Escape)
	{
		exitManipulator();
		return true;
	}

	if (watched &&
		(watched->inherits("QLineEdit") ||
			watched->inherits("QTextEdit") ||
			watched->inherits("QPlainTextEdit") ||
			watched->inherits("QAbstractSpinBox") ||
			watched->inherits("QComboBox")))
	{
		return false;
	}

	if (!selected_)
	{
		return false;
	}

	if (event->key() == Qt::Key_Delete)
	{
		removeSelected();
		return true;
	}

	if (event->key() == Qt::Key_S &&
		(event->modifiers() & Qt::ControlModifier) &&
		(event->modifiers() & Qt::ShiftModifier))
	{
		saveSelectedWrl();
		return true;
	}

	return false;
}

SoPath* MovableWrlManager::makePosePath(MovableWrl* item) const
{
	if (!item || !item->poseNode() || !examiner_->getSceneGraph())
	{
		return nullptr;
	}

	SoSearchAction search;
	search.setNode(item->poseNode());
	search.setSearchingAll(TRUE);
	search.setInterest(SoSearchAction::FIRST);
	search.apply(examiner_->getSceneGraph());

	SoPath* found = search.getPath();
	if (found)
	{
		found->ref();
		return found;
	}

	SoPath* path = new SoPath(movableRoot_);
	path->ref();
	path->append(item->root);
	path->append(item->poseNode());
	return path;
}

MovableWrl* MovableWrlManager::findByPath(SoPath* path) const
{
	if (!path)
	{
		return nullptr;
	}

	SoFullPath* full = static_cast<SoFullPath*>(path);
	for (MovableWrl* item : items_)
	{
		for (int i = 0; i < full->getLength(); ++i)
		{
			if (full->getNode(i) == item->root)
			{
				return item;
			}
		}
	}
	return nullptr;
}

void MovableWrlManager::attachManip(MovableWrl* item)
{
	if (!item)
	{
		return;
	}

	if (selected_ && selected_ != item)
	{
		detachManip(selected_);
	}

	SoTransform* t = item->poseNode();
	if (!t || t->isOfType(SoTransformManip::getClassTypeId()))
	{
		selected_ = item;
		setEditMode(true);
		return;
	}

	SoPath* path = makePosePath(item);
	if (!path)
	{
		qWarning("MovableWrl: pose path not found");
		return;
	}

	SoTransformerManip* manip = new SoTransformerManip();
	const SbBool ok = manip->replaceNode(path);
	path->unref();

	if (!ok)
	{
		qWarning("MovableWrl: replaceNode failed, manipulator not shown");
		selected_ = nullptr;
		examiner_->setViewing(TRUE);
		return;
	}

	item->attachPoseSensors();
	selected_ = item;
	setEditMode(true);
	qDebug() << "MovableWrl selected, drag arrows/spheres; ESC to exit";
}

void MovableWrlManager::detachManip(MovableWrl* item)
{
	if (!item)
	{
		return;
	}

	SoTransform* t = item->poseNode();
	if (!t || !t->isOfType(SoTransformManip::getClassTypeId()))
	{
		return;
	}

	SoPath* path = makePosePath(item);
	if (!path)
	{
		return;
	}

	static_cast<SoTransformManip*>(t)->replaceManip(path, new SoTransform());
	path->unref();
	item->attachPoseSensors();
	item->syncCollisionBody();
}

void MovableWrlManager::pickFromWidget(QWidget* widget, const QPoint& pos)
{
	if (!widget || !examiner_ || items_.empty())
	{
		return;
	}

	const qreal dpr = widgetDpr(widget);
	const int x = qRound(pos.x() * dpr);
	const int yTop = qRound(pos.y() * dpr);
	const int height = qRound(widget->height() * dpr);
	const int yBottom = height - yTop - 1;

	SoCamera* camera = examiner_->getCamera();
	const SbViewportRegion vp = examiner_->getViewportRegion();

	auto pickAtPixel = [&](int px, int py) -> MovableWrl*
		{
			SoRayPickAction pick(vp);
			pick.setPickAll(TRUE);
			pick.setRadius(12.0f);

			if (camera)
			{
				const SbVec2s vpsize = vp.getViewportSizePixels();
				if (vpsize[0] <= 0 || vpsize[1] <= 0)
				{
					return nullptr;
				}

				const SbVec2f ndc(
					(static_cast<float>(px) + 0.5f) / static_cast<float>(vpsize[0]),
					(static_cast<float>(py) + 0.5f) / static_cast<float>(vpsize[1])
				);
				const SbViewVolume volume = camera->getViewVolume(vp.getViewportAspectRatio());
				const SbVec3f nearPt = volume.getPlanePoint(camera->nearDistance.getValue(), ndc);
				const SbVec3f farPt = volume.getPlanePoint(camera->farDistance.getValue(), ndc);
				pick.setRay(nearPt, farPt - nearPt);
			}
			else
			{
				pick.setPoint(SbVec2s(static_cast<short>(px), static_cast<short>(py)));
			}

			pick.apply(movableRoot_);

			const SoPickedPointList& list = pick.getPickedPointList();
			for (int i = 0; i < list.getLength(); ++i)
			{
				if (MovableWrl* item = findByPath(list[i]->getPath()))
				{
					return item;
				}
			}

			if (examiner_->getSceneGraph())
			{
				pick.apply(examiner_->getSceneGraph());
				const SoPickedPointList& sceneList = pick.getPickedPointList();
				for (int i = 0; i < sceneList.getLength(); ++i)
				{
					if (MovableWrl* item = findByPath(sceneList[i]->getPath()))
					{
						return item;
					}
				}
			}

			return nullptr;
		};

	MovableWrl* item = pickAtPixel(x, yBottom);
	if (!item)
	{
		item = pickAtPixel(x, yTop);
	}

	if (!item)
	{
		exitManipulator();
		return;
	}

	attachManip(item);
}

SbBool MovableWrlManager::onViewerEvent(void* userData, QEvent* event)
{
	auto* self = static_cast<MovableWrlManager*>(userData);

	if (event->type() == QEvent::KeyPress)
	{
		if (self->handleKey(static_cast<QKeyEvent*>(event), self->examiner_ ? self->examiner_->getWidget() : nullptr))
		{
			return TRUE;
		}
	}

	if (event->type() == QEvent::MouseButtonPress)
	{
		const auto* mouseEvent = static_cast<QMouseEvent*>(event);
		if (mouseEvent->button() == Qt::LeftButton &&
			(mouseEvent->modifiers() & Qt::ControlModifier))
		{
			if (QWidget* widget = self->examiner_->getWidget())
			{
				self->pickFromWidget(widget, mousePos(mouseEvent));
			}
			return TRUE;
		}
	}

	return FALSE;
}

void MovableWrlManager::onPoseFieldChanged(void* userData, SoSensor* /*sensor*/)
{
	static_cast<MovableWrl*>(userData)->syncCollisionBody();
}