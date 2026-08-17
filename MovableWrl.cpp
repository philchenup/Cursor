#include "MovableWrl.h"

#include <cstdlib>
#include <stdexcept>

#include <QApplication>
#include <QCoreApplication>
#include <QDebug>
#include <QEvent>
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
#include <Inventor/SoPickedPoint.h>
#include <Inventor/SoPath.h>
#include <Inventor/actions/SoRayPickAction.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/manips/SoTransformManip.h>
#include <Inventor/manips/SoTransformerManip.h>
#include <Inventor/nodes/SoCamera.h>
#include <Inventor/SbViewVolume.h>
#include <Inventor/sensors/SoFieldSensor.h>
#include <Inventor/sensors/SoSensor.h>

#include <rl/math/Rotation.h>

namespace
{
rl::math::Transform soToRl(const SoTransform* t)
{
	const SbVec3f p = t->translation.getValue();
	SbVec3f axis;
	float angle = 0.0f;
	t->rotation.getValue(axis, angle);

	rl::math::Transform T = rl::math::Transform::Identity();
	T.translation() = rl::math::Vector3(p[0], p[1], p[2]);
	T.linear() = rl::math::AngleAxis(
		angle,
		rl::math::Vector3(axis[0], axis[1], axis[2])
	).toRotationMatrix();
	return T;
}

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
			const auto* keyEvent = static_cast<QKeyEvent*>(event);
			if (keyEvent->key() == Qt::Key_Escape)
			{
				manager_->exitManipulator();
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

rl::math::Transform MovableWrl::getPose() const
{
	SoTransform* t = poseNode();
	return t ? soToRl(t) : rl::math::Transform::Identity();
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
	onPoseChanged = [body](const rl::math::Transform& T)
	{
		if (body)
		{
			body->setFrame(T);
		}
	};
	syncCollisionBody();
}

void MovableWrl::syncCollisionBody() const
{
	if (onPoseChanged)
	{
		onPoseChanged(getPose());
	}
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

	translationSensor_->detach();
	rotationSensor_->detach();
	translationSensor_->attach(&t->translation);
	rotationSensor_->attach(&t->rotation);
}

MovableWrlManager* MovableWrlManager::install(SoGroup* sceneParent, SoQtExaminerViewer* examiner)
{
	static MovableWrlManager* instance = nullptr;
	if (!instance)
	{
		instance = new MovableWrlManager(sceneParent, examiner);
	}
	return instance;
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
	item->attachPoseSensors();

	movableRoot_->addChild(item->root);
	items_.push_back(item);

	qDebug() << "MovableWrl loaded:" << QString::fromStdString(wrlFile)
		<< "- Ctrl+Left click to show gizmos, ESC to return to camera";
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
		qDebug() << "MovableWrl: Ctrl+click missed model at" << pos;
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
		const auto* keyEvent = static_cast<QKeyEvent*>(event);
		if (keyEvent->key() == Qt::Key_Escape)
		{
			self->exitManipulator();
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
