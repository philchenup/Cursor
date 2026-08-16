#include "MovableWrl.h"

#include <cstdlib>
#include <stdexcept>

#include <QtGlobal>
#include <QDebug>
#include <QEvent>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QString>

#include <Inventor/SoDB.h>
#include <Inventor/SoFullPath.h>
#include <Inventor/SoInput.h>
#include <Inventor/SoInteraction.h>
#include <Inventor/SoPickedPoint.h>
#include <Inventor/SoPath.h>
#include <Inventor/actions/SoRayPickAction.h>
#include <Inventor/manips/SoTransformManip.h>
#include <Inventor/manips/SoTransformerManip.h>
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
}

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
	static_cast<SoTransformManip*>(t)->replaceManip(path, new SoTransform());
	path->unref();
	item->attachPoseSensors();
	item->syncCollisionBody();
}

void MovableWrlManager::pickAt(int x, int y)
{
	const SbVec2s glSize = examiner_->getViewportRegion().getViewportSizePixels();
	const int iy = glSize[1] - y - 1;

	SoRayPickAction pick(examiner_->getViewportRegion());
	pick.setPoint(SbVec2s(static_cast<short>(x), static_cast<short>(iy)));
	pick.setPickAll(FALSE);
	pick.apply(examiner_->getSceneGraph());

	SoPickedPoint* picked = pick.getPickedPoint();
	if (!picked)
	{
		exitManipulator();
		return;
	}

	MovableWrl* item = findByPath(picked->getPath());
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
			const QPoint pos = mousePos(mouseEvent);
			self->pickAt(pos.x(), pos.y());
			return TRUE;
		}
	}

	return FALSE;
}

void MovableWrlManager::onPoseFieldChanged(void* userData, SoSensor* /*sensor*/)
{
	static_cast<MovableWrl*>(userData)->syncCollisionBody();
}
