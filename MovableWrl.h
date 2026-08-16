#ifndef MOVABLE_WRL_H
#define MOVABLE_WRL_H

#include <functional>
#include <string>
#include <vector>

#include <Inventor/nodes/SoGroup.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>

#include <rl/math/Transform.h>
#include <rl/sg/Body.h>

class QEvent;
class SoFieldSensor;
class SoPath;
class SoSensor;

class MovableWrl
{
public:
	MovableWrl();
	~MovableWrl();

	SoSeparator* root = nullptr;

	SoTransform* poseNode() const;

	rl::math::Transform getPose() const;
	void setPose(const rl::math::Transform& T);

	void bindCollisionBody(rl::sg::Body* body);
	void syncCollisionBody() const;
	void attachPoseSensors();

	std::function<void(const rl::math::Transform&)> onPoseChanged;

private:
	SoFieldSensor* translationSensor_ = nullptr;
	SoFieldSensor* rotationSensor_ = nullptr;
};

class MovableWrlManager
{
public:
	static MovableWrlManager* install(SoGroup* sceneParent, SoQtExaminerViewer* examiner);

	static void addDraggerDefaultsDir(const std::string& dir);

	MovableWrl* addWrl(const std::string& wrlFile,
		const rl::math::Transform& initialPose = rl::math::Transform::Identity());

	// Ctrl+左键选中后进入编辑；ESC 退出并恢复转相机
	void setEditMode(bool on);
	bool isEditMode() const;
	void exitManipulator();
	MovableWrl* selectedItem() const;

private:
	MovableWrlManager(SoGroup* sceneParent, SoQtExaminerViewer* examiner);

	static SbBool onViewerEvent(void* userData, QEvent* event);
	static void onPoseFieldChanged(void* userData, SoSensor* sensor);

	SoPath* makePosePath(MovableWrl* item) const;
	MovableWrl* findByPath(SoPath* path) const;
	void attachManip(MovableWrl* item);
	void detachManip(MovableWrl* item);
	void pickAt(int x, int y);

	SoSeparator* movableRoot_ = nullptr;
	SoQtExaminerViewer* examiner_ = nullptr;
	std::vector<MovableWrl*> items_;
	MovableWrl* selected_ = nullptr;

	friend class MovableWrl;
};

inline MovableWrl* addMovableWrl(SoGroup* sceneParent,
	SoQtExaminerViewer* examiner,
	const std::string& wrlFile,
	const rl::math::Transform& initialPose = rl::math::Transform::Identity())
{
	return MovableWrlManager::install(sceneParent, examiner)->addWrl(wrlFile, initialPose);
}

#endif
