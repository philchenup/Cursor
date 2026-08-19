#include <Inventor/SbName.h>
#include <Inventor/nodes/SoGroup.h>
#include <Inventor/nodes/SoNode.h>
#include <Inventor/nodes/SoSeparator.h>

void MainWindow::showViewerSeparator(SoSeparator* node, const char* name)
{
    if (this->viewer == nullptr || this->viewer->sceneGroup == nullptr) {
        return;
    }

    SoGroup* root = this->viewer->sceneGroup;
    const SbName nodeName((name != nullptr && name[0] != '\0') ? name : "scenePcd");

    // 仅从显示场景图中移除旧节点，不创建 / 绑定碰撞体
    for (int i = root->getNumChildren() - 1; i >= 0; --i) {
        SoNode* child = root->getChild(i);
        if (child != nullptr && child->getName() == nodeName) {
            root->removeChild(i);
        }
    }

    if (node == nullptr) {
        return;
    }

    node->setName(nodeName);
    if (root->findChild(node) < 0) {
        root->addChild(node);
    }
}
