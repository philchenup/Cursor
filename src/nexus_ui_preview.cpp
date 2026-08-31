#include "NexusWorkspaceWindow.h"

#include <QApplication>

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    app.setApplicationName(QStringLiteral("NexusVIT"));
    NexusWorkspaceWindow window;
    window.show();
    return app.exec();
}
