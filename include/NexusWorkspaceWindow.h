#ifndef NEXUS_WORKSPACE_WINDOW_H
#define NEXUS_WORKSPACE_WINDOW_H

#include <QMainWindow>

class QAction;
class QComboBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QSlider;
class QStackedWidget;
class QTabWidget;
class QTableWidget;
class QTextEdit;
class QTreeWidget;
class QWidget;

/**
 * @brief 全新 Command Workspace 布局的 NexusVIT 主窗口预览。
 *
 * 所有原界面按钮/输入框都保留，且 objectName 与现有 MainWindow 槽函数
 * 自动连接约定一致（on_<objectName>_<signal>）。只改布局与 QSS，不改功能入口。
 *
 * 接入方式：
 *   1. 用本窗口替换 QMainWindow 的 setupUi 布局；或
 *   2. 仅调用 ApplyNexusWorkspaceTheme(window) 套用新主题。
 */
class NexusWorkspaceWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit NexusWorkspaceWindow(QWidget* parent = nullptr);

    static QString loadWorkspaceStyleSheet();

public slots:
    void on_actionOpen_triggered();
    void on_actionSave_triggered();
    void on_actionRemove_triggered();
    void on_InitializeRobot_clicked();
    void on_robotConnectBtn_clicked();
    void on_InitializeCamera_clicked();
    void on_cam_btn_connect_clicked();
    void on_cam_btn_capture_clicked();
    void on_cam_btn_add_clicked();
    void on_cam_btn_reset_clicked();
    void on_loadVisionConfigBtn_clicked();
    void on_InitializeLaser_clicked();
    void on_connectLaserBtn_clicked();
    void on_GoCapBtn_clicked();
    void on_setCapBtn_clicked();
    void on_GoHomBtn_clicked();
    void on_setHomBtn_clicked();
    void on_PlaceConfigBtn_clicked();
    void on_AutoCalibBtn_clicked();

private:
    void buildChrome();
    void buildCommandStrip();
    void buildWorkspace();
    void buildDeck();
    void logLine(const QString& message);

    QWidget* makeAxisGrid(const QStringList& names, const QString& prefix);

    QTreeWidget* cloudtree = nullptr;
    QTableWidget* propertyTable = nullptr;
    QWidget* cloudview = nullptr;
    QPlainTextEdit* console = nullptr;
    QStackedWidget* viewStack = nullptr;
    QTabWidget* deckTabs = nullptr;
    QSlider* robotSpeed = nullptr;
    QLabel* speedValue = nullptr;
    QLineEdit* robotIpEdit = nullptr;
    QLineEdit* robotPortEdit = nullptr;
    QComboBox* scoketComb = nullptr;
    QTextEdit* recvBox = nullptr;
    QTextEdit* sendBox = nullptr;
};

void ApplyNexusWorkspaceTheme(QWidget* root);

#endif // NEXUS_WORKSPACE_WINDOW_H
