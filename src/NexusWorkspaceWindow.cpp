#include "NexusWorkspaceWindow.h"

#include <QAction>
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QMetaObject>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMenuBar>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSlider>
#include <QSplitter>
#include <QStackedWidget>
#include <QStatusBar>
#include <QTabWidget>
#include <QTableWidget>
#include <QTextEdit>
#include <QToolBar>
#include <QToolButton>
#include <QTreeWidget>
#include <QVBoxLayout>

namespace {

QString findStyleSheetPath()
{
    const QStringList candidates = {
        QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("ui/nexus_workspace.qss")),
        QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("../ui/nexus_workspace.qss")),
        QStringLiteral("ui/nexus_workspace.qss"),
        QStringLiteral(":/ui/nexus_workspace.qss"),
    };
    for (const QString& path : candidates) {
        if (QFile::exists(path)) {
            return path;
        }
    }
    return QString();
}

QPushButton* pillButton(const QString& objectName, const QString& text, QWidget* parent)
{
    auto* btn = new QPushButton(text, parent);
    btn->setObjectName(objectName);
    return btn;
}

QLineEdit* fieldEdit(const QString& objectName, const QString& value, int width, QWidget* parent)
{
    auto* edit = new QLineEdit(value, parent);
    edit->setObjectName(objectName);
    edit->setFixedWidth(width);
    return edit;
}

} // namespace

QString NexusWorkspaceWindow::loadWorkspaceStyleSheet()
{
    const QString path = findStyleSheetPath();
    if (path.isEmpty()) {
        return QString();
    }
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return QString();
    }
    return QString::fromUtf8(file.readAll());
}

void ApplyNexusWorkspaceTheme(QWidget* root)
{
    if (root == nullptr) {
        return;
    }
    const QString qss = NexusWorkspaceWindow::loadWorkspaceStyleSheet();
    if (!qss.isEmpty()) {
        root->setStyleSheet(qss);
    }
}

NexusWorkspaceWindow::NexusWorkspaceWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setObjectName(QStringLiteral("MainWindow"));
    setWindowTitle(QStringLiteral("NexusVIT"));
    resize(1440, 900);

    buildChrome();
    buildCommandStrip();

    auto* central = new QWidget(this);
    central->setObjectName(QStringLiteral("workspaceRoot"));
    auto* layout = new QVBoxLayout(central);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);
    setCentralWidget(central);

    buildWorkspace();
    buildDeck();

    auto* split = new QSplitter(Qt::Vertical, central);
    split->setObjectName(QStringLiteral("workspaceSplitter"));
    split->addWidget(findChild<QWidget*>(QStringLiteral("workspaceBody")));
    split->addWidget(deckTabs);
    split->setStretchFactor(0, 4);
    split->setStretchFactor(1, 1);
    layout->addWidget(split);

    ApplyNexusWorkspaceTheme(this);
    QMetaObject::connectSlotsByName(this);

    const QList<QAction*> namedActions = findChildren<QAction*>();
    for (QAction* action : namedActions) {
        connect(action, &QAction::triggered, this, [this, action]() {
            logLine(action->text());
        });
    }
    const QList<QPushButton*> buttons = findChildren<QPushButton*>();
    for (QPushButton* btn : buttons) {
        connect(btn, &QPushButton::clicked, this, [this, btn]() {
            logLine(btn->text());
        });
    }
    logLine(QStringLiteral("NexusVIT started!"));
}

void NexusWorkspaceWindow::buildChrome()
{
    auto addMenu = [this](const QString& title) {
        return menuBar()->addMenu(title);
    };
    QMenu* fileMenu = addMenu(QStringLiteral("文件"));
    QMenu* editMenu = addMenu(QStringLiteral("编辑"));
    QMenu* toolMenu = addMenu(QStringLiteral("工具"));
    QMenu* ctrlMenu = addMenu(QStringLiteral("控制"));
    QMenu* viewMenu = addMenu(QStringLiteral("视图"));
    QMenu* optMenu = addMenu(QStringLiteral("选项"));
    QMenu* helpMenu = addMenu(QStringLiteral("帮助"));

    auto act = [this](const QString& name, const QString& text) {
        auto* a = new QAction(text, this);
        a->setObjectName(name);
        return a;
    };
    QAction* actionOpen = act(QStringLiteral("actionOpen"), QStringLiteral("打开"));
    QAction* actionSave = act(QStringLiteral("actionSave"), QStringLiteral("保存"));
    QAction* actionRemove = act(QStringLiteral("actionRemove"), QStringLiteral("删除"));
    QAction* actionColor = act(QStringLiteral("actionColor"), QStringLiteral("着色"));
    QAction* actionCoordinate = act(QStringLiteral("actionCoordinate"), QStringLiteral("坐标系"));
    QAction* actionCutting = act(QStringLiteral("actionCutting"), QStringLiteral("裁剪"));
    QAction* actionSample = act(QStringLiteral("actionSample"), QStringLiteral("降采样"));
    QAction* actionTransform = act(QStringLiteral("actionTransform"), QStringLiteral("变换"));
    QAction* actionMerge = act(QStringLiteral("actionMerge"), QStringLiteral("合并"));
    QAction* actionWorkpiece = act(QStringLiteral("actionWorkpiece"), QStringLiteral("工件库"));
    QAction* actionPause = act(QStringLiteral("actionPause"), QStringLiteral("暂停"));
    QAction* actionTcpCalib = act(QStringLiteral("actionTcpCalib"), QStringLiteral("TCP标定"));
    QAction* actionCalibration = act(QStringLiteral("actionCalibration"), QStringLiteral("手眼标定"));
    QAction* actionVision = act(QStringLiteral("actionVision"), QStringLiteral("视觉配置"));
    QAction* actionExecTraj = act(QStringLiteral("actionExecTraj"), QStringLiteral("单步"));
    QAction* actionTrajectory = act(QStringLiteral("actionTrajectory"), QStringLiteral("连续"));
    QAction* actionHome = act(QStringLiteral("actionHome"), QStringLiteral("重置"));
    QAction* actionAbout = act(QStringLiteral("actionAbout"), QStringLiteral("关于"));

    fileMenu->addAction(actionOpen);
    fileMenu->addAction(actionSave);
    editMenu->addAction(actionRemove);
    toolMenu->addAction(actionColor);
    toolMenu->addAction(actionCoordinate);
    toolMenu->addAction(actionTransform);
    toolMenu->addAction(actionSample);
    toolMenu->addAction(actionMerge);
    toolMenu->addAction(actionCutting);
    ctrlMenu->addAction(actionExecTraj);
    ctrlMenu->addAction(actionTrajectory);
    ctrlMenu->addAction(actionPause);
    ctrlMenu->addAction(actionHome);
    viewMenu->addAction(act(QStringLiteral("actionScreenshot"), QStringLiteral("截图")));
    optMenu->addAction(actionTcpCalib);
    optMenu->addAction(actionCalibration);
    optMenu->addAction(actionWorkpiece);
    optMenu->addAction(actionVision);
    optMenu->addAction(act(QStringLiteral("actionPlaceConfig"), QStringLiteral("摆放设置")));
    helpMenu->addAction(actionAbout);

    statusBar()->showMessage(QStringLiteral("机器人 待机  ·  视觉 待机  ·  通信 待机  ·  工具 待机"));
}

void NexusWorkspaceWindow::buildCommandStrip()
{
    auto* bar = addToolBar(QStringLiteral("CommandStrip"));
    bar->setObjectName(QStringLiteral("commandStrip"));
    bar->setMovable(false);
    bar->setIconSize(QSize(16, 16));
    bar->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);

    auto add = [this, bar](QAction* action) {
        bar->addAction(action);
    };
    add(findChild<QAction*>(QStringLiteral("actionOpen")));
    add(findChild<QAction*>(QStringLiteral("actionSave")));
    add(findChild<QAction*>(QStringLiteral("actionRemove")));
    bar->addSeparator();
    add(findChild<QAction*>(QStringLiteral("actionColor")));
    add(findChild<QAction*>(QStringLiteral("actionCoordinate")));
    add(findChild<QAction*>(QStringLiteral("actionTransform")));
    add(findChild<QAction*>(QStringLiteral("actionSample")));
    add(findChild<QAction*>(QStringLiteral("actionMerge")));
    add(findChild<QAction*>(QStringLiteral("actionCutting")));
    bar->addSeparator();
    add(findChild<QAction*>(QStringLiteral("actionTcpCalib")));
    add(findChild<QAction*>(QStringLiteral("actionCalibration")));
    add(findChild<QAction*>(QStringLiteral("actionWorkpiece")));
    add(findChild<QAction*>(QStringLiteral("actionVision")));
    add(findChild<QAction*>(QStringLiteral("actionPlaceConfig")));
    add(findChild<QAction*>(QStringLiteral("actionScreenshot")));
    bar->addSeparator();
    add(findChild<QAction*>(QStringLiteral("actionExecTraj")));
    add(findChild<QAction*>(QStringLiteral("actionTrajectory")));
    add(findChild<QAction*>(QStringLiteral("actionPause")));
    add(findChild<QAction*>(QStringLiteral("actionHome")));
}

void NexusWorkspaceWindow::buildWorkspace()
{
    auto* body = new QWidget(this);
    body->setObjectName(QStringLiteral("workspaceBody"));
    auto* layout = new QHBoxLayout(body);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    auto* left = new QWidget(body);
    left->setFixedWidth(228);
    auto* leftLay = new QVBoxLayout(left);
    leftLay->setContentsMargins(0, 0, 0, 0);
    leftLay->setSpacing(0);
    auto* dataHeader = new QLabel(QStringLiteral("数据"), left);
    dataHeader->setObjectName(QStringLiteral("dataHeader"));
    cloudtree = new QTreeWidget(left);
    cloudtree->setObjectName(QStringLiteral("cloudtree"));
    cloudtree->setHeaderHidden(true);
    auto* root = new QTreeWidgetItem(cloudtree, QStringList{QStringLiteral("PointCloud")});
    root->addChild(new QTreeWidgetItem(QStringList{QStringLiteral("Workpiece")}));
    new QTreeWidgetItem(cloudtree, QStringList{QStringLiteral("Robot")});
    cloudtree->expandAll();
    leftLay->addWidget(dataHeader);
    leftLay->addWidget(cloudtree);

    cloudview = new QWidget(body);
    cloudview->setObjectName(QStringLiteral("cloudview"));
    auto* viewLay = new QVBoxLayout(cloudview);
    viewLay->setContentsMargins(0, 0, 0, 0);
    viewStack = new QStackedWidget(cloudview);
    viewStack->setObjectName(QStringLiteral("tabWidget"));
    auto* scene = new QWidget(viewStack);
    scene->setObjectName(QStringLiteral("sceneWidget"));
    auto* sceneLay = new QVBoxLayout(scene);
    auto* sceneHint = new QLabel(QStringLiteral("Scene  ·  深色视口（原白底已替换）"), scene);
    sceneHint->setAlignment(Qt::AlignCenter);
    sceneLay->addWidget(sceneHint);
    auto* camera = new QWidget(viewStack);
    camera->setObjectName(QStringLiteral("MonitorWidget"));
    auto* camLay = new QVBoxLayout(camera);
    auto* camHint = new QLabel(QStringLiteral("Camera  ·  预览"), camera);
    camHint->setAlignment(Qt::AlignCenter);
    camLay->addWidget(camHint);
    viewStack->addWidget(scene);
    viewStack->addWidget(camera);

    auto* viewSwitch = new QWidget(cloudview);
    auto* switchLay = new QHBoxLayout(viewSwitch);
    switchLay->setContentsMargins(10, 8, 10, 0);
    auto* sceneBtn = new QPushButton(QStringLiteral("Scene"), viewSwitch);
    auto* camBtn = new QPushButton(QStringLiteral("Camera"), viewSwitch);
    sceneBtn->setCheckable(true);
    camBtn->setCheckable(true);
    sceneBtn->setChecked(true);
    connect(sceneBtn, &QPushButton::clicked, this, [this, sceneBtn, camBtn]() {
        viewStack->setCurrentIndex(0);
        sceneBtn->setChecked(true);
        camBtn->setChecked(false);
        logLine(QStringLiteral("Scene"));
    });
    connect(camBtn, &QPushButton::clicked, this, [this, sceneBtn, camBtn]() {
        viewStack->setCurrentIndex(1);
        camBtn->setChecked(true);
        sceneBtn->setChecked(false);
        logLine(QStringLiteral("Camera"));
    });
    switchLay->addWidget(sceneBtn);
    switchLay->addWidget(camBtn);
    switchLay->addStretch();
    viewLay->addWidget(viewSwitch);
    viewLay->addWidget(viewStack, 1);

    auto* right = new QWidget(body);
    right->setFixedWidth(252);
    auto* rightLay = new QVBoxLayout(right);
    rightLay->setContentsMargins(0, 0, 0, 0);
    auto* propHeader = new QLabel(QStringLiteral("属性"), right);
    propertyTable = new QTableWidget(5, 2, right);
    propertyTable->setObjectName(QStringLiteral("propertyTable"));
    propertyTable->setHorizontalHeaderLabels({QStringLiteral("属性"), QStringLiteral("值")});
    propertyTable->verticalHeader()->setVisible(false);
    propertyTable->horizontalHeader()->setStretchLastSection(true);
    const char* keys[] = {"ID", "类别", "长度", "分辨率", "体积"};
    const char* vals[] = {"PointCloud", "—", "—", "—", "—"};
    for (int i = 0; i < 5; ++i) {
        propertyTable->setItem(i, 0, new QTableWidgetItem(QString::fromUtf8(keys[i])));
        propertyTable->setItem(i, 1, new QTableWidgetItem(QString::fromUtf8(vals[i])));
    }
    rightLay->addWidget(propHeader);
    rightLay->addWidget(propertyTable);

    layout->addWidget(left);
    layout->addWidget(cloudview, 1);
    layout->addWidget(right);
}

QWidget* NexusWorkspaceWindow::makeAxisGrid(const QStringList& names, const QString& prefix)
{
    auto* box = new QWidget;
    auto* grid = new QGridLayout(box);
    grid->setContentsMargins(0, 0, 0, 0);
    for (int i = 0; i < names.size(); ++i) {
        auto* lab = new QLabel(names[i], box);
        auto* edit = fieldEdit(prefix + names[i], QStringLiteral("0.000"), 72, box);
        grid->addWidget(lab, i / 3, (i % 3) * 2);
        grid->addWidget(edit, i / 3, (i % 3) * 2 + 1);
    }
    return box;
}

void NexusWorkspaceWindow::buildDeck()
{
    deckTabs = new QTabWidget(this);
    deckTabs->setObjectName(QStringLiteral("controlDeck"));
    deckTabs->setDocumentMode(true);

    auto* robotPage = new QWidget;
    robotPage->setObjectName(QStringLiteral("robotControlWidget"));
    auto* robotLay = new QVBoxLayout(robotPage);
    auto* robotTop = new QHBoxLayout;
    robotTop->addWidget(new QLabel(QStringLiteral("IP")));
    robotIpEdit = fieldEdit(QStringLiteral("robotIpEdit"), QStringLiteral("192.168.5.1"), 118, robotPage);
    robotTop->addWidget(robotIpEdit);
    robotTop->addWidget(new QLabel(QStringLiteral("Port")));
    robotPortEdit = fieldEdit(QStringLiteral("robotPortEdit"), QStringLiteral("29999"), 64, robotPage);
    robotTop->addWidget(robotPortEdit);
    robotTop->addWidget(pillButton(QStringLiteral("InitializeRobot"), QStringLiteral("Initialize"), robotPage));
    robotTop->addWidget(pillButton(QStringLiteral("robotConnectBtn"), QStringLiteral("Connect"), robotPage));
    robotTop->addWidget(pillButton(QStringLiteral("robotEnableBtn"), QStringLiteral("Enable"), robotPage));
    robotTop->addWidget(new QLabel(QStringLiteral("Speed")));
    robotSpeed = new QSlider(Qt::Horizontal, robotPage);
    robotSpeed->setObjectName(QStringLiteral("robotSpeed"));
    robotSpeed->setRange(1, 100);
    robotSpeed->setValue(5);
    robotSpeed->setFixedWidth(120);
    speedValue = new QLabel(QStringLiteral("5"), robotPage);
    connect(robotSpeed, &QSlider::valueChanged, this, [this](int v) {
        speedValue->setText(QString::number(v));
    });
    robotTop->addWidget(robotSpeed);
    robotTop->addWidget(speedValue);
    robotTop->addStretch();
    robotLay->addLayout(robotTop);

    auto* jog = new QHBoxLayout;
    auto* jointBox = new QFrame(robotPage);
    auto* jointLay = new QVBoxLayout(jointBox);
    jointLay->addWidget(new QLabel(QStringLiteral("关节 J0–J5")));
    jointLay->addWidget(makeAxisGrid({QStringLiteral("J0"), QStringLiteral("J1"), QStringLiteral("J2"),
                                      QStringLiteral("J3"), QStringLiteral("J4"), QStringLiteral("J5")},
                                     QStringLiteral("")));
    auto* cartBox = new QFrame(robotPage);
    auto* cartLay = new QVBoxLayout(cartBox);
    cartLay->addWidget(new QLabel(QStringLiteral("笛卡尔 X Y Z Rx Ry Rz")));
    cartLay->addWidget(makeAxisGrid({QStringLiteral("X"), QStringLiteral("Y"), QStringLiteral("Z"),
                                     QStringLiteral("Rx"), QStringLiteral("Ry"), QStringLiteral("Rz")},
                                    QStringLiteral("")));
    auto* motion = new QHBoxLayout;
    motion->addWidget(pillButton(QStringLiteral("GoCapBtn"), QStringLiteral("GoCap"), robotPage));
    motion->addWidget(pillButton(QStringLiteral("setCapBtn"), QStringLiteral("setCap"), robotPage));
    motion->addWidget(pillButton(QStringLiteral("GoHomBtn"), QStringLiteral("GoHom"), robotPage));
    motion->addWidget(pillButton(QStringLiteral("setHomBtn"), QStringLiteral("setHom"), robotPage));
    motion->addWidget(new QLabel(QStringLiteral("Down Speed")));
    motion->addWidget(fieldEdit(QStringLiteral("downSpeed"), QStringLiteral("10"), 56, robotPage));
    motion->addWidget(pillButton(QStringLiteral("SetDownSpeedBtn"), QStringLiteral("Set"), robotPage));
    motion->addWidget(pillButton(QStringLiteral("PlaceConfigBtn"), QStringLiteral("PlaceConfig"), robotPage));
    motion->addWidget(pillButton(QStringLiteral("AutoCalibBtn"), QStringLiteral("AutoCalib"), robotPage));
    motion->addStretch();
    cartLay->addLayout(motion);
    jog->addWidget(jointBox);
    jog->addWidget(cartBox, 1);
    robotLay->addLayout(jog);

    auto* visionPage = new QWidget;
    auto* visLay = new QVBoxLayout(visionPage);
    auto* visTop = new QHBoxLayout;
    auto* initCam = new QCheckBox(QStringLiteral("Initialize"), visionPage);
    initCam->setObjectName(QStringLiteral("InitializeCamera"));
    auto* saveCam = new QCheckBox(QStringLiteral("Save"), visionPage);
    visTop->addWidget(initCam);
    visTop->addWidget(saveCam);
    auto* search = fieldEdit(QStringLiteral("cam_btn_search"), QString(), 180, visionPage);
    search->setPlaceholderText(QStringLiteral("Search"));
    visTop->addWidget(search);
    visTop->addWidget(pillButton(QStringLiteral("cam_btn_connect"), QStringLiteral("Connect"), visionPage));
    visTop->addWidget(pillButton(QStringLiteral("cam_btn_capture"), QStringLiteral("Cap"), visionPage));
    visTop->addWidget(pillButton(QStringLiteral("cam_btn_add"), QStringLiteral("Add"), visionPage));
    visTop->addWidget(pillButton(QStringLiteral("cam_btn_reset"), QStringLiteral("Reset"), visionPage));
    visTop->addStretch();
    visLay->addLayout(visTop);
    auto* visPath = new QHBoxLayout;
    visPath->addWidget(new QLabel(QStringLiteral("Work Path")));
    auto* workPath = new QComboBox(visionPage);
    workPath->addItem(QStringLiteral("选择工作路径"));
    visPath->addWidget(workPath);
    visPath->addWidget(new QLabel(QStringLiteral("Vision Config")));
    auto* visionConfigEdit = new QComboBox(visionPage);
    visionConfigEdit->setObjectName(QStringLiteral("visionConfigEdit"));
    visionConfigEdit->addItem(QStringLiteral("选择视觉配置"));
    visPath->addWidget(visionConfigEdit);
    visPath->addWidget(pillButton(QStringLiteral("loadVisionConfigBtn"), QStringLiteral("…"), visionPage));
    visPath->addStretch();
    visLay->addLayout(visPath);
    visLay->addStretch();

    auto* toolPage = new QWidget;
    auto* toolLay = new QVBoxLayout(toolPage);
    auto* screwType = new QComboBox(toolPage);
    screwType->addItem(QStringLiteral("Screw"));
    toolLay->addWidget(screwType, 0, Qt::AlignLeft);
    auto* toolRow = new QHBoxLayout;
    toolRow->addWidget(new QLabel(QStringLiteral("IP")));
    toolRow->addWidget(fieldEdit(QStringLiteral("screwIp"), QStringLiteral("192.168.5.50"), 118, toolPage));
    toolRow->addWidget(new QLabel(QStringLiteral("Port")));
    toolRow->addWidget(fieldEdit(QStringLiteral("screwPort"), QStringLiteral("5000"), 64, toolPage));
    toolRow->addWidget(pillButton(QStringLiteral("InitializeLaser"), QStringLiteral("Initialize"), toolPage));
    toolRow->addWidget(pillButton(QStringLiteral("connectLaserBtn"), QStringLiteral("Connect"), toolPage));
    toolRow->addWidget(pillButton(QStringLiteral("StartScrewBtn"), QStringLiteral("Start"), toolPage));
    toolRow->addWidget(pillButton(QStringLiteral("BackScrewBtn"), QStringLiteral("Back"), toolPage));
    toolRow->addWidget(pillButton(QStringLiteral("ResetScrewBtn"), QStringLiteral("Reset"), toolPage));
    toolRow->addStretch();
    toolLay->addLayout(toolRow);
    toolLay->addStretch();

    auto* commPage = new QWidget;
    auto* commLay = new QVBoxLayout(commPage);
    scoketComb = new QComboBox(commPage);
    scoketComb->setObjectName(QStringLiteral("scoketComb"));
    scoketComb->addItems({QStringLiteral("TCP Server"), QStringLiteral("TCP Client"), QStringLiteral("UDP")});
    commLay->addWidget(scoketComb, 0, Qt::AlignLeft);
    auto* commRow = new QHBoxLayout;
    commRow->addWidget(new QLabel(QStringLiteral("IP")));
    commRow->addWidget(fieldEdit(QStringLiteral("commIp"), QStringLiteral("127.0.0.1"), 118, commPage));
    commRow->addWidget(new QLabel(QStringLiteral("Port")));
    commRow->addWidget(fieldEdit(QStringLiteral("commPort"), QStringLiteral("3000"), 64, commPage));
    commRow->addWidget(pillButton(QStringLiteral("InitializeCommBtn"), QStringLiteral("Initialize"), commPage));
    commRow->addWidget(pillButton(QStringLiteral("connectCommBtn"), QStringLiteral("Connect"), commPage));
    commRow->addWidget(pillButton(QStringLiteral("sendCommBtn"), QStringLiteral("Send"), commPage));
    commRow->addStretch();
    commLay->addLayout(commRow);
    auto* io = new QHBoxLayout;
    recvBox = new QTextEdit(commPage);
    recvBox->setReadOnly(true);
    sendBox = new QTextEdit(commPage);
    sendBox->setPlainText(QStringLiteral("NexusVIT ping"));
    auto* recvCol = new QVBoxLayout;
    recvCol->addWidget(new QLabel(QStringLiteral("Recv")));
    recvCol->addWidget(recvBox);
    auto* sendCol = new QVBoxLayout;
    sendCol->addWidget(new QLabel(QStringLiteral("Send")));
    sendCol->addWidget(sendBox);
    io->addLayout(recvCol);
    io->addLayout(sendCol);
    commLay->addLayout(io);
    connect(findChild<QPushButton*>(QStringLiteral("sendCommBtn")), &QPushButton::clicked, this, [this]() {
        recvBox->append(QStringLiteral("[%1] %2")
                            .arg(QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss")),
                                 sendBox->toPlainText()));
        logLine(QStringLiteral("Send"));
    });

    auto* consolePage = new QWidget;
    auto* consoleLay = new QVBoxLayout(consolePage);
    console = new QPlainTextEdit(consolePage);
    console->setObjectName(QStringLiteral("console"));
    console->setReadOnly(true);
    consoleLay->addWidget(console);

    deckTabs->addTab(robotPage, QStringLiteral("机器人"));
    deckTabs->addTab(visionPage, QStringLiteral("视觉"));
    deckTabs->addTab(toolPage, QStringLiteral("工具"));
    deckTabs->addTab(commPage, QStringLiteral("通信"));
    deckTabs->addTab(consolePage, QStringLiteral("控制台"));
}

void NexusWorkspaceWindow::logLine(const QString& message)
{
    if (console == nullptr) {
        return;
    }
    const QString ts = QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss"));
    console->appendPlainText(QStringLiteral("[%1]: %2").arg(ts, message));
}

void NexusWorkspaceWindow::on_actionOpen_triggered() {}
void NexusWorkspaceWindow::on_actionSave_triggered() {}
void NexusWorkspaceWindow::on_actionRemove_triggered() {}
void NexusWorkspaceWindow::on_InitializeRobot_clicked() {}
void NexusWorkspaceWindow::on_robotConnectBtn_clicked() {}
void NexusWorkspaceWindow::on_InitializeCamera_clicked() {}
void NexusWorkspaceWindow::on_cam_btn_connect_clicked() {}
void NexusWorkspaceWindow::on_cam_btn_capture_clicked() {}
void NexusWorkspaceWindow::on_cam_btn_add_clicked() {}
void NexusWorkspaceWindow::on_cam_btn_reset_clicked() {}
void NexusWorkspaceWindow::on_loadVisionConfigBtn_clicked() {}
void NexusWorkspaceWindow::on_InitializeLaser_clicked() {}
void NexusWorkspaceWindow::on_connectLaserBtn_clicked() {}
void NexusWorkspaceWindow::on_GoCapBtn_clicked() {}
void NexusWorkspaceWindow::on_setCapBtn_clicked() {}
void NexusWorkspaceWindow::on_GoHomBtn_clicked() {}
void NexusWorkspaceWindow::on_setHomBtn_clicked() {}
void NexusWorkspaceWindow::on_PlaceConfigBtn_clicked() {}
void NexusWorkspaceWindow::on_AutoCalibBtn_clicked() {}
