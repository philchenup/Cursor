#include "MainWindow.h"

#include "Icons.h"
#include "ViewportWidget.h"

#include <QAction>
#include <QComboBox>
#include <QFile>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMenuBar>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSlider>
#include <QSplitter>
#include <QTabWidget>
#include <QTableWidget>
#include <QToolButton>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <QAbstractItemView>

namespace {

QFrame* hLine()
{
    auto* line = new QFrame;
    line->setFrameShape(QFrame::VLine);
    line->setObjectName(QStringLiteral("ribbonSep"));
    line->setFixedWidth(1);
    return line;
}

QWidget* labeledField(const QString& label, QWidget* field)
{
    auto* w = new QWidget;
    auto* lay = new QHBoxLayout(w);
    lay->setContentsMargins(0, 0, 0, 0);
    lay->setSpacing(6);
    auto* lab = new QLabel(label);
    lab->setObjectName(QStringLiteral("fieldLabel"));
    lay->addWidget(lab);
    lay->addWidget(field, 1);
    return w;
}

} // namespace

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setObjectName(QStringLiteral("MainWindow"));
    setWindowTitle(QStringLiteral("NexusVIT UNORDERED GRASP"));
    resize(1600, 920);
    setMinimumSize(1280, 760);

    auto* root = new QWidget(this);
    root->setObjectName(QStringLiteral("appRoot"));
    auto* rootLay = new QVBoxLayout(root);
    rootLay->setContentsMargins(0, 0, 0, 0);
    rootLay->setSpacing(0);
    setCentralWidget(root);

    buildHeader();
    buildRibbon();
    buildBody();
    applyStyle();
}

void MainWindow::buildHeader()
{
    auto* header = new QWidget;
    header->setObjectName(QStringLiteral("titleHeader"));
    header->setFixedHeight(40);
    auto* lay = new QHBoxLayout(header);
    lay->setContentsMargins(12, 0, 14, 0);
    lay->setSpacing(10);

    auto* logo = new QLabel;
    logo->setPixmap(NexusIcons::logoMark(26));
    logo->setFixedSize(28, 28);

    auto* brand = new QLabel(QStringLiteral("NexusVIT"));
    brand->setObjectName(QStringLiteral("brandLabel"));

    auto* bar = new QMenuBar(header);
    bar->setObjectName(QStringLiteral("topMenu"));
    const QStringList menus = {
        QStringLiteral("File"), QStringLiteral("Edit"), QStringLiteral("Tool"),
        QStringLiteral("Control"), QStringLiteral("View"), QStringLiteral("Option"),
        QStringLiteral("Help")};
    const QStringList menusZh = {
        QStringLiteral("文件"), QStringLiteral("编辑"), QStringLiteral("工具"),
        QStringLiteral("控制"), QStringLiteral("视图"), QStringLiteral("选项"),
        QStringLiteral("帮助")};
    for (int i = 0; i < menus.size(); ++i) {
        auto* menu = bar->addMenu(menus[i]);
        menu->addAction(menusZh[i]);
        menu->addSeparator();
        menu->addAction(QStringLiteral("…"));
    }

    auto* pills = new QWidget;
    auto* pillLay = new QHBoxLayout(pills);
    pillLay->setContentsMargins(0, 0, 0, 0);
    pillLay->setSpacing(8);
    pillLay->addWidget(statusPill(QStringLiteral("Robot"), QStringLiteral("Idle")));
    pillLay->addWidget(statusPill(QStringLiteral("Vision"), QStringLiteral("Idle")));
    pillLay->addWidget(statusPill(QStringLiteral("Communication"), QStringLiteral("Idle")));
    pillLay->addWidget(statusPill(QStringLiteral("Tool"), QStringLiteral("Idle")));

    lay->addWidget(logo);
    lay->addWidget(brand);
    lay->addSpacing(8);
    lay->addWidget(bar, 1);
    lay->addWidget(pills);

    qobject_cast<QVBoxLayout*>(centralWidget()->layout())->addWidget(header);
}

QWidget* MainWindow::statusPill(const QString& name, const QString& state)
{
    auto* pill = new QFrame;
    pill->setObjectName(QStringLiteral("statusPill"));
    auto* lay = new QHBoxLayout(pill);
    lay->setContentsMargins(10, 4, 12, 4);
    lay->setSpacing(6);
    auto* dot = new QLabel;
    dot->setPixmap(NexusIcons::statusDot(QColor(61, 220, 132), 10));
    auto* text = new QLabel(QStringLiteral("%1  %2").arg(name, state));
    text->setObjectName(QStringLiteral("statusPillText"));
    lay->addWidget(dot);
    lay->addWidget(text);
    return pill;
}

QToolButton* MainWindow::ribbonButton(const QString& icon, const QString& text)
{
    auto* btn = new QToolButton;
    btn->setObjectName(QStringLiteral("ribbonBtn"));
    btn->setIcon(NexusIcons::toolbar(icon));
    btn->setIconSize(QSize(22, 22));
    btn->setText(text);
    btn->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    btn->setAutoRaise(true);
    btn->setCursor(Qt::PointingHandCursor);
    btn->setFixedWidth(64);
    return btn;
}

QPushButton* MainWindow::accentButton(const QString& text, const QString& role, const QString& objectName)
{
    auto* btn = new QPushButton(text);
    btn->setProperty("role", role);
    btn->setCursor(Qt::PointingHandCursor);
    if (!objectName.isEmpty()) {
        btn->setObjectName(objectName);
    }
    return btn;
}

QWidget* MainWindow::ribbonGroup(const QString& title, const QList<QWidget*>& buttons)
{
    auto* group = new QWidget;
    group->setObjectName(QStringLiteral("ribbonGroup"));
    auto* lay = new QVBoxLayout(group);
    lay->setContentsMargins(6, 4, 6, 2);
    lay->setSpacing(2);
    auto* row = new QWidget;
    auto* rowLay = new QHBoxLayout(row);
    rowLay->setContentsMargins(0, 0, 0, 0);
    rowLay->setSpacing(2);
    for (QWidget* b : buttons) {
        rowLay->addWidget(b);
    }
    auto* caption = new QLabel(title);
    caption->setObjectName(QStringLiteral("ribbonCaption"));
    caption->setAlignment(Qt::AlignCenter);
    lay->addWidget(row, 1);
    lay->addWidget(caption);
    return group;
}

void MainWindow::buildRibbon()
{
    auto* ribbon = new QWidget;
    ribbon->setObjectName(QStringLiteral("ribbon"));
    ribbon->setFixedHeight(86);
    auto* lay = new QHBoxLayout(ribbon);
    lay->setContentsMargins(8, 2, 12, 2);
    lay->setSpacing(4);

    lay->addWidget(ribbonGroup(QStringLiteral("文件"), {
        ribbonButton(QStringLiteral("open"), QStringLiteral("打开")),
        ribbonButton(QStringLiteral("save"), QStringLiteral("保存")),
        ribbonButton(QStringLiteral("delete"), QStringLiteral("删除")),
        ribbonButton(QStringLiteral("delete_all"), QStringLiteral("全部删除")),
    }));
    lay->addWidget(hLine());
    lay->addWidget(ribbonGroup(QStringLiteral("点云"), {
        ribbonButton(QStringLiteral("color"), QStringLiteral("着色")),
        ribbonButton(QStringLiteral("coordinate"), QStringLiteral("坐标系")),
        ribbonButton(QStringLiteral("transform"), QStringLiteral("变换")),
        ribbonButton(QStringLiteral("sample"), QStringLiteral("降采样")),
        ribbonButton(QStringLiteral("merge"), QStringLiteral("合并")),
        ribbonButton(QStringLiteral("clip"), QStringLiteral("裁剪")),
    }));
    lay->addWidget(hLine());
    lay->addWidget(ribbonGroup(QStringLiteral("标定"), {
        ribbonButton(QStringLiteral("tcp"), QStringLiteral("TCP标定")),
        ribbonButton(QStringLiteral("handeye"), QStringLiteral("手眼标定")),
        ribbonButton(QStringLiteral("library"), QStringLiteral("工件库")),
        ribbonButton(QStringLiteral("vision"), QStringLiteral("视觉配置")),
        ribbonButton(QStringLiteral("place"), QStringLiteral("摆放配置")),
        ribbonButton(QStringLiteral("screenshot"), QStringLiteral("截图")),
    }));
    lay->addStretch(1);

    auto* controls = new QWidget;
    auto* cLay = new QHBoxLayout(controls);
    cLay->setContentsMargins(8, 10, 4, 10);
    cLay->setSpacing(8);

    auto* step = accentButton(QStringLiteral("  单步  "), QStringLiteral("outline-green"), QStringLiteral("btnStep"));
    auto* cont = accentButton(QStringLiteral("  连续  "), QStringLiteral("solid-green"), QStringLiteral("btnContinuous"));
    auto* pause = accentButton(QStringLiteral("  暂停  "), QStringLiteral("solid-orange"), QStringLiteral("btnPause"));
    auto* reset = accentButton(QStringLiteral("  复位  "), QStringLiteral("outline-blue"), QStringLiteral("btnReset"));
    step->setIcon(NexusIcons::toolbar(QStringLiteral("step"), QColor(80, 200, 120)));
    cont->setIcon(NexusIcons::toolbar(QStringLiteral("continuous"), QColor(20, 40, 28)));
    pause->setIcon(NexusIcons::toolbar(QStringLiteral("pause"), QColor(40, 24, 8)));
    reset->setIcon(NexusIcons::toolbar(QStringLiteral("reset"), QColor(90, 160, 230)));
    for (QPushButton* b : {step, cont, pause, reset}) {
        b->setMinimumSize(92, 40);
        b->setIconSize(QSize(16, 16));
    }
    cLay->addWidget(step);
    cLay->addWidget(cont);
    cLay->addWidget(pause);
    cLay->addWidget(reset);
    lay->addWidget(controls);

    qobject_cast<QVBoxLayout*>(centralWidget()->layout())->addWidget(ribbon);
}

QWidget* MainWindow::buildLeftPanel()
{
    auto* panel = new QWidget;
    panel->setObjectName(QStringLiteral("leftPanel"));
    auto* lay = new QVBoxLayout(panel);
    lay->setContentsMargins(8, 8, 8, 8);
    lay->setSpacing(8);

    auto* dataBox = new QFrame;
    dataBox->setObjectName(QStringLiteral("panelCard"));
    auto* dataLay = new QVBoxLayout(dataBox);
    dataLay->setContentsMargins(8, 8, 8, 8);
    auto* dataTitle = new QLabel(QStringLiteral("Data"));
    dataTitle->setObjectName(QStringLiteral("cardTitle"));
    m_dataTree = new QTreeWidget;
    m_dataTree->setObjectName(QStringLiteral("cloudtree"));
    m_dataTree->setColumnCount(2);
    m_dataTree->setHeaderHidden(true);
    m_dataTree->setRootIsDecorated(true);
    m_dataTree->setIndentation(16);
    m_dataTree->header()->setStretchLastSection(false);
    m_dataTree->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    m_dataTree->header()->setSectionResizeMode(1, QHeaderView::Fixed);
    m_dataTree->setColumnWidth(1, 22);
    auto* root = new QTreeWidgetItem(m_dataTree, QStringList{QStringLiteral("PointCloud")});
    root->setIcon(1, QIcon(NexusIcons::eyeOpen(14)));
    auto* wp = new QTreeWidgetItem(root, QStringList{QStringLiteral("Workpiece")});
    wp->setIcon(1, QIcon(NexusIcons::eyeOpen(14)));
    auto* robot = new QTreeWidgetItem(root, QStringList{QStringLiteral("Robot")});
    robot->setIcon(1, QIcon(NexusIcons::eyeOpen(14)));
    m_dataTree->expandAll();
    dataLay->addWidget(dataTitle);
    dataLay->addWidget(m_dataTree, 1);

    auto* propBox = new QFrame;
    propBox->setObjectName(QStringLiteral("panelCard"));
    auto* propLay = new QVBoxLayout(propBox);
    propLay->setContentsMargins(8, 8, 8, 8);
    auto* propTitle = new QLabel(QStringLiteral("Properties"));
    propTitle->setObjectName(QStringLiteral("cardTitle"));
    m_props = new QTableWidget(5, 2);
    m_props->setObjectName(QStringLiteral("propertyTable"));
    m_props->setHorizontalHeaderLabels({QStringLiteral("Property"), QStringLiteral("Value")});
    m_props->verticalHeader()->setVisible(false);
    m_props->horizontalHeader()->setStretchLastSection(true);
    m_props->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    m_props->setShowGrid(false);
    m_props->setAlternatingRowColors(true);
    m_props->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_props->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_props->verticalHeader()->setDefaultSectionSize(26);
    const char* keys[] = {"ID", "Category", "Length", "Resolution", "Volume"};
    const char* vals[] = {"0", "Workpiece", "300.00 mm", "1.00 mm", "1250.68 cm³"};
    for (int i = 0; i < 5; ++i) {
        m_props->setItem(i, 0, new QTableWidgetItem(QString::fromUtf8(keys[i])));
        m_props->setItem(i, 1, new QTableWidgetItem(QString::fromUtf8(vals[i])));
    }
    propLay->addWidget(propTitle);
    propLay->addWidget(m_props, 1);

    lay->addWidget(dataBox, 1);
    lay->addWidget(propBox, 1);
    panel->setMinimumWidth(232);
    panel->setMaximumWidth(320);
    return panel;
}

QWidget* MainWindow::coordColumn(const QString& title, const QStringList& labels)
{
    auto* box = new QFrame;
    box->setObjectName(QStringLiteral("coordBox"));
    auto* lay = new QVBoxLayout(box);
    lay->setContentsMargins(8, 6, 8, 6);
    lay->setSpacing(4);
    auto* cap = new QLabel(title);
    cap->setObjectName(QStringLiteral("coordTitle"));
    lay->addWidget(cap);
    for (const QString& name : labels) {
        auto* row = new QWidget;
        auto* rowLay = new QHBoxLayout(row);
        rowLay->setContentsMargins(0, 0, 0, 0);
        rowLay->setSpacing(6);
        auto* lab = new QLabel(name);
        lab->setFixedWidth(28);
        lab->setObjectName(QStringLiteral("axisLabel"));
        auto* val = new QLineEdit(QStringLiteral("0.000"));
        val->setObjectName(QStringLiteral("axisValue"));
        val->setAlignment(Qt::AlignRight);
        val->setReadOnly(true);
        rowLay->addWidget(lab);
        rowLay->addWidget(val, 1);
        lay->addWidget(row);
    }
    return box;
}

QWidget* MainWindow::buildRightPanel()
{
    auto* scroll = new QScrollArea;
    scroll->setObjectName(QStringLiteral("rightScroll"));
    scroll->setWidgetResizable(true);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll->setFrameShape(QFrame::NoFrame);

    auto* panel = new QWidget;
    panel->setObjectName(QStringLiteral("rightPanel"));
    auto* lay = new QVBoxLayout(panel);
    lay->setContentsMargins(8, 8, 8, 8);
    lay->setSpacing(8);

    auto* vision = new QFrame;
    vision->setObjectName(QStringLiteral("panelCard"));
    auto* vLay = new QVBoxLayout(vision);
    vLay->setContentsMargins(10, 8, 10, 10);
    vLay->setSpacing(8);
    auto* vTitle = new QLabel(QStringLiteral("Vision"));
    vTitle->setObjectName(QStringLiteral("cardTitle"));
    auto* search = new QLineEdit;
    search->setPlaceholderText(QStringLiteral("Search"));
    search->setObjectName(QStringLiteral("searchEdit"));
    search->addAction(QIcon(NexusIcons::search(14)), QLineEdit::LeadingPosition);

    auto* vBtns = new QWidget;
    auto* vb = new QHBoxLayout(vBtns);
    vb->setContentsMargins(0, 0, 0, 0);
    vb->setSpacing(6);
    vb->addWidget(accentButton(QStringLiteral("Connect"), QStringLiteral("solid-green"), QStringLiteral("cam_btn_connect")));
    vb->addWidget(accentButton(QStringLiteral("Cap"), QStringLiteral("solid-blue"), QStringLiteral("cam_btn_capture")));
    vb->addWidget(accentButton(QStringLiteral("Add"), QStringLiteral("solid-blue"), QStringLiteral("cam_btn_add")));
    vb->addWidget(accentButton(QStringLiteral("Reset"), QStringLiteral("solid-orange"), QStringLiteral("cam_btn_reset")));

    auto* workPath = new QComboBox;
    workPath->addItems({QStringLiteral("Work Path"), QStringLiteral("/home/work/job_01")});
    auto* visionCfg = new QComboBox;
    visionCfg->addItems({QStringLiteral("Vision Config"), QStringLiteral("unordered_grasp.json")});

    vLay->addWidget(vTitle);
    vLay->addWidget(search);
    vLay->addWidget(vBtns);
    vLay->addWidget(labeledField(QStringLiteral("Work Path"), workPath));
    vLay->addWidget(labeledField(QStringLiteral("Vision Config"), visionCfg));

    auto* robot = new QFrame;
    robot->setObjectName(QStringLiteral("panelCard"));
    auto* rLay = new QVBoxLayout(robot);
    rLay->setContentsMargins(10, 8, 10, 10);
    rLay->setSpacing(8);
    auto* rTitle = new QLabel(QStringLiteral("Robot"));
    rTitle->setObjectName(QStringLiteral("cardTitle"));

    auto* ipRow = new QWidget;
    auto* ipLay = new QHBoxLayout(ipRow);
    ipLay->setContentsMargins(0, 0, 0, 0);
    ipLay->setSpacing(6);
    auto* ip = new QLineEdit(QStringLiteral("192.168.5.1"));
    ip->setObjectName(QStringLiteral("robotIpEdit"));
    auto* port = new QLineEdit(QStringLiteral("29999"));
    port->setObjectName(QStringLiteral("robotPortEdit"));
    port->setFixedWidth(64);
    ipLay->addWidget(labeledField(QStringLiteral("IP"), ip), 1);
    ipLay->addWidget(labeledField(QStringLiteral("Port"), port));

    auto* rBtns = new QWidget;
    auto* rb = new QHBoxLayout(rBtns);
    rb->setContentsMargins(0, 0, 0, 0);
    rb->setSpacing(6);
    rb->addWidget(accentButton(QStringLiteral("Initialize"), QStringLiteral("ghost"), QStringLiteral("InitializeRobot")));
    rb->addWidget(accentButton(QStringLiteral("Connect"), QStringLiteral("solid-green"), QStringLiteral("robotConnectBtn")));
    rb->addWidget(accentButton(QStringLiteral("Enable"), QStringLiteral("solid-green"), QStringLiteral("robotEnableBtn")));

    auto* speedRow = new QWidget;
    auto* sl = new QHBoxLayout(speedRow);
    sl->setContentsMargins(0, 0, 0, 0);
    sl->setSpacing(8);
    auto* speedLab = new QLabel(QStringLiteral("Speed"));
    speedLab->setObjectName(QStringLiteral("fieldLabel"));
    m_speed = new QSlider(Qt::Horizontal);
    m_speed->setObjectName(QStringLiteral("robotSpeed"));
    m_speed->setRange(1, 100);
    m_speed->setValue(5);
    m_speedValue = new QLabel(QStringLiteral("5"));
    m_speedValue->setObjectName(QStringLiteral("speedValue"));
    m_speedValue->setFixedWidth(28);
    connect(m_speed, &QSlider::valueChanged, this, [this](int v) {
        m_speedValue->setText(QString::number(v));
    });
    sl->addWidget(speedLab);
    sl->addWidget(m_speed, 1);
    sl->addWidget(m_speedValue);

    auto* coords = new QWidget;
    auto* cl = new QHBoxLayout(coords);
    cl->setContentsMargins(0, 0, 0, 0);
    cl->setSpacing(6);
    cl->addWidget(coordColumn(QStringLiteral("Joints (°)"),
                              {QStringLiteral("J0"), QStringLiteral("J1"), QStringLiteral("J2"),
                               QStringLiteral("J3"), QStringLiteral("J4"), QStringLiteral("J5")}));
    cl->addWidget(coordColumn(QStringLiteral("Cartesian (mm/°)"),
                              {QStringLiteral("X"), QStringLiteral("Y"), QStringLiteral("Z"),
                               QStringLiteral("Rx"), QStringLiteral("Ry"), QStringLiteral("Rz")}));

    auto* motion = new QWidget;
    auto* ml = new QGridLayout(motion);
    ml->setContentsMargins(0, 0, 0, 0);
    ml->setSpacing(6);
    ml->addWidget(accentButton(QStringLiteral("GoCap"), QStringLiteral("solid-red"), QStringLiteral("GoCapBtn")), 0, 0);
    ml->addWidget(accentButton(QStringLiteral("setCap"), QStringLiteral("solid-blue"), QStringLiteral("setCapBtn")), 0, 1);
    ml->addWidget(accentButton(QStringLiteral("GoHom"), QStringLiteral("solid-orange"), QStringLiteral("GoHomBtn")), 1, 0);
    ml->addWidget(accentButton(QStringLiteral("setHom"), QStringLiteral("solid-blue"), QStringLiteral("setHomBtn")), 1, 1);

    auto* cfgRow = new QWidget;
    auto* cfgLay = new QHBoxLayout(cfgRow);
    cfgLay->setContentsMargins(0, 0, 0, 0);
    cfgLay->setSpacing(6);
    auto* place = new QComboBox;
    place->addItems({QStringLiteral("PlaceConfig"), QStringLiteral("place_01.json")});
    cfgLay->addWidget(place, 1);
    cfgLay->addWidget(accentButton(QStringLiteral("AutoCalib"), QStringLiteral("ghost"), QStringLiteral("AutoCalibBtn")));

    rLay->addWidget(rTitle);
    rLay->addWidget(ipRow);
    rLay->addWidget(rBtns);
    rLay->addWidget(speedRow);
    rLay->addWidget(coords);
    rLay->addWidget(motion);
    rLay->addWidget(cfgRow);

    lay->addWidget(vision);
    lay->addWidget(robot);
    lay->addStretch(1);

    scroll->setWidget(panel);
    scroll->setMinimumWidth(276);
    scroll->setMaximumWidth(360);
    return scroll;
}

QWidget* MainWindow::buildCenter()
{
    auto* center = new QWidget;
    center->setObjectName(QStringLiteral("centerPane"));
    auto* lay = new QVBoxLayout(center);
    lay->setContentsMargins(0, 0, 0, 0);
    lay->setSpacing(0);

    m_viewTabs = new QTabWidget;
    m_viewTabs->setObjectName(QStringLiteral("viewTabs"));
    m_viewTabs->setDocumentMode(true);

    m_viewport = new ViewportWidget;
    auto* camera = new ViewportWidget;
    camera->setMode(QStringLiteral("camera"));

    m_viewTabs->addTab(m_viewport, QStringLiteral("Scene"));
    m_viewTabs->addTab(camera, QStringLiteral("Camera"));

    lay->addWidget(m_viewTabs, 1);
    return center;
}

QWidget* MainWindow::buildBottom()
{
    auto* tabs = new QTabWidget;
    tabs->setObjectName(QStringLiteral("bottomTabs"));
    tabs->setDocumentMode(true);

    auto* toolPage = new QWidget;
    auto* toolLay = new QGridLayout(toolPage);
    toolLay->setContentsMargins(12, 10, 12, 10);
    toolLay->setSpacing(8);
    auto* tip = new QLineEdit(QStringLiteral("192.168.5.50"));
    auto* tport = new QLineEdit(QStringLiteral("5000"));
    tport->setFixedWidth(70);
    toolLay->addWidget(new QLabel(QStringLiteral("Screw  IP")), 0, 0);
    toolLay->addWidget(tip, 0, 1);
    toolLay->addWidget(new QLabel(QStringLiteral("Port")), 0, 2);
    toolLay->addWidget(tport, 0, 3);
    auto* toolBtns = new QHBoxLayout;
    toolBtns->addWidget(accentButton(QStringLiteral("Initialize"), QStringLiteral("ghost")));
    toolBtns->addWidget(accentButton(QStringLiteral("Connect"), QStringLiteral("solid-green")));
    toolBtns->addWidget(accentButton(QStringLiteral("Start"), QStringLiteral("solid-green")));
    toolBtns->addWidget(accentButton(QStringLiteral("Back"), QStringLiteral("ghost")));
    toolBtns->addWidget(accentButton(QStringLiteral("Reset"), QStringLiteral("solid-orange")));
    toolBtns->addStretch(1);
    toolLay->addLayout(toolBtns, 1, 0, 1, 4);
    tabs->addTab(toolPage, QStringLiteral("Tool"));

    auto* commPage = new QWidget;
    auto* commLay = new QGridLayout(commPage);
    commLay->setContentsMargins(12, 10, 12, 10);
    commLay->setSpacing(8);
    auto* proto = new QComboBox;
    proto->addItems({QStringLiteral("TCP Server"), QStringLiteral("TCP Client"), QStringLiteral("UDP")});
    auto* cip = new QLineEdit(QStringLiteral("192.168.5.10"));
    auto* cport = new QLineEdit(QStringLiteral("8080"));
    cport->setFixedWidth(70);
    commLay->addWidget(new QLabel(QStringLiteral("Mode")), 0, 0);
    commLay->addWidget(proto, 0, 1);
    commLay->addWidget(new QLabel(QStringLiteral("IP")), 0, 2);
    commLay->addWidget(cip, 0, 3);
    commLay->addWidget(new QLabel(QStringLiteral("Port")), 0, 4);
    commLay->addWidget(cport, 0, 5);
    auto* recv = new QPlainTextEdit;
    recv->setPlaceholderText(QStringLiteral("Receive"));
    recv->setMaximumHeight(72);
    auto* send = new QPlainTextEdit;
    send->setPlaceholderText(QStringLiteral("Send"));
    send->setMaximumHeight(72);
    commLay->addWidget(recv, 1, 0, 1, 3);
    commLay->addWidget(send, 1, 3, 1, 3);
    auto* commBtns = new QHBoxLayout;
    commBtns->addWidget(accentButton(QStringLiteral("Initialize"), QStringLiteral("ghost")));
    commBtns->addWidget(accentButton(QStringLiteral("Connect"), QStringLiteral("solid-green")));
    commBtns->addWidget(accentButton(QStringLiteral("Send"), QStringLiteral("solid-blue")));
    commBtns->addStretch(1);
    commLay->addLayout(commBtns, 2, 0, 1, 6);
    tabs->addTab(commPage, QStringLiteral("Communication"));

    auto* consolePage = new QWidget;
    auto* consoleLay = new QVBoxLayout(consolePage);
    consoleLay->setContentsMargins(0, 0, 0, 0);
    auto* consoleBar = new QWidget;
    auto* barLay = new QHBoxLayout(consoleBar);
    barLay->setContentsMargins(8, 4, 8, 0);
    barLay->addStretch(1);
    auto* clear = accentButton(QStringLiteral("清除"), QStringLiteral("ghost"), QStringLiteral("btnClear"));
    barLay->addWidget(clear);
    m_console = new QPlainTextEdit;
    m_console->setObjectName(QStringLiteral("console"));
    m_console->setReadOnly(true);
    m_console->setPlainText(QStringLiteral("[20:41:19] NexusVIT started!"));
    connect(clear, &QPushButton::clicked, m_console, &QPlainTextEdit::clear);
    consoleLay->addWidget(consoleBar);
    consoleLay->addWidget(m_console, 1);
    tabs->addTab(consolePage, QStringLiteral("Console"));
    tabs->setCurrentIndex(2);

    tabs->setMinimumHeight(150);
    return tabs;
}

void MainWindow::buildBody()
{
    auto* splitV = new QSplitter(Qt::Vertical);
    splitV->setObjectName(QStringLiteral("splitV"));
    splitV->setChildrenCollapsible(false);

    auto* splitH = new QSplitter(Qt::Horizontal);
    splitH->setObjectName(QStringLiteral("splitH"));
    splitH->setChildrenCollapsible(false);
    splitH->addWidget(buildLeftPanel());
    splitH->addWidget(buildCenter());
    splitH->addWidget(buildRightPanel());
    splitH->setStretchFactor(0, 0);
    splitH->setStretchFactor(1, 1);
    splitH->setStretchFactor(2, 0);
    splitH->setSizes({250, 980, 300});

    splitV->addWidget(splitH);
    splitV->addWidget(buildBottom());
    splitV->setStretchFactor(0, 1);
    splitV->setStretchFactor(1, 0);
    splitV->setSizes({720, 180});

    qobject_cast<QVBoxLayout*>(centralWidget()->layout())->addWidget(splitV, 1);
}

void MainWindow::applyStyle()
{
    QFile file(QStringLiteral(":/nexusvit.qss"));
    if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        setStyleSheet(QString::fromUtf8(file.readAll()));
    }
}
