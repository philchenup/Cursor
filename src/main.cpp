#include <QApplication>
#include <QDesktopWidget>
#include "MainWindow.h"
#include <vtkOutputWindow.h>
#include <QSplashScreen>
#include <QSharedMemory>
#include <QScreen>
#include <QPainter>
#include <QLinearGradient>
#include <QLabel>
#include <QProgressBar>
#include <QEventLoop>
#include <QTimer>
#include <QMessageBox>
#include <QGuiApplication>
#include <QDir>
#include <QFile>

MainWindow* MainWindow::singleton = nullptr;

class EventFilter : public QObject {
public:
    bool eventFilter(QObject* obj, QEvent* event) override {
        switch (event->type()) {
        case QEvent::MouseButtonPress:
        case QEvent::MouseButtonRelease:
        case QEvent::MouseButtonDblClick:
        case QEvent::MouseMove:
        case QEvent::Wheel:
            return true; // 屏蔽鼠标事件
        default:
            return QObject::eventFilter(obj, event);
        }
    }
};

int main(int argc, char* argv[])
{
    QSharedMemory sharedMemory("HSIWELDSYSTEM");

    if (sharedMemory.attach()) {
        QMessageBox::warning(nullptr,
            QString::fromUtf8(u8"提示"),
            QString::fromUtf8(u8"程序已经在运行中！"));
        return 0;
    }
    if (!sharedMemory.create(1)) {
        return 0;
    }

    vtkOutputWindow::SetGlobalWarningDisplay(0); // 关闭 vtk 警告窗口

    qRegisterMetaType<std::vector<rl::math::Vector>>("std::vector<rl::math::Vector>");
    qRegisterMetaType<rl::math::Vector>("rl::math::Vector");
    qRegisterMetaType<rl::plan::VectorList>("rl::plan::VectorList");
    qRegisterMetaType<std::vector<float>>("std::vector<float>");
    qRegisterMetaType<cv::Mat>("cv::Mat");
    qRegisterMetaType<ct::Cloud::Ptr>("ct::Cloud::Ptr");
    qRegisterMetaType<std::string>("std::string");
    qRegisterMetaType<TopoDS_Shape>("TopoDS_Shape");
    qRegisterMetaType<std::vector<TopoDS_Edge>>("std::vector<TopoDS_Edge>");
    qRegisterMetaType<IKSolveParams>("IKSolveParams");
    qRegisterMetaType<IKReturnHomeParams>("IKReturnHomeParams");
    qRegisterMetaType<DiscretePoint>("DiscretePoint");
    qRegisterMetaType<IKGoToStartParams>("IKGoToStartParams");
    qRegisterMetaType<RobotData>("RobotData");
    qRegisterMetaType<Eigen::Affine3f>("Eigen::Affine3f");
    qRegisterMetaType<PlanToPreWeldParams>("PlanToPreWeldParams");

    QApplication a(argc, argv);
    EventFilter filter;
    a.installEventFilter(&filter);

    // —— 启动画面：平面深色、无网格、四角角标 ——
    constexpr int W = 1080, H = 540, CX = W / 2;
    QPixmap canvas(W, H);
    {
        QPainter p(&canvas);
        p.setRenderHint(QPainter::Antialiasing);
        p.setRenderHint(QPainter::SmoothPixmapTransform);
        QLinearGradient bg(0, 0, 0, H);
        bg.setColorAt(0, QColor("#0b121c"));
        bg.setColorAt(1, QColor("#111b2a"));
        p.fillRect(canvas.rect(), bg);

        const int m = 18, len = 22;
        p.setPen(QPen(QColor(0, 112, 210, 140), 1.5));
        p.drawLine(m, m, m + len, m);                 p.drawLine(m, m, m, m + len);
        p.drawLine(W - m - len, m, W - m, m);         p.drawLine(W - m, m, W - m, m + len);
        p.drawLine(m, H - m, m + len, H - m);         p.drawLine(m, H - m - len, m, H - m);
        p.drawLine(W - m - len, H - m, W - m, H - m); p.drawLine(W - m, H - m - len, W - m, H - m);

        // Logo：相对可执行文件目录查找（避免 cwd 不是工程目录时加载失败）
        const QStringList logoCandidates = {
            QCoreApplication::applicationDirPath() + QStringLiteral("/icon/preview.png"),
            QDir::currentPath() + QStringLiteral("/icon/preview.png"),
            QStringLiteral("./icon/preview.png"),
            QStringLiteral("icon/preview.png"),
            QCoreApplication::applicationDirPath() + QStringLiteral("/../icon/preview.png"),
        };
        QPixmap logoPm;
        for (const QString& path : logoCandidates) {
            if (QFile::exists(path)) {
                logoPm.load(path);
                if (!logoPm.isNull())
                    break;
            }
        }
        if (!logoPm.isNull()) {
            logoPm = logoPm.scaled(860, 860 * 468 / 1490, Qt::KeepAspectRatio, Qt::SmoothTransformation);
            p.drawPixmap(CX - logoPm.width() / 2, 72, logoPm);
        }

        QLinearGradient line(300, 0, W - 300, 0);
        line.setColorAt(0, QColor(0, 112, 210, 0));
        line.setColorAt(0.5, QColor(0, 112, 210, 120));
        line.setColorAt(1, QColor(0, 112, 210, 0));
        p.setPen(QPen(QBrush(line), 1));
        p.drawLine(300, 318, W - 300, 318);
    }

    QSplashScreen* splash = new QSplashScreen(canvas);
    splash->setWindowFlags(splash->windowFlags() | Qt::WindowStaysOnTopHint | Qt::FramelessWindowHint);
    splash->setFixedSize(W, H);

    QLabel* version = new QLabel(QStringLiteral("Version 1.0.0"), splash);
    version->setStyleSheet(QStringLiteral(
        "QLabel{color:#8fa3b8;font-size:14px;letter-spacing:4px;background:transparent;}"));
    version->adjustSize();
    version->move(CX - version->width() / 2, 335);

    QLabel* loading = new QLabel(QString::fromUtf8(u8"正在启动..."), splash);
    loading->setStyleSheet(QStringLiteral(
        "QLabel{color:#c5d4e3;font-size:14px;background:transparent;}"));
    loading->adjustSize();

    QLabel* percent = new QLabel(QStringLiteral("0%"), splash);
    percent->setStyleSheet(QStringLiteral(
        "QLabel{color:#4aa3e8;font-size:13px;font-weight:600;background:transparent;}"));
    percent->adjustSize();

    QProgressBar* progress = new QProgressBar(splash);
    progress->setRange(0, 100);
    progress->setTextVisible(false);
    progress->setFixedSize(680, 4);
    progress->setStyleSheet(QStringLiteral(
        "QProgressBar{border:none;background:rgba(255,255,255,18);}"
        "QProgressBar::chunk{background:#0070D2;}"));
    progress->move(CX - 340, 455);
    loading->move(CX - 340, 418);
    percent->move(CX + 340 - percent->width(), 418);

    splash->show();
    splash->raise();
    if (QScreen* screen = QGuiApplication::primaryScreen())
        splash->move(screen->geometry().center() - splash->rect().center());

    const QString stages[] = {
        QString::fromUtf8(u8"初始化 UI 界面..."),
        QString::fromUtf8(u8"初始化仿真引擎..."),
        QString::fromUtf8(u8"加载场景模型..."),
        QString::fromUtf8(u8"准备渲染资源..."),
        QString::fromUtf8(u8"正在启动用户界面..."),
        QString::fromUtf8(u8"即将完成，请稍候...")
    };
    const int thresholds[] = {0, 15, 35, 55, 80, 95};

    QEventLoop loop;
    QTimer timeoutTimer;
    QTimer progressTimer;
    int progressValue = 0;

    QObject::connect(&progressTimer, &QTimer::timeout, [&]() {
        progressValue = qMin(100, progressValue + (progressValue < 25 ? 6 : progressValue < 90 ? 3 : 2));
        progress->setValue(progressValue);

        int i = 5;
        while (i > 0 && progressValue < thresholds[i]) --i;
        loading->setText(stages[i]);
        loading->adjustSize();
        loading->move(CX - 340, 418);

        percent->setText(QStringLiteral("%1%").arg(progressValue));
        percent->adjustSize();
        percent->move(CX + 340 - percent->width(), 418);

        if (progressValue >= 100) {
            progressTimer.stop();
            timeoutTimer.stop();
            loop.quit();
        }
    });

    progressTimer.start(80);
    // 超时兜底，避免主窗口初始化过久一直卡住
    timeoutTimer.setSingleShot(true);
    QObject::connect(&timeoutTimer, &QTimer::timeout, [&]() {
        progressTimer.stop();
        loop.quit();
    });
    timeoutTimer.start(15000);

    MainWindow w;
    loop.exec();
    splash->finish(&w);
    delete splash;
    a.removeEventFilter(&filter);
    w.show();
    return a.exec();
}
