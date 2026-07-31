#include <QApplication>
#include <QEventLoop>
#include <QGuiApplication>
#include <QLabel>
#include <QLinearGradient>
#include <QMainWindow>
#include <QPainter>
#include <QProgressBar>
#include <QScreen>
#include <QSplashScreen>
#include <QTimer>
#include <QVBoxLayout>

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    constexpr int W = 1080, H = 540, CX = W / 2;

    // —— 背景：平面深色 + 四角角标 + 细分割线 ——
    QPixmap canvas(W, H);
    {
        QPainter p(&canvas);
        p.setRenderHint(QPainter::Antialiasing);
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

        QLinearGradient line(300, 0, W - 300, 0);
        line.setColorAt(0, QColor(0, 112, 210, 0));
        line.setColorAt(0.5, QColor(0, 112, 210, 120));
        line.setColorAt(1, QColor(0, 112, 210, 0));
        p.setPen(QPen(QBrush(line), 1));
        p.drawLine(300, 318, W - 300, 318);
    }

    QSplashScreen splash(canvas);
    splash.setWindowFlags(splash.windowFlags() | Qt::WindowStaysOnTopHint | Qt::FramelessWindowHint);
    splash.setFixedSize(W, H);

    // —— Logo（1490×468 透明 PNG，等比）——
    auto* logo = new QLabel(&splash);
    QPixmap logoPm(QStringLiteral("./icon/preview.png"));
    if (!logoPm.isNull())
        logoPm = logoPm.scaled(860, 860 * 468 / 1490, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    logo->setPixmap(logoPm);
    logo->setAttribute(Qt::WA_TranslucentBackground);
    logo->adjustSize();
    logo->move(CX - logo->width() / 2, 72);

    auto* version = new QLabel(QStringLiteral("Version 1.0.0"), &splash);
    version->setStyleSheet(QStringLiteral(
        "QLabel{color:#8fa3b8;font-size:14px;letter-spacing:4px;background:transparent;}"));
    version->adjustSize();
    version->move(CX - version->width() / 2, 335);

    auto* status = new QLabel(QString::fromUtf8(u8"正在启动..."), &splash);
    status->setStyleSheet(QStringLiteral(
        "QLabel{color:#c5d4e3;font-size:14px;background:transparent;}"));
    status->adjustSize();

    auto* percent = new QLabel(QStringLiteral("0%"), &splash);
    percent->setStyleSheet(QStringLiteral(
        "QLabel{color:#4aa3e8;font-size:13px;font-weight:600;background:transparent;}"));
    percent->adjustSize();

    auto* progress = new QProgressBar(&splash);
    progress->setRange(0, 100);
    progress->setTextVisible(false);
    progress->setFixedSize(680, 4);
    progress->setStyleSheet(QStringLiteral(
        "QProgressBar{border:none;background:rgba(255,255,255,18);}"
        "QProgressBar::chunk{background:#0070D2;}"));
    progress->move(CX - 340, 455);
    status->move(CX - 340, 418);
    percent->move(CX + 340 - percent->width(), 418);

    splash.show();
    splash.raise();
    if (QScreen* screen = QGuiApplication::primaryScreen())
        splash.move(screen->geometry().center() - splash.rect().center());

    // —— 进度动画 ——
    static const char* stages[] = {
        u8"初始化 UI 界面...", u8"初始化仿真引擎...", u8"加载场景模型...",
        u8"准备渲染资源...", u8"正在启动用户界面...", u8"即将完成，请稍候..."
    };
    const int thresholds[] = {0, 15, 35, 55, 80, 95};

    QEventLoop loop;
    QTimer timer;
    int value = 0;
    QObject::connect(&timer, &QTimer::timeout, [&]() {
        value = qMin(100, value + (value < 25 ? 6 : value < 90 ? 3 : 2));
        progress->setValue(value);

        int i = 5;
        while (i > 0 && value < thresholds[i]) --i;
        status->setText(QString::fromUtf8(stages[i]));
        status->adjustSize();
        status->move(CX - 340, 418);

        percent->setText(QStringLiteral("%1%").arg(value));
        percent->adjustSize();
        percent->move(CX + 340 - percent->width(), 418);

        if (value >= 100) {
            timer.stop();
            QTimer::singleShot(280, &loop, &QEventLoop::quit);
        }
    });
    timer.start(80);
    loop.exec();

    // —— 主窗口 ——
    QMainWindow window;
    window.setWindowTitle(QStringLiteral("HWI"));
    window.resize(1280, 720);
    auto* central = new QWidget(&window);
    auto* layout = new QVBoxLayout(central);
    auto* hello = new QLabel(QString::fromUtf8(u8"主界面已启动"), central);
    hello->setAlignment(Qt::AlignCenter);
    hello->setStyleSheet(QStringLiteral("font-size:24px;color:#1e3a55;"));
    layout->addWidget(hello);
    window.setCentralWidget(central);
    window.show();

    splash.finish(&window);
    return app.exec();
}
