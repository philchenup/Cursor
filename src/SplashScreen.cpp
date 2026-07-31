#include "SplashScreen.h"

#include <QApplication>
#include <QEventLoop>
#include <QGuiApplication>
#include <QLinearGradient>
#include <QPainter>
#include <QRadialGradient>
#include <QScreen>
#include <QTimer>
#include <QtMath>

SplashScreen::SplashScreen(const QString& logoPath,
                           const QString& version,
                           QWidget* parent)
    : QSplashScreen(parent)
    , m_version(version)
{
    setWindowFlags(windowFlags() | Qt::WindowStaysOnTopHint | Qt::FramelessWindowHint);

    QPixmap canvas(kWidth, kHeight);
    canvas.fill(Qt::transparent);
    buildBackground(canvas);
    setPixmap(canvas);
    setFixedSize(kWidth, kHeight);

    // —— Logo：保持 1490x468 比例，最大宽度 720 ——
    m_logoLabel = new QLabel(this);
    QPixmap logo(logoPath);
    if (logo.isNull()) {
        // 占位：无资源时画文字品牌，避免空白
        QPixmap placeholder(kLogoMaxWidth, qRound(kLogoMaxWidth * 468.0 / 1490.0));
        placeholder.fill(Qt::transparent);
        QPainter p(&placeholder);
        p.setRenderHint(QPainter::Antialiasing);
        p.setPen(QColor("#e8eef5"));
        QFont f(QStringLiteral("Segoe UI"), 48, QFont::Bold);
        p.setFont(f);
        p.drawText(placeholder.rect(), Qt::AlignCenter, QStringLiteral("HWI"));
        logo = placeholder;
    } else {
        logo = logo.scaled(kLogoMaxWidth,
                           qRound(kLogoMaxWidth * logo.height() / static_cast<qreal>(logo.width())),
                           Qt::KeepAspectRatio,
                           Qt::SmoothTransformation);
    }
    m_logoLabel->setPixmap(logo);
    m_logoLabel->setAttribute(Qt::WA_TranslucentBackground);
    m_logoLabel->adjustSize();

    // —— 版本号 ——
    m_versionLabel = new QLabel(QStringLiteral("Version %1").arg(m_version), this);
    m_versionLabel->setStyleSheet(QStringLiteral(
        "QLabel {"
        "  color: #9aafc4;"
        "  font-size: 15px;"
        "  font-family: 'Segoe UI', 'Microsoft YaHei', sans-serif;"
        "  letter-spacing: 3px;"
        "  background: transparent;"
        "}"));
    m_versionLabel->adjustSize();

    // —— 加载状态 ——
    m_statusLabel = new QLabel(QString::fromUtf8(u8"正在启动..."), this);
    m_statusLabel->setStyleSheet(QStringLiteral(
        "QLabel {"
        "  color: #c5d4e3;"
        "  font-size: 14px;"
        "  font-family: 'Microsoft YaHei', 'Segoe UI', sans-serif;"
        "  background: transparent;"
        "}"));
    m_statusLabel->adjustSize();

    // —— 百分比 ——
    m_percentLabel = new QLabel(QStringLiteral("0%"), this);
    m_percentLabel->setStyleSheet(QStringLiteral(
        "QLabel {"
        "  color: #5eb3f5;"
        "  font-size: 13px;"
        "  font-family: 'Segoe UI', 'Consolas', monospace;"
        "  font-weight: 600;"
        "  background: transparent;"
        "}"));
    m_percentLabel->adjustSize();

    // —— 进度条 ——
    m_progress = new QProgressBar(this);
    m_progress->setRange(0, 100);
    m_progress->setValue(0);
    m_progress->setTextVisible(false);
    m_progress->setFixedSize(640, 6);
    m_progress->setStyleSheet(QStringLiteral(R"(
        QProgressBar {
            border: none;
            border-radius: 3px;
            background-color: rgba(255, 255, 255, 28);
        }
        QProgressBar::chunk {
            border-radius: 3px;
            background: qlineargradient(
                x1:0, y1:0, x2:1, y2:0,
                stop:0 #2b7fc7,
                stop:0.5 #4aa3e8,
                stop:1 #7ec8ff
            );
        }
    )"));

    layoutWidgets();
}

void SplashScreen::buildBackground(QPixmap& canvas) const
{
    QPainter painter(&canvas);
    painter.setRenderHint(QPainter::Antialiasing);

    // 主对角渐变：深蓝石板 → 午夜蓝灰（避免紫系）
    QLinearGradient base(0, 0, canvas.width(), canvas.height());
    base.setColorAt(0.0, QColor("#0c1520"));
    base.setColorAt(0.45, QColor("#152536"));
    base.setColorAt(1.0, QColor("#1e3a55"));
    painter.fillRect(canvas.rect(), base);

    // 中心柔光：托住 Logo
    QRadialGradient glow(canvas.width() / 2.0, canvas.height() * 0.38, canvas.width() * 0.42);
    glow.setColorAt(0.0, QColor(61, 155, 233, 55));
    glow.setColorAt(0.45, QColor(61, 155, 233, 18));
    glow.setColorAt(1.0, QColor(61, 155, 233, 0));
    painter.fillRect(canvas.rect(), glow);

    // 顶部细高光带
    QLinearGradient topSheen(0, 0, 0, 90);
    topSheen.setColorAt(0.0, QColor(255, 255, 255, 18));
    topSheen.setColorAt(1.0, QColor(255, 255, 255, 0));
    painter.fillRect(0, 0, canvas.width(), 90, topSheen);

    // 底部暗角
    QLinearGradient bottomFade(0, canvas.height() - 140, 0, canvas.height());
    bottomFade.setColorAt(0.0, QColor(0, 0, 0, 0));
    bottomFade.setColorAt(1.0, QColor(0, 0, 0, 90));
    painter.fillRect(0, canvas.height() - 140, canvas.width(), 140, bottomFade);

    // 装饰细线：Logo 区域下方
    const int lineY = 330;
    const int lineMargin = 280;
    QLinearGradient lineGrad(lineMargin, 0, canvas.width() - lineMargin, 0);
    lineGrad.setColorAt(0.0, QColor(94, 179, 245, 0));
    lineGrad.setColorAt(0.5, QColor(94, 179, 245, 120));
    lineGrad.setColorAt(1.0, QColor(94, 179, 245, 0));
    painter.setPen(QPen(QBrush(lineGrad), 1.0));
    painter.drawLine(lineMargin, lineY, canvas.width() - lineMargin, lineY);

    // 四角微弱边框高光
    painter.setPen(QPen(QColor(255, 255, 255, 22), 1));
    painter.drawRect(0, 0, canvas.width() - 1, canvas.height() - 1);
}

void SplashScreen::layoutWidgets()
{
    const int cx = kWidth / 2;

    // Logo 垂直居中偏上
    const int logoY = 95;
    m_logoLabel->move(cx - m_logoLabel->width() / 2, logoY);

    // 版本：Logo 下方 + 装饰线附近
    m_versionLabel->move(cx - m_versionLabel->width() / 2, 345);

    // 进度条与状态靠底
    m_progress->move(cx - m_progress->width() / 2, 470);

    m_statusLabel->move(cx - m_statusLabel->width() / 2, 430);
    m_percentLabel->move(cx + m_progress->width() / 2 - m_percentLabel->width(), 448);
}

QString SplashScreen::statusForProgress(int value) const
{
    if (value < 15)
        return QString::fromUtf8(u8"初始化 UI 界面...");
    if (value < 35)
        return QString::fromUtf8(u8"初始化仿真引擎...");
    if (value < 55)
        return QString::fromUtf8(u8"加载场景模型...");
    if (value < 80)
        return QString::fromUtf8(u8"准备渲染资源...");
    if (value < 95)
        return QString::fromUtf8(u8"正在启动用户界面...");
    return QString::fromUtf8(u8"即将完成，请稍候...");
}

void SplashScreen::setProgress(int value, const QString& status)
{
    value = qBound(0, 100, value);
    m_progress->setValue(value);

    const QString text = status.isEmpty() ? statusForProgress(value) : status;
    m_statusLabel->setText(text);
    m_statusLabel->adjustSize();
    m_statusLabel->move(kWidth / 2 - m_statusLabel->width() / 2, 430);

    m_percentLabel->setText(QStringLiteral("%1%").arg(value));
    m_percentLabel->adjustSize();
    m_percentLabel->move(kWidth / 2 + m_progress->width() / 2 - m_percentLabel->width(), 448);

    QApplication::processEvents();
}

void SplashScreen::run()
{
    show();
    raise();

    if (QScreen* screen = QGuiApplication::primaryScreen()) {
        move(screen->geometry().center() - rect().center());
    }

    QEventLoop loop;
    QTimer progressTimer;
    int progressValue = 0;

    // 前期稍快、中段加载感、末段放缓
    QObject::connect(&progressTimer, &QTimer::timeout, this, [&]() {
        int step = 4;
        if (progressValue < 25)
            step = 6;
        else if (progressValue < 60)
            step = 4;
        else if (progressValue < 90)
            step = 3;
        else
            step = 2;

        progressValue = qMin(100, progressValue + step);
        setProgress(progressValue);

        if (progressValue >= 100) {
            progressTimer.stop();
            // 短暂停留，让用户看到 100%
            QTimer::singleShot(280, &loop, &QEventLoop::quit);
        }
    });

    progressTimer.start(80);
    loop.exec();
}
