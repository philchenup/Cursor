#include "SplashScreen.h"

#include <QApplication>
#include <QEventLoop>
#include <QGuiApplication>
#include <QLinearGradient>
#include <QPainter>
#include <QRadialGradient>
#include <QScreen>
#include <QTimer>

namespace {
// 与 Logo 一致的品牌色
constexpr QRgb kBrandBlue   = 0xFF0070D2;
constexpr QRgb kBrandOrange = 0xFFF39C12;
}

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

    // —— Logo：1490×468 透明 PNG，等比缩放，不再拉伸 ——
    m_logoLabel = new QLabel(this);
    QPixmap logo(logoPath);
    if (logo.isNull()) {
        QPixmap placeholder(kLogoMaxWidth, qRound(kLogoMaxWidth * 468.0 / 1490.0));
        placeholder.fill(Qt::transparent);
        QPainter p(&placeholder);
        p.setRenderHint(QPainter::Antialiasing);
        p.setPen(QColor(kBrandBlue));
        QFont f(QStringLiteral("Segoe UI"), 42, QFont::Bold);
        p.setFont(f);
        p.drawText(placeholder.rect(), Qt::AlignCenter, QStringLiteral("CAM  ·  HWI"));
        logo = placeholder;
    } else {
        logo = logo.scaled(kLogoMaxWidth,
                           qRound(kLogoMaxWidth * logo.height()
                                  / static_cast<qreal>(logo.width())),
                           Qt::KeepAspectRatio,
                           Qt::SmoothTransformation);
    }
    m_logoLabel->setPixmap(logo);
    m_logoLabel->setAttribute(Qt::WA_TranslucentBackground);
    m_logoLabel->adjustSize();

    // —— 版本号（Logo 已含 HWI / 哈焊，此处不再重复大标题）——
    m_versionLabel = new QLabel(QStringLiteral("Version %1").arg(m_version), this);
    m_versionLabel->setStyleSheet(QStringLiteral(
        "QLabel {"
        "  color: #8fa3b8;"
        "  font-size: 14px;"
        "  font-family: 'Segoe UI', 'Microsoft YaHei', sans-serif;"
        "  letter-spacing: 4px;"
        "  background: transparent;"
        "}"));
    m_versionLabel->adjustSize();

    m_statusLabel = new QLabel(QString::fromUtf8(u8"正在启动..."), this);
    m_statusLabel->setStyleSheet(QStringLiteral(
        "QLabel {"
        "  color: #c5d4e3;"
        "  font-size: 14px;"
        "  font-family: 'Microsoft YaHei', 'Segoe UI', sans-serif;"
        "  background: transparent;"
        "}"));
    m_statusLabel->adjustSize();

    m_percentLabel = new QLabel(QStringLiteral("0%"), this);
    m_percentLabel->setStyleSheet(QStringLiteral(
        "QLabel {"
        "  color: #4aa3e8;"
        "  font-size: 13px;"
        "  font-family: 'Segoe UI', 'Consolas', monospace;"
        "  font-weight: 600;"
        "  background: transparent;"
        "}"));
    m_percentLabel->adjustSize();

    // 进度条：品牌蓝 → 品牌橙
    m_progress = new QProgressBar(this);
    m_progress->setRange(0, 100);
    m_progress->setValue(0);
    m_progress->setTextVisible(false);
    m_progress->setFixedSize(680, 7);
    m_progress->setStyleSheet(QStringLiteral(R"(
        QProgressBar {
            border: none;
            border-radius: 3px;
            background-color: rgba(255, 255, 255, 26);
        }
        QProgressBar::chunk {
            border-radius: 3px;
            background: qlineargradient(
                x1:0, y1:0, x2:1, y2:0,
                stop:0 #0070D2,
                stop:0.55 #2B8FE0,
                stop:1 #F39C12
            );
        }
    )"));

    layoutWidgets();
}

void SplashScreen::buildBackground(QPixmap& canvas) const
{
    QPainter painter(&canvas);
    painter.setRenderHint(QPainter::Antialiasing);

    // 深炭底：让 Logo 蓝/橙/红更突出，避免与 #4a6491 抢色
    QLinearGradient base(0, 0, canvas.width(), canvas.height());
    base.setColorAt(0.0, QColor("#0a1018"));
    base.setColorAt(0.5, QColor("#101a28"));
    base.setColorAt(1.0, QColor("#152436"));
    painter.fillRect(canvas.rect(), base);

    // 左侧暖橙柔光（呼应 CAM 橙色三角）
    QRadialGradient warmGlow(canvas.width() * 0.28, canvas.height() * 0.36,
                             canvas.width() * 0.38);
    warmGlow.setColorAt(0.0, QColor(243, 156, 18, 42));
    warmGlow.setColorAt(0.5, QColor(243, 156, 18, 12));
    warmGlow.setColorAt(1.0, QColor(243, 156, 18, 0));
    painter.fillRect(canvas.rect(), warmGlow);

    // 右侧冷蓝柔光（呼应 HWI 椭圆）
    QRadialGradient coolGlow(canvas.width() * 0.72, canvas.height() * 0.36,
                             canvas.width() * 0.40);
    coolGlow.setColorAt(0.0, QColor(0, 112, 210, 55));
    coolGlow.setColorAt(0.5, QColor(0, 112, 210, 16));
    coolGlow.setColorAt(1.0, QColor(0, 112, 210, 0));
    painter.fillRect(canvas.rect(), coolGlow);

    // 顶部细高光
    QLinearGradient topSheen(0, 0, 0, 80);
    topSheen.setColorAt(0.0, QColor(255, 255, 255, 14));
    topSheen.setColorAt(1.0, QColor(255, 255, 255, 0));
    painter.fillRect(0, 0, canvas.width(), 80, topSheen);

    // 底部暗角，托住进度区
    QLinearGradient bottomFade(0, canvas.height() - 130, 0, canvas.height());
    bottomFade.setColorAt(0.0, QColor(0, 0, 0, 0));
    bottomFade.setColorAt(1.0, QColor(0, 0, 0, 100));
    painter.fillRect(0, canvas.height() - 130, canvas.width(), 130, bottomFade);

    // 蓝→橙装饰线（呼应双色标）
    const int lineY = 318;
    const int lineMargin = 260;
    QLinearGradient lineGrad(lineMargin, 0, canvas.width() - lineMargin, 0);
    lineGrad.setColorAt(0.0, QColor(0, 112, 210, 0));
    lineGrad.setColorAt(0.25, QColor(0, 112, 210, 160));
    lineGrad.setColorAt(0.5, QColor(243, 156, 18, 180));
    lineGrad.setColorAt(0.75, QColor(0, 112, 210, 160));
    lineGrad.setColorAt(1.0, QColor(0, 112, 210, 0));
    painter.setPen(QPen(QBrush(lineGrad), 1.2));
    painter.drawLine(lineMargin, lineY, canvas.width() - lineMargin, lineY);

    painter.setPen(QPen(QColor(255, 255, 255, 18), 1));
    painter.drawRect(0, 0, canvas.width() - 1, canvas.height() - 1);
}

void SplashScreen::layoutWidgets()
{
    const int cx = kWidth / 2;

    // Logo 偏上居中，作为唯一品牌主视觉
    const int logoY = 72;
    m_logoLabel->move(cx - m_logoLabel->width() / 2, logoY);

    m_versionLabel->move(cx - m_versionLabel->width() / 2, 335);

    m_progress->move(cx - m_progress->width() / 2, 455);

    m_statusLabel->move(cx - m_progress->width() / 2, 418);
    m_percentLabel->move(cx + m_progress->width() / 2 - m_percentLabel->width(), 418);
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
    m_statusLabel->move(kWidth / 2 - m_progress->width() / 2, 418);

    m_percentLabel->setText(QStringLiteral("%1%").arg(value));
    m_percentLabel->adjustSize();
    m_percentLabel->move(kWidth / 2 + m_progress->width() / 2 - m_percentLabel->width(), 418);

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
            QTimer::singleShot(280, &loop, &QEventLoop::quit);
        }
    });

    progressTimer.start(80);
    loop.exec();
}
