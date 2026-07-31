#include "SplashScreen.h"

#include <QApplication>
#include <QEventLoop>
#include <QGuiApplication>
#include <QLinearGradient>
#include <QPainter>
#include <QScreen>
#include <QTimer>

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
        p.setPen(QColor(QStringLiteral("#0070D2")));
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
    m_progress->setFixedSize(680, 4);
    m_progress->setStyleSheet(QStringLiteral(R"(
        QProgressBar {
            border: none;
            border-radius: 0px;
            background-color: rgba(255, 255, 255, 18);
        }
        QProgressBar::chunk {
            border-radius: 0px;
            background-color: #0070D2;
        }
    )"));

    layoutWidgets();
}

void SplashScreen::buildBackground(QPixmap& canvas) const
{
    QPainter painter(&canvas);
    painter.setRenderHint(QPainter::Antialiasing);

    // 平面深色底：无圆形柔光、无网格
    QLinearGradient base(0, 0, 0, canvas.height());
    base.setColorAt(0.0, QColor("#0b121c"));
    base.setColorAt(1.0, QColor("#111b2a"));
    painter.fillRect(canvas.rect(), base);

    // 四角 HUD 角标
    const int m = 18;
    const int len = 22;
    painter.setPen(QPen(QColor(0, 112, 210, 140), 1.5));
    // 左上
    painter.drawLine(m, m, m + len, m);
    painter.drawLine(m, m, m, m + len);
    // 右上
    painter.drawLine(canvas.width() - m - len, m, canvas.width() - m, m);
    painter.drawLine(canvas.width() - m, m, canvas.width() - m, m + len);
    // 左下
    painter.drawLine(m, canvas.height() - m, m + len, canvas.height() - m);
    painter.drawLine(m, canvas.height() - m - len, m, canvas.height() - m);
    // 右下
    painter.drawLine(canvas.width() - m - len, canvas.height() - m, canvas.width() - m, canvas.height() - m);
    painter.drawLine(canvas.width() - m, canvas.height() - m - len, canvas.width() - m, canvas.height() - m);

    // 单条细分割线
    const int lineY = 318;
    const int lineMargin = 300;
    QLinearGradient lineGrad(lineMargin, 0, canvas.width() - lineMargin, 0);
    lineGrad.setColorAt(0.0, QColor(0, 112, 210, 0));
    lineGrad.setColorAt(0.5, QColor(0, 112, 210, 120));
    lineGrad.setColorAt(1.0, QColor(0, 112, 210, 0));
    painter.setPen(QPen(QBrush(lineGrad), 1.0));
    painter.drawLine(lineMargin, lineY, canvas.width() - lineMargin, lineY);
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
