#ifndef SPLASHSCREEN_H
#define SPLASHSCREEN_H

#include <QSplashScreen>
#include <QProgressBar>
#include <QLabel>
#include <QString>

/**
 * @brief HWI / 哈焊 启动画面 —— 适配 CAM+HWI 双色标（1490×468 透明 PNG）。
 *
 * 品牌色：蓝 #0070D2 · 橙 #F39C12 · 红 #DC2626（Logo 内「哈焊」）
 *
 * 布局（1080 × 540）：
 *   ┌──────────────────────────────────────┐
 *   │   左暖橙柔光 · 右冷蓝柔光 · 深炭底     │
 *   │                                      │
 *   │      [ CAM 标 + HWI 椭圆 Logo ]       │
 *   │           蓝→橙 装饰线                │
 *   │            Version x.x.x             │
 *   │                                      │
 *   │      状态文案              nn%        │
 *   │      ████████░░░░  蓝→橙进度条        │
 *   └──────────────────────────────────────┘
 */
class SplashScreen : public QSplashScreen
{
    Q_OBJECT

public:
    explicit SplashScreen(const QString& logoPath = QStringLiteral("./icon/preview.png"),
                          const QString& version = QStringLiteral("1.0.0"),
                          QWidget* parent = nullptr);

    /** 阻塞显示启动画面，进度从 0 走到 100 后返回。 */
    void run();

    /** 外部驱动进度（0–100），同时更新状态文案。 */
    void setProgress(int value, const QString& status = QString());

private:
    void buildBackground(QPixmap& canvas) const;
    void layoutWidgets();
    QString statusForProgress(int value) const;

    // 画布略宽，容纳 1490×468 宽幅 Logo
    static constexpr int kWidth  = 1080;
    static constexpr int kHeight = 540;
    static constexpr int kLogoMaxWidth = 860;

    QLabel* m_logoLabel = nullptr;
    QLabel* m_versionLabel = nullptr;
    QLabel* m_statusLabel = nullptr;
    QLabel* m_percentLabel = nullptr;
    QProgressBar* m_progress = nullptr;
    QString m_version;
};

#endif // SPLASHSCREEN_H
