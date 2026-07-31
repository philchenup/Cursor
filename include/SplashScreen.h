#ifndef SPLASHSCREEN_H
#define SPLASHSCREEN_H

#include <QSplashScreen>
#include <QProgressBar>
#include <QLabel>
#include <QString>

/**
 * @brief 品牌启动画面：适配 1490x468 透明 PNG Logo，含阶段性进度反馈。
 *
 * 布局（1000 x 560）：
 *   ┌────────────────────────────────────┐
 *   │          氛围渐变背景               │
 *   │                                    │
 *   │         [ Logo 720 宽 ]            │
 *   │           Version x.x.x            │
 *   │                                    │
 *   │         状态文案 / 进度条           │
 *   └────────────────────────────────────┘
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

    static constexpr int kWidth  = 1000;
    static constexpr int kHeight = 560;
    static constexpr int kLogoMaxWidth = 720;

    QLabel* m_logoLabel = nullptr;
    QLabel* m_versionLabel = nullptr;
    QLabel* m_statusLabel = nullptr;
    QLabel* m_percentLabel = nullptr;
    QProgressBar* m_progress = nullptr;
    QString m_version;
};

#endif // SPLASHSCREEN_H
