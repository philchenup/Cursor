#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QMainWindow>
#include <QString>

class QLabel;
class QPlainTextEdit;
class QPushButton;
class AudioCapture;
class VoskAsrEngine;

/**
 * 主窗口：音响按钮切换录音，结束后显示识别文字。
 * 第一次点击开始录音，再点一次结束录音并输出结果。
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(const QString& modelPath, QWidget* parent = nullptr);
    ~MainWindow() override;

private slots:
    void onSpeakerClicked();
    void onPcmReady(const QByteArray& pcm);
    void onPartialResult(const QString& text);
    void onSegmentResult(const QString& text);
    void onFinalResult(const QString& text);
    void onAsrError(const QString& message);
    void onCaptureError(const QString& message);
    void onClearClicked();

private:
    void setupUi();
    void setRecording(bool recording);
    void updateStatus(const QString& status, bool isError = false);
    void appendResultLine(const QString& text);

    QString m_modelPath;

    QPushButton* m_speakerButton = nullptr;
    QPushButton* m_clearButton = nullptr;
    QLabel* m_statusLabel = nullptr;
    QLabel* m_hintLabel = nullptr;
    QPlainTextEdit* m_resultEdit = nullptr;

    AudioCapture* m_capture = nullptr;
    VoskAsrEngine* m_engine = nullptr;

    bool m_recording = false;
};

#endif  // MAIN_WINDOW_H
