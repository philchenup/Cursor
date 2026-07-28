#ifndef VOICE_INPUT_DIALOG_H
#define VOICE_INPUT_DIALOG_H

#include <QDialog>
#include <QString>

class QLabel;
class QPlainTextEdit;
class QPushButton;
class QLineEdit;
class AudioCapture;
class XfyunAsrEngine;

/**
 * 支持「打字输入 + 语音听写」的 Qt 对话框。
 * - 文本框可直接键盘输入/编辑
 * - 按住/点击「开始说话」进行讯飞语音识别，结果追加到文本框
 */
class VoiceInputDialog : public QDialog
{
    Q_OBJECT

public:
    explicit VoiceInputDialog(const QString& xfyunAppId,
                              QWidget* parent = nullptr);
    ~VoiceInputDialog() override;

    QString text() const;
    void setText(const QString& text);

private slots:
    void onToggleVoice();
    void onPcmReady(const QByteArray& pcm);
    void onPartialResult(const QString& text);
    void onFinalResult(const QString& text);
    void onAsrError(const QString& message);
    void onCaptureError(const QString& message);
    void onAcceptClicked();
    void onClearClicked();

private:
    void setupUi();
    void setListening(bool listening);
    void appendRecognizedText(const QString& text);
    void updateStatus(const QString& status, bool isError = false);

    QString m_appId;
    QPlainTextEdit* m_editor = nullptr;
    QPushButton* m_voiceButton = nullptr;
    QPushButton* m_clearButton = nullptr;
    QPushButton* m_okButton = nullptr;
    QPushButton* m_cancelButton = nullptr;
    QLabel* m_statusLabel = nullptr;
    QLineEdit* m_appIdEdit = nullptr;

    AudioCapture* m_capture = nullptr;
    XfyunAsrEngine* m_engine = nullptr;

    bool m_listening = false;
    int m_voiceAnchorPos = -1;  // 本轮语音写入前的光标位置，用于增量替换中间结果
};

#endif  // VOICE_INPUT_DIALOG_H
