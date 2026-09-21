#ifndef VOSK_ASR_ENGINE_H
#define VOSK_ASR_ENGINE_H

#include <QByteArray>
#include <QObject>
#include <QString>

struct VoskModel;
struct VoskRecognizer;

/**
 * Vosk 离线语音识别封装（中文模型）。
 * 输入 16kHz / 16bit / mono PCM，输出 partial / final 文本。
 */
class VoskAsrEngine : public QObject
{
    Q_OBJECT

public:
    explicit VoskAsrEngine(QObject* parent = nullptr);
    ~VoskAsrEngine() override;

    /** 加载 model 目录（含 am/ conf/ graph/ 等） */
    bool loadModel(const QString& modelPath);

    bool isReady() const { return m_model != nullptr; }
    QString modelPath() const { return m_modelPath; }

    /** 开始一轮新的识别（重置 recognizer） */
    bool beginUtterance();

    /** 写入 PCM 音频块 */
    void writeAudio(const QByteArray& pcm);

    /** 结束录音，刷出最终结果（通过 finalResult 信号） */
    void endUtterance();

    void cancel();

signals:
    void partialResult(const QString& text);
    void finalResult(const QString& text);
    void segmentResult(const QString& text);  // 录音过程中因静音切分出的完整句
    void errorOccurred(const QString& message);

private:
    static QString extractJsonField(const QString& json, const QString& field);

    QString m_modelPath;
    VoskModel* m_model = nullptr;
    VoskRecognizer* m_recognizer = nullptr;
    bool m_inUtterance = false;
    QString m_lastPartial;
};

#endif  // VOSK_ASR_ENGINE_H
