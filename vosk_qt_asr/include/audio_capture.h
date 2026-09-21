#ifndef AUDIO_CAPTURE_H
#define AUDIO_CAPTURE_H

#include <QAudioFormat>
#include <QByteArray>
#include <QIODevice>
#include <QObject>
#include <memory>

class QAudioSource;
class QAudioInput;

/**
 * 麦克风采集：16kHz / 16bit / 单声道 PCM，供 Vosk 识别使用。
 * 兼容 Qt5(QAudioInput) 与 Qt6(QAudioSource)。
 */
class AudioCapture : public QObject
{
    Q_OBJECT

public:
    explicit AudioCapture(QObject* parent = nullptr);
    ~AudioCapture() override;

    bool start();
    void stop();
    bool isCapturing() const { return m_capturing; }

    static QAudioFormat preferredFormat();

signals:
    void pcmReady(const QByteArray& pcm);
    void errorOccurred(const QString& message);

private slots:
    void onReadyRead();

private:
    bool m_capturing = false;
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    std::unique_ptr<QAudioSource> m_audioSource;
#else
    std::unique_ptr<QAudioInput> m_audioInput;
#endif
    QIODevice* m_ioDevice = nullptr;
};

#endif  // AUDIO_CAPTURE_H
