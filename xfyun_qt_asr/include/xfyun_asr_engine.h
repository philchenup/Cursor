#ifndef XFYUN_ASR_ENGINE_H
#define XFYUN_ASR_ENGINE_H

#include <QByteArray>
#include <QObject>
#include <QString>
#include <atomic>
#include <mutex>
#include <vector>

struct XfyunPendingEvent;

/**
 * 科大讯飞语音听写引擎封装。
 * 在工作线程中写入音频并轮询结果，通过信号把文本回传到 UI 线程。
 */
class XfyunAsrEngine : public QObject
{
    Q_OBJECT

public:
    explicit XfyunAsrEngine(QObject* parent = nullptr);
    ~XfyunAsrEngine() override;

    /** appId 来自讯飞开放平台控制台。 */
    bool login(const QString& appId, const QString& workDir = QStringLiteral("."));
    void logout();

    bool isLoggedIn() const { return m_loggedIn; }
    bool isSessionActive() const { return m_sessionActive; }

public slots:
    /** 开启听写会话并准备接收 PCM。 */
    bool startSession();
    /** 写入 16kHz/16bit/mono PCM 数据块。 */
    void writeAudio(const QByteArray& pcm);
    /** 标记音频结束，拉取最终结果并关闭会话。 */
    void finishSession();
    /** 取消当前会话，不回传结果。 */
    void cancelSession();

signals:
    void partialResult(const QString& text);
    void finalResult(const QString& text);
    void errorOccurred(const QString& message);
    void sessionEnded();

private:
    struct PendingEvent {
        enum class Type { Partial, Final, Error, SessionEnded };
        Type type;
        QString text;
    };

    QString buildSessionParams() const;
    void collectResultsLocked(bool waitForComplete, std::vector<PendingEvent>& events);
    void endSessionInternalLocked(const char* hints, std::vector<PendingEvent>& events);
    void flushEvents(const std::vector<PendingEvent>& events);

    mutable std::mutex m_mutex;
    QString m_appId;
    QString m_workDir;
    QString m_sessionId;
    std::atomic_bool m_loggedIn{false};
    std::atomic_bool m_sessionActive{false};
    std::atomic_bool m_firstAudio{true};
    QString m_accumulatedText;
};

#endif  // XFYUN_ASR_ENGINE_H
