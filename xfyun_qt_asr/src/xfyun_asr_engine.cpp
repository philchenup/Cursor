#include "xfyun_asr_engine.h"

#include "msp_cmn.h"
#include "msp_errors.h"
#include "qisr.h"

#include <QThread>
#include <cstring>

XfyunAsrEngine::XfyunAsrEngine(QObject* parent)
    : QObject(parent)
{
}

XfyunAsrEngine::~XfyunAsrEngine()
{
    cancelSession();
    logout();
}

bool XfyunAsrEngine::login(const QString& appId, const QString& workDir)
{
    if (appId.trimmed().isEmpty()) {
        emit errorOccurred(QStringLiteral("APPID 不能为空，请在讯飞开放平台申请。"));
        return false;
    }

    QString loginError;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_loggedIn) {
            return true;
        }

        m_appId = appId.trimmed();
        m_workDir = workDir.isEmpty() ? QStringLiteral(".") : workDir;

        const QByteArray loginParams =
            QStringLiteral("appid = %1, work_dir = %2").arg(m_appId, m_workDir).toUtf8();

        const int ret = MSPLogin(nullptr, nullptr, loginParams.constData());
        if (ret != MSP_SUCCESS) {
            loginError = QStringLiteral("MSPLogin 失败，错误码: %1").arg(ret);
        } else {
            m_loggedIn = true;
        }
    }
    if (!loginError.isEmpty()) {
        emit errorOccurred(loginError);
        return false;
    }
    return true;
}

void XfyunAsrEngine::logout()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_loggedIn) {
        return;
    }
    MSPLogout();
    m_loggedIn = false;
}

QString XfyunAsrEngine::buildSessionParams() const
{
    return QStringLiteral(
        "sub = iat, domain = iat, language = zh_cn, accent = mandarin, "
        "sample_rate = 16000, result_type = plain, result_encoding = utf8, "
        "vad_eos = 3000, vad_bos = 5000, ptt = 1");
}

bool XfyunAsrEngine::startSession()
{
    QString error;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_loggedIn) {
            error = QStringLiteral("尚未登录 MSC，请先调用 login。");
        } else if (m_sessionActive) {
            error = QStringLiteral("已有进行中的听写会话。");
        } else {
            int err = 0;
            const QByteArray params = buildSessionParams().toUtf8();
            const char* sid = QISRSessionBegin(nullptr, params.constData(), &err);
            if (err != MSP_SUCCESS || sid == nullptr) {
                error = QStringLiteral("QISRSessionBegin 失败，错误码: %1").arg(err);
            } else {
                m_sessionId = QString::fromUtf8(sid);
                m_sessionActive = true;
                m_firstAudio = true;
                m_accumulatedText.clear();
                return true;
            }
        }
    }
    if (!error.isEmpty()) {
        emit errorOccurred(error);
    }
    return false;
}

void XfyunAsrEngine::writeAudio(const QByteArray& pcm)
{
    if (pcm.isEmpty()) {
        return;
    }

    std::vector<PendingEvent> events;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_sessionActive || m_sessionId.isEmpty()) {
            return;
        }

        int epStatus = 0;
        int recogStatus = 0;
        const int audioStatus =
            m_firstAudio ? MSP_AUDIO_SAMPLE_FIRST : MSP_AUDIO_SAMPLE_CONTINUE;

        const QByteArray sid = m_sessionId.toUtf8();
        const int ret = QISRAudioWrite(sid.constData(),
                                       pcm.constData(),
                                       static_cast<unsigned int>(pcm.size()),
                                       audioStatus,
                                       &epStatus,
                                       &recogStatus);
        m_firstAudio = false;

        if (ret != MSP_SUCCESS) {
            events.push_back({PendingEvent::Type::Error,
                              QStringLiteral("QISRAudioWrite 失败，错误码: %1").arg(ret)});
            endSessionInternalLocked("write-error", events);
        } else {
            collectResultsLocked(false, events);
            Q_UNUSED(epStatus);
            Q_UNUSED(recogStatus);
        }
    }
    flushEvents(events);
}

void XfyunAsrEngine::finishSession()
{
    std::vector<PendingEvent> events;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_sessionActive || m_sessionId.isEmpty()) {
            return;
        }

        int epStatus = 0;
        int recogStatus = 0;
        const QByteArray sid = m_sessionId.toUtf8();
        const int audioStatus =
            m_firstAudio ? (MSP_AUDIO_SAMPLE_FIRST | MSP_AUDIO_SAMPLE_LAST)
                         : MSP_AUDIO_SAMPLE_LAST;

        const int ret = QISRAudioWrite(sid.constData(),
                                       nullptr,
                                       0,
                                       audioStatus,
                                       &epStatus,
                                       &recogStatus);
        if (ret != MSP_SUCCESS) {
            events.push_back(
                {PendingEvent::Type::Error,
                 QStringLiteral("结束音频写入失败，错误码: %1").arg(ret)});
            endSessionInternalLocked("finish-error", events);
        } else {
            collectResultsLocked(true, events);
            const QString finalText = m_accumulatedText;
            endSessionInternalLocked("normal-end", events);
            if (!finalText.isEmpty()) {
                events.push_back({PendingEvent::Type::Final, finalText});
            } else {
                events.push_back({PendingEvent::Type::Error,
                                  QStringLiteral("未识别到有效语音内容。")});
            }
        }
    }
    flushEvents(events);
}

void XfyunAsrEngine::cancelSession()
{
    std::vector<PendingEvent> events;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_sessionActive) {
            return;
        }
        endSessionInternalLocked("user-cancel", events);
    }
    flushEvents(events);
}

void XfyunAsrEngine::collectResultsLocked(bool waitForComplete,
                                          std::vector<PendingEvent>& events)
{
    if (m_sessionId.isEmpty()) {
        return;
    }

    const QByteArray sid = m_sessionId.toUtf8();
    const int maxLoops = waitForComplete ? 50 : 3;

    for (int i = 0; i < maxLoops; ++i) {
        int rsltStatus = 0;
        int err = 0;
        const char* result =
            QISRGetResult(sid.constData(), &rsltStatus, waitForComplete ? 200 : 0, &err);

        if (err != MSP_SUCCESS) {
            events.push_back({PendingEvent::Type::Error,
                              QStringLiteral("QISRGetResult 失败，错误码: %1").arg(err)});
            return;
        }

        if (result != nullptr && std::strlen(result) > 0) {
            m_accumulatedText += QString::fromUtf8(result);
            events.push_back({PendingEvent::Type::Partial, m_accumulatedText});
        }

        if (rsltStatus == MSP_REC_STATUS_COMPLETE) {
            return;
        }
        if (!waitForComplete) {
            return;
        }
        QThread::msleep(40);
    }
}

void XfyunAsrEngine::endSessionInternalLocked(const char* hints,
                                              std::vector<PendingEvent>& events)
{
    if (!m_sessionId.isEmpty()) {
        const QByteArray sid = m_sessionId.toUtf8();
        QISRSessionEnd(sid.constData(), hints);
    }
    m_sessionId.clear();
    m_sessionActive = false;
    m_firstAudio = true;
    events.push_back({PendingEvent::Type::SessionEnded, {}});
}

void XfyunAsrEngine::flushEvents(const std::vector<PendingEvent>& events)
{
    for (const PendingEvent& ev : events) {
        switch (ev.type) {
        case PendingEvent::Type::Partial:
            emit partialResult(ev.text);
            break;
        case PendingEvent::Type::Final:
            emit finalResult(ev.text);
            break;
        case PendingEvent::Type::Error:
            emit errorOccurred(ev.text);
            break;
        case PendingEvent::Type::SessionEnded:
            emit sessionEnded();
            break;
        }
    }
}
