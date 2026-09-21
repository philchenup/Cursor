#include "vosk_asr_engine.h"

#include "vosk_api.h"

#include <QFileInfo>

VoskAsrEngine::VoskAsrEngine(QObject* parent)
    : QObject(parent)
{
    vosk_set_log_level(-1);
}

VoskAsrEngine::~VoskAsrEngine()
{
    cancel();
    if (m_recognizer) {
        vosk_recognizer_free(m_recognizer);
        m_recognizer = nullptr;
    }
    if (m_model) {
        vosk_model_free(m_model);
        m_model = nullptr;
    }
}

bool VoskAsrEngine::loadModel(const QString& modelPath)
{
    if (m_model) {
        cancel();
        if (m_recognizer) {
            vosk_recognizer_free(m_recognizer);
            m_recognizer = nullptr;
        }
        vosk_model_free(m_model);
        m_model = nullptr;
    }

    const QFileInfo info(modelPath);
    if (!info.exists() || !info.isDir()) {
        emit errorOccurred(QStringLiteral("模型目录不存在: %1").arg(modelPath));
        return false;
    }

    m_model = vosk_model_new(modelPath.toUtf8().constData());
    if (!m_model) {
        emit errorOccurred(QStringLiteral("加载 Vosk 模型失败: %1").arg(modelPath));
        return false;
    }

    m_recognizer = vosk_recognizer_new(m_model, 16000.0f);
    if (!m_recognizer) {
        vosk_model_free(m_model);
        m_model = nullptr;
        emit errorOccurred(QStringLiteral("创建 Vosk 识别器失败。"));
        return false;
    }

    vosk_recognizer_set_words(m_recognizer, 0);
    m_modelPath = modelPath;
    return true;
}

bool VoskAsrEngine::beginUtterance()
{
    if (!m_model || !m_recognizer) {
        emit errorOccurred(QStringLiteral("模型未加载，无法开始识别。"));
        return false;
    }

    vosk_recognizer_reset(m_recognizer);
    m_inUtterance = true;
    m_lastPartial.clear();
    return true;
}

void VoskAsrEngine::writeAudio(const QByteArray& pcm)
{
    if (!m_inUtterance || !m_recognizer || pcm.isEmpty()) {
        return;
    }

    const int accepted = vosk_recognizer_accept_waveform(
        m_recognizer, pcm.constData(), static_cast<int>(pcm.size()));

    if (accepted == 1) {
        const char* json = vosk_recognizer_result(m_recognizer);
        const QString text = extractJsonField(QString::fromUtf8(json ? json : ""),
                                              QStringLiteral("text"));
        if (!text.isEmpty()) {
            emit segmentResult(text);
        }
        m_lastPartial.clear();
    } else if (accepted == 0) {
        const char* json = vosk_recognizer_partial_result(m_recognizer);
        const QString partial = extractJsonField(QString::fromUtf8(json ? json : ""),
                                                 QStringLiteral("partial"));
        if (!partial.isEmpty() && partial != m_lastPartial) {
            m_lastPartial = partial;
            emit partialResult(partial);
        }
    } else {
        emit errorOccurred(QStringLiteral("Vosk 处理音频时发生错误。"));
    }
}

void VoskAsrEngine::endUtterance()
{
    if (!m_inUtterance || !m_recognizer) {
        return;
    }

    m_inUtterance = false;
    const char* json = vosk_recognizer_final_result(m_recognizer);
    const QString text = extractJsonField(QString::fromUtf8(json ? json : ""),
                                          QStringLiteral("text"));
    emit finalResult(text);
    m_lastPartial.clear();
}

void VoskAsrEngine::cancel()
{
    if (!m_inUtterance) {
        return;
    }
    m_inUtterance = false;
    m_lastPartial.clear();
    if (m_recognizer) {
        vosk_recognizer_reset(m_recognizer);
    }
}

QString VoskAsrEngine::extractJsonField(const QString& json, const QString& field)
{
    const QString key = QStringLiteral("\"%1\"").arg(field);
    int pos = json.indexOf(key);
    if (pos < 0) {
        return {};
    }
    pos = json.indexOf(QLatin1Char(':'), pos + key.size());
    if (pos < 0) {
        return {};
    }
    pos = json.indexOf(QLatin1Char('"'), pos + 1);
    if (pos < 0) {
        return {};
    }

    const int start = pos + 1;
    QString out;
    out.reserve(json.size() - start);
    for (int i = start; i < json.size(); ++i) {
        const QChar ch = json.at(i);
        if (ch == QLatin1Char('\\') && i + 1 < json.size()) {
            out.append(json.at(i + 1));
            ++i;
            continue;
        }
        if (ch == QLatin1Char('"')) {
            break;
        }
        out.append(ch);
    }
    return out.trimmed();
}
