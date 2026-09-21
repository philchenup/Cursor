#include "audio_capture.h"

#include <QDebug>

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#include <QAudioDevice>
#include <QAudioSource>
#include <QMediaDevices>
#else
#include <QAudioDeviceInfo>
#include <QAudioInput>
#endif

AudioCapture::AudioCapture(QObject* parent)
    : QObject(parent)
{
}

AudioCapture::~AudioCapture()
{
    stop();
}

QAudioFormat AudioCapture::preferredFormat()
{
    QAudioFormat format;
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    format.setSampleRate(16000);
    format.setChannelCount(1);
    format.setSampleFormat(QAudioFormat::Int16);
#else
    format.setSampleRate(16000);
    format.setChannelCount(1);
    format.setSampleSize(16);
    format.setCodec(QStringLiteral("audio/pcm"));
    format.setByteOrder(QAudioFormat::LittleEndian);
    format.setSampleType(QAudioFormat::SignedInt);
#endif
    return format;
}

bool AudioCapture::start()
{
    if (m_capturing) {
        return true;
    }

    const QAudioFormat format = preferredFormat();

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    const QAudioDevice device = QMediaDevices::defaultAudioInput();
    if (device.isNull()) {
        emit errorOccurred(QStringLiteral("未找到可用的麦克风设备。"));
        return false;
    }
    if (!device.isFormatSupported(format)) {
        emit errorOccurred(QStringLiteral("麦克风不支持 16kHz/16bit/mono PCM。"));
        return false;
    }

    m_audioSource = std::make_unique<QAudioSource>(device, format, this);
    m_ioDevice = m_audioSource->start();
    if (!m_ioDevice) {
        emit errorOccurred(QStringLiteral("启动麦克风采集失败。"));
        m_audioSource.reset();
        return false;
    }
#else
    const QAudioDeviceInfo device = QAudioDeviceInfo::defaultInputDevice();
    if (device.isNull()) {
        emit errorOccurred(QStringLiteral("未找到可用的麦克风设备。"));
        return false;
    }
    if (!device.isFormatSupported(format)) {
        emit errorOccurred(QStringLiteral("麦克风不支持 16kHz/16bit/mono PCM。"));
        return false;
    }

    m_audioInput = std::make_unique<QAudioInput>(device, format, this);
    m_ioDevice = m_audioInput->start();
    if (!m_ioDevice) {
        emit errorOccurred(QStringLiteral("启动麦克风采集失败。"));
        m_audioInput.reset();
        return false;
    }
#endif

    connect(m_ioDevice, &QIODevice::readyRead, this, &AudioCapture::onReadyRead);
    m_capturing = true;
    return true;
}

void AudioCapture::stop()
{
    if (!m_capturing) {
        return;
    }

    if (m_ioDevice) {
        disconnect(m_ioDevice, &QIODevice::readyRead, this, &AudioCapture::onReadyRead);
        m_ioDevice = nullptr;
    }

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    if (m_audioSource) {
        m_audioSource->stop();
        m_audioSource.reset();
    }
#else
    if (m_audioInput) {
        m_audioInput->stop();
        m_audioInput.reset();
    }
#endif

    m_capturing = false;
}

void AudioCapture::onReadyRead()
{
    if (!m_ioDevice) {
        return;
    }
    const QByteArray data = m_ioDevice->readAll();
    if (!data.isEmpty()) {
        emit pcmReady(data);
    }
}
