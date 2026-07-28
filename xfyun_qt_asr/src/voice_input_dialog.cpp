#include "voice_input_dialog.h"

#include "audio_capture.h"
#include "xfyun_asr_engine.h"

#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTextCursor>
#include <QVBoxLayout>

VoiceInputDialog::VoiceInputDialog(const QString& xfyunAppId, QWidget* parent)
    : QDialog(parent)
    , m_appId(xfyunAppId)
{
    setWindowTitle(QStringLiteral("语音 / 文字输入"));
    setMinimumSize(520, 420);
    resize(640, 480);

    m_capture = new AudioCapture(this);
    m_engine = new XfyunAsrEngine(this);

    setupUi();

    connect(m_capture, &AudioCapture::pcmReady, this, &VoiceInputDialog::onPcmReady);
    connect(m_capture, &AudioCapture::errorOccurred, this, &VoiceInputDialog::onCaptureError);
    connect(m_engine, &XfyunAsrEngine::partialResult, this, &VoiceInputDialog::onPartialResult);
    connect(m_engine, &XfyunAsrEngine::finalResult, this, &VoiceInputDialog::onFinalResult);
    connect(m_engine, &XfyunAsrEngine::errorOccurred, this, &VoiceInputDialog::onAsrError);

    if (!m_appId.isEmpty()) {
        m_appIdEdit->setText(m_appId);
        if (m_engine->login(m_appId)) {
            updateStatus(QStringLiteral("已登录讯飞 MSC，可打字或语音输入。"));
        }
    } else {
        updateStatus(QStringLiteral("请填写 APPID 后开始语音输入（也可直接打字）。"));
    }
}

VoiceInputDialog::~VoiceInputDialog()
{
    if (m_listening) {
        m_capture->stop();
        m_engine->cancelSession();
    }
}

void VoiceInputDialog::setupUi()
{
    auto* root = new QVBoxLayout(this);

    auto* tip = new QLabel(
        QStringLiteral("可在下方直接打字；也可点击「开始说话」进行讯飞语音识别，识别结果会写入文本框。"),
        this);
    tip->setWordWrap(true);
    root->addWidget(tip);

    auto* form = new QFormLayout();
    m_appIdEdit = new QLineEdit(this);
    m_appIdEdit->setPlaceholderText(QStringLiteral("讯飞开放平台 APPID"));
    form->addRow(QStringLiteral("APPID"), m_appIdEdit);
    root->addLayout(form);

    m_editor = new QPlainTextEdit(this);
    m_editor->setPlaceholderText(QStringLiteral("在此输入文字，或使用语音输入…"));
    m_editor->setTabChangesFocus(true);
    root->addWidget(m_editor, 1);

    m_statusLabel = new QLabel(this);
    m_statusLabel->setObjectName(QStringLiteral("statusLabel"));
    root->addWidget(m_statusLabel);

    auto* voiceRow = new QHBoxLayout();
    m_voiceButton = new QPushButton(QStringLiteral("开始说话"), this);
    m_voiceButton->setCheckable(true);
    m_voiceButton->setMinimumHeight(36);
    m_clearButton = new QPushButton(QStringLiteral("清空"), this);
    voiceRow->addWidget(m_voiceButton, 1);
    voiceRow->addWidget(m_clearButton);
    root->addLayout(voiceRow);

    auto* buttonRow = new QHBoxLayout();
    buttonRow->addStretch(1);
    m_okButton = new QPushButton(QStringLiteral("确定"), this);
    m_cancelButton = new QPushButton(QStringLiteral("取消"), this);
    m_okButton->setDefault(true);
    buttonRow->addWidget(m_okButton);
    buttonRow->addWidget(m_cancelButton);
    root->addLayout(buttonRow);

    connect(m_voiceButton, &QPushButton::clicked, this, &VoiceInputDialog::onToggleVoice);
    connect(m_clearButton, &QPushButton::clicked, this, &VoiceInputDialog::onClearClicked);
    connect(m_okButton, &QPushButton::clicked, this, &VoiceInputDialog::onAcceptClicked);
    connect(m_cancelButton, &QPushButton::clicked, this, &QDialog::reject);
}

QString VoiceInputDialog::text() const
{
    return m_editor->toPlainText();
}

void VoiceInputDialog::setText(const QString& text)
{
    m_editor->setPlainText(text);
}

void VoiceInputDialog::onToggleVoice()
{
    if (m_listening) {
        // 停止录音并取最终结果
        m_capture->stop();
        m_engine->finishSession();
        setListening(false);
        return;
    }

    const QString appId = m_appIdEdit->text().trimmed();
    if (appId.isEmpty()) {
        QMessageBox::warning(this,
                             QStringLiteral("缺少 APPID"),
                             QStringLiteral("请填写讯飞开放平台 APPID。"));
        m_voiceButton->setChecked(false);
        return;
    }

    if (!m_engine->isLoggedIn() || appId != m_appId) {
        m_appId = appId;
        if (!m_engine->login(m_appId)) {
            m_voiceButton->setChecked(false);
            return;
        }
    }

    if (!m_engine->startSession()) {
        m_voiceButton->setChecked(false);
        return;
    }

    m_voiceAnchorPos = m_editor->textCursor().position();
    if (!m_capture->start()) {
        m_engine->cancelSession();
        m_voiceButton->setChecked(false);
        return;
    }

    setListening(true);
    updateStatus(QStringLiteral("正在聆听… 再次点击按钮结束识别。"));
}

void VoiceInputDialog::onPcmReady(const QByteArray& pcm)
{
    if (m_listening) {
        m_engine->writeAudio(pcm);
    }
}

void VoiceInputDialog::onPartialResult(const QString& text)
{
    // 用本轮语音的中间结果替换锚点后的内容，避免重复追加。
    QTextCursor cursor = m_editor->textCursor();
    cursor.beginEditBlock();
    cursor.setPosition(m_voiceAnchorPos);
    cursor.movePosition(QTextCursor::End, QTextCursor::KeepAnchor);
    cursor.insertText(text);
    cursor.endEditBlock();
    m_editor->setTextCursor(cursor);
    updateStatus(QStringLiteral("识别中…"));
}

void VoiceInputDialog::onFinalResult(const QString& text)
{
    appendRecognizedText(text);
    updateStatus(QStringLiteral("识别完成。可继续打字或再次语音输入。"));
}

void VoiceInputDialog::appendRecognizedText(const QString& text)
{
    if (text.isEmpty()) {
        return;
    }
    QTextCursor cursor = m_editor->textCursor();
    if (m_voiceAnchorPos >= 0) {
        cursor.beginEditBlock();
        cursor.setPosition(m_voiceAnchorPos);
        cursor.movePosition(QTextCursor::End, QTextCursor::KeepAnchor);
        cursor.insertText(text);
        cursor.endEditBlock();
    } else {
        cursor.movePosition(QTextCursor::End);
        cursor.insertText(text);
    }
    m_editor->setTextCursor(cursor);
    m_voiceAnchorPos = -1;
}

void VoiceInputDialog::onAsrError(const QString& message)
{
    if (m_listening) {
        m_capture->stop();
        setListening(false);
    }
    updateStatus(message, true);
}

void VoiceInputDialog::onCaptureError(const QString& message)
{
    if (m_listening) {
        m_engine->cancelSession();
        setListening(false);
    }
    updateStatus(message, true);
}

void VoiceInputDialog::onAcceptClicked()
{
    if (m_listening) {
        m_capture->stop();
        m_engine->finishSession();
        setListening(false);
    }
    accept();
}

void VoiceInputDialog::onClearClicked()
{
    m_editor->clear();
    m_voiceAnchorPos = -1;
    updateStatus(QStringLiteral("已清空文本。"));
}

void VoiceInputDialog::setListening(bool listening)
{
    m_listening = listening;
    m_voiceButton->setChecked(listening);
    m_voiceButton->setText(listening ? QStringLiteral("结束说话")
                                     : QStringLiteral("开始说话"));
    m_appIdEdit->setEnabled(!listening);
}

void VoiceInputDialog::updateStatus(const QString& status, bool isError)
{
    m_statusLabel->setText(status);
    m_statusLabel->setStyleSheet(isError ? QStringLiteral("color: #b00020;")
                                         : QStringLiteral("color: #33691e;"));
}
