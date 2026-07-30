#include "main_window.h"

#include "audio_capture.h"
#include "vosk_asr_engine.h"

#include <QFont>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

MainWindow::MainWindow(const QString& modelPath, QWidget* parent)
    : QMainWindow(parent)
    , m_modelPath(modelPath)
{
    setWindowTitle(QStringLiteral("Vosk 中文语音识别"));
    setMinimumSize(480, 560);
    resize(560, 640);

    m_capture = new AudioCapture(this);
    m_engine = new VoskAsrEngine(this);

    setupUi();

    connect(m_capture, &AudioCapture::pcmReady, this, &MainWindow::onPcmReady);
    connect(m_capture, &AudioCapture::errorOccurred, this, &MainWindow::onCaptureError);
    connect(m_engine, &VoskAsrEngine::partialResult, this, &MainWindow::onPartialResult);
    connect(m_engine, &VoskAsrEngine::segmentResult, this, &MainWindow::onSegmentResult);
    connect(m_engine, &VoskAsrEngine::finalResult, this, &MainWindow::onFinalResult);
    connect(m_engine, &VoskAsrEngine::errorOccurred, this, &MainWindow::onAsrError);

    if (!m_engine->loadModel(m_modelPath)) {
        m_speakerButton->setEnabled(false);
        updateStatus(QStringLiteral("模型加载失败，请检查 model 目录。"), true);
    } else {
        updateStatus(QStringLiteral("模型已就绪：%1").arg(m_modelPath));
    }
}

MainWindow::~MainWindow()
{
    if (m_recording) {
        m_capture->stop();
        m_engine->cancel();
    }
}

void MainWindow::setupUi()
{
    auto* central = new QWidget(this);
    setCentralWidget(central);

    auto* root = new QVBoxLayout(central);
    root->setContentsMargins(24, 24, 24, 24);
    root->setSpacing(16);

    m_hintLabel = new QLabel(
        QStringLiteral("点击下方音响按钮开始录音，再次点击结束录音并显示识别结果。"),
        central);
    m_hintLabel->setWordWrap(true);
    m_hintLabel->setAlignment(Qt::AlignCenter);
    root->addWidget(m_hintLabel);

    root->addStretch(1);

    // 圆形音响按钮
    m_speakerButton = new QPushButton(central);
    m_speakerButton->setObjectName(QStringLiteral("speakerButton"));
    m_speakerButton->setCheckable(true);
    m_speakerButton->setFixedSize(160, 160);
    m_speakerButton->setCursor(Qt::PointingHandCursor);
    m_speakerButton->setToolTip(QStringLiteral("点击开始 / 结束录音"));
    m_speakerButton->setText(QStringLiteral("开始录音"));

    auto* buttonRow = new QHBoxLayout();
    buttonRow->addStretch(1);
    buttonRow->addWidget(m_speakerButton);
    buttonRow->addStretch(1);
    root->addLayout(buttonRow);

    root->addStretch(1);

    m_statusLabel = new QLabel(central);
    m_statusLabel->setObjectName(QStringLiteral("statusLabel"));
    m_statusLabel->setAlignment(Qt::AlignCenter);
    m_statusLabel->setWordWrap(true);
    root->addWidget(m_statusLabel);

    auto* resultTitle = new QLabel(QStringLiteral("识别结果"), central);
    QFont titleFont = resultTitle->font();
    titleFont.setBold(true);
    resultTitle->setFont(titleFont);
    root->addWidget(resultTitle);

    m_resultEdit = new QPlainTextEdit(central);
    m_resultEdit->setReadOnly(true);
    m_resultEdit->setPlaceholderText(QStringLiteral("识别文字将显示在这里…"));
    m_resultEdit->setMinimumHeight(180);
    root->addWidget(m_resultEdit, 1);

    m_clearButton = new QPushButton(QStringLiteral("清空结果"), central);
    root->addWidget(m_clearButton);

    // 样式：未录音 / 录音中
    setStyleSheet(QStringLiteral(
        "QMainWindow { background: #f3f6f9; }"
        "QLabel#statusLabel { color: #334155; font-size: 13px; }"
        "QPushButton#speakerButton {"
        "  border-radius: 80px;"
        "  background: qlineargradient(x1:0, y1:0, x2:0, y2:1,"
        "                              stop:0 #3b82f6, stop:1 #1d4ed8);"
        "  color: white;"
        "  font-size: 16px;"
        "  font-weight: 600;"
        "  border: 3px solid #93c5fd;"
        "}"
        "QPushButton#speakerButton:hover {"
        "  background: qlineargradient(x1:0, y1:0, x2:0, y2:1,"
        "                              stop:0 #60a5fa, stop:1 #2563eb);"
        "}"
        "QPushButton#speakerButton:checked {"
        "  background: qlineargradient(x1:0, y1:0, x2:0, y2:1,"
        "                              stop:0 #ef4444, stop:1 #b91c1c);"
        "  border: 3px solid #fca5a5;"
        "}"
        "QPlainTextEdit {"
        "  background: white;"
        "  border: 1px solid #cbd5e1;"
        "  border-radius: 8px;"
        "  padding: 8px;"
        "  font-size: 15px;"
        "}"));

    connect(m_speakerButton, &QPushButton::clicked, this, &MainWindow::onSpeakerClicked);
    connect(m_clearButton, &QPushButton::clicked, this, &MainWindow::onClearClicked);

    setRecording(false);
}

void MainWindow::setRecording(bool recording)
{
    m_recording = recording;
    m_speakerButton->setChecked(recording);
    if (recording) {
        m_speakerButton->setText(QStringLiteral("结束录音"));
        updateStatus(QStringLiteral("正在录音，请说话… 再次点击结束。"));
    } else {
        m_speakerButton->setText(QStringLiteral("开始录音"));
    }
}

void MainWindow::updateStatus(const QString& status, bool isError)
{
    m_statusLabel->setText(status);
    m_statusLabel->setStyleSheet(isError
                                     ? QStringLiteral("color: #b91c1c; font-size: 13px;")
                                     : QStringLiteral("color: #334155; font-size: 13px;"));
}

void MainWindow::appendResultLine(const QString& text)
{
    if (text.trimmed().isEmpty()) {
        return;
    }
    m_resultEdit->appendPlainText(text);
}

void MainWindow::onSpeakerClicked()
{
    if (!m_engine->isReady()) {
        QMessageBox::warning(this,
                             QStringLiteral("模型未就绪"),
                             QStringLiteral("请将中文模型放到 model 目录后重新启动。"));
        m_speakerButton->setChecked(false);
        return;
    }

    if (m_recording) {
        // 结束录音 → 输出最终识别结果
        m_capture->stop();
        m_engine->endUtterance();
        setRecording(false);
        return;
    }

    if (!m_engine->beginUtterance()) {
        m_speakerButton->setChecked(false);
        return;
    }
    if (!m_capture->start()) {
        m_engine->cancel();
        m_speakerButton->setChecked(false);
        return;
    }
    setRecording(true);
}

void MainWindow::onPcmReady(const QByteArray& pcm)
{
    if (m_recording) {
        m_engine->writeAudio(pcm);
    }
}

void MainWindow::onPartialResult(const QString& text)
{
    updateStatus(QStringLiteral("识别中: %1").arg(text));
}

void MainWindow::onSegmentResult(const QString& text)
{
    appendResultLine(text);
    updateStatus(QStringLiteral("已识别片段，继续说或点击结束…"));
}

void MainWindow::onFinalResult(const QString& text)
{
    if (text.trimmed().isEmpty()) {
        updateStatus(QStringLiteral("录音结束：未识别到有效语音。"));
        // 也打印到结果区，方便确认流程走完
        m_resultEdit->appendPlainText(QStringLiteral("（未识别到有效语音）"));
        return;
    }

    appendResultLine(text);
    updateStatus(QStringLiteral("录音结束，识别完成。"));

    // 控制台也打印一份，便于调试 / Visual Studio 输出窗口查看
    qInfo("ASR result: %s", qUtf8Printable(text));
}

void MainWindow::onAsrError(const QString& message)
{
    updateStatus(message, true);
    if (m_recording) {
        m_capture->stop();
        m_engine->cancel();
        setRecording(false);
    }
}

void MainWindow::onCaptureError(const QString& message)
{
    updateStatus(message, true);
    if (m_recording) {
        m_capture->stop();
        m_engine->cancel();
        setRecording(false);
    }
    QMessageBox::warning(this, QStringLiteral("录音错误"), message);
}

void MainWindow::onClearClicked()
{
    m_resultEdit->clear();
    updateStatus(QStringLiteral("结果已清空。点击音响按钮开始录音。"));
}
