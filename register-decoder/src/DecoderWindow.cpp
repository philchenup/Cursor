#include "DecoderWindow.h"

#include "LicenseCode.h"

#include <QApplication>
#include <QClipboard>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QVBoxLayout>

DecoderWindow::DecoderWindow(QWidget* parent)
    : QWidget(parent)
{
    setWindowTitle(QStringLiteral("注册码解码"));
    setMinimumSize(560, 420);

    auto* title = new QLabel(QStringLiteral("注册码解码"), this);
    title->setObjectName(QStringLiteral("title"));
    auto* subtitle = new QLabel(QStringLiteral("输入软件显示的机器序列号，生成对应授权码。"), this);
    subtitle->setObjectName(QStringLiteral("subtitle"));
    subtitle->setWordWrap(true);

    auto* serialCaption = new QLabel(QStringLiteral("机器序列号"), this);
    m_serialEdit = new QPlainTextEdit(this);
    m_serialEdit->setObjectName(QStringLiteral("serial"));
    m_serialEdit->setPlaceholderText(QStringLiteral("粘贴软件中显示的机器序列号"));
    m_serialEdit->setTabChangesFocus(true);
    m_serialEdit->setFixedHeight(96);

    auto* codeCaption = new QLabel(QStringLiteral("授权码"), this);
    m_codeEdit = new QLineEdit(this);
    m_codeEdit->setObjectName(QStringLiteral("code"));
    m_codeEdit->setReadOnly(true);
    m_codeEdit->setPlaceholderText(QStringLiteral("生成后显示在这里"));

    auto* generateButton = new QPushButton(QStringLiteral("生成授权码"), this);
    generateButton->setObjectName(QStringLiteral("primary"));
    generateButton->setDefault(true);
    generateButton->setCursor(Qt::PointingHandCursor);

    auto* copyButton = new QPushButton(QStringLiteral("复制"), this);
    copyButton->setObjectName(QStringLiteral("ghost"));
    copyButton->setCursor(Qt::PointingHandCursor);

    auto* clearButton = new QPushButton(QStringLiteral("清空"), this);
    clearButton->setObjectName(QStringLiteral("ghost"));
    clearButton->setCursor(Qt::PointingHandCursor);

    m_statusLabel = new QLabel(QStringLiteral("等待输入"), this);
    m_statusLabel->setObjectName(QStringLiteral("status"));

    auto* codeRow = new QHBoxLayout();
    codeRow->setSpacing(8);
    codeRow->addWidget(m_codeEdit, 1);
    codeRow->addWidget(copyButton);

    auto* buttonRow = new QHBoxLayout();
    buttonRow->setSpacing(8);
    buttonRow->addWidget(generateButton);
    buttonRow->addWidget(clearButton);
    buttonRow->addStretch(1);

    auto* cardLayout = new QVBoxLayout();
    cardLayout->setContentsMargins(20, 20, 20, 20);
    cardLayout->setSpacing(8);
    cardLayout->addWidget(serialCaption);
    cardLayout->addWidget(m_serialEdit);
    cardLayout->addSpacing(6);
    cardLayout->addLayout(buttonRow);
    cardLayout->addSpacing(8);
    cardLayout->addWidget(codeCaption);
    cardLayout->addLayout(codeRow);

    auto* card = new QFrame(this);
    card->setObjectName(QStringLiteral("card"));
    card->setLayout(cardLayout);

    auto* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(28, 24, 28, 20);
    rootLayout->setSpacing(8);
    rootLayout->addWidget(title);
    rootLayout->addWidget(subtitle);
    rootLayout->addSpacing(8);
    rootLayout->addWidget(card);
    rootLayout->addStretch(1);
    rootLayout->addWidget(m_statusLabel);

    connect(generateButton, &QPushButton::clicked, this, &DecoderWindow::generateCode);
    connect(copyButton, &QPushButton::clicked, this, &DecoderWindow::copyCode);
    connect(clearButton, &QPushButton::clicked, this, &DecoderWindow::clearAll);

    setStyleSheet(QStringLiteral(
        "QWidget { background: #f3f5f7; color: #1d272c; font-size: 14px; }"
        "QLabel#title { font-size: 22px; font-weight: 600; background: transparent; }"
        "QLabel#subtitle, QLabel#status { color: #5d6b73; background: transparent; }"
        "QLabel { background: transparent; }"
        "QFrame#card { background: #ffffff; border: 1px solid #e2e7eb; border-radius: 12px; }"
        "QPlainTextEdit, QLineEdit {"
        "  background: #ffffff; border: 1px solid #cfd6dc; border-radius: 8px;"
        "  padding: 8px 10px; selection-background-color: #9ed4b8; }"
        "QPlainTextEdit:focus, QLineEdit:focus { border: 1px solid #55aa7f; }"
        "QPushButton#primary {"
        "  background: #55aa7f; color: #102018; border: none; border-radius: 8px;"
        "  padding: 8px 18px; font-weight: 600; }"
        "QPushButton#primary:hover { background: #92d96c; }"
        "QPushButton#primary:pressed { background: #3e8f68; }"
        "QPushButton#ghost {"
        "  background: #eef2f4; color: #1d272c; border: none; border-radius: 8px;"
        "  padding: 8px 16px; }"
        "QPushButton#ghost:hover { background: #e2e8eb; }"
        "QPushButton#ghost:pressed { background: #d5dde1; }"
    ));
}

void DecoderWindow::generateCode()
{
    const QString serial = m_serialEdit->toPlainText().trimmed();
    if (serial.isEmpty())
    {
        m_codeEdit->clear();
        m_statusLabel->setText(QStringLiteral("请输入机器序列号"));
        QMessageBox box(QMessageBox::Warning, QStringLiteral("提示"), QStringLiteral("请输入机器序列号"), QMessageBox::Ok, this);
        box.exec();
        m_serialEdit->setFocus();
        return;
    }

    const QString code = makeAuthorizationCode(serial);
    m_codeEdit->setText(code);
    m_codeEdit->setCursorPosition(0);
    m_statusLabel->setText(QStringLiteral("授权码已生成（%1 位）").arg(code.size()));
}

void DecoderWindow::copyCode()
{
    const QString code = m_codeEdit->text();
    if (code.isEmpty())
    {
        m_statusLabel->setText(QStringLiteral("请先生成授权码"));
        return;
    }
    QApplication::clipboard()->setText(code);
    m_statusLabel->setText(QStringLiteral("授权码已复制"));
}

void DecoderWindow::clearAll()
{
    m_serialEdit->clear();
    m_codeEdit->clear();
    m_statusLabel->setText(QStringLiteral("等待输入"));
    m_serialEdit->setFocus();
}
