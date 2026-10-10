#pragma once

#include <QWidget>

class QPlainTextEdit;
class QLineEdit;
class QLabel;

class DecoderWindow : public QWidget
{
    Q_OBJECT

public:
    explicit DecoderWindow(QWidget* parent = nullptr);

private slots:
    void generateCode();
    void copyCode();
    void clearAll();

private:
    QPlainTextEdit* m_serialEdit = nullptr;
    QLineEdit* m_codeEdit = nullptr;
    QLabel* m_statusLabel = nullptr;
};
