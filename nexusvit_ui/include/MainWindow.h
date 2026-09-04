#ifndef NEXUSVIT_MAIN_WINDOW_H
#define NEXUSVIT_MAIN_WINDOW_H

#include <QMainWindow>
#include <QList>
#include <QString>

class QComboBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QSlider;
class QTabWidget;
class QTableWidget;
class QToolButton;
class QTreeWidget;
class ViewportWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);

private:
    void buildHeader();
    void buildRibbon();
    void buildBody();
    QWidget* buildLeftPanel();
    QWidget* buildRightPanel();
    QWidget* buildCenter();
    QWidget* buildBottom();
    QWidget* ribbonGroup(const QString& title, const QList<QWidget*>& buttons);
    QToolButton* ribbonButton(const QString& icon, const QString& text);
    QPushButton* accentButton(const QString& text, const QString& role, const QString& objectName = QString());
    QWidget* statusPill(const QString& name, const QString& state);
    QWidget* coordColumn(const QString& title, const QStringList& labels);
    void applyStyle();

    ViewportWidget* m_viewport = nullptr;
    QTabWidget* m_viewTabs = nullptr;
    QPlainTextEdit* m_console = nullptr;
    QTreeWidget* m_dataTree = nullptr;
    QTableWidget* m_props = nullptr;
    QSlider* m_speed = nullptr;
    QLabel* m_speedValue = nullptr;
};

#endif
