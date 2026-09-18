#ifndef IGRIPPER_H
#define IGRIPPER_H

#include <QObject>
#include <QString>
#include <QStringList>

#include <cstdint>
#include <string>

/**
 * IGripper
 *
 * Qt 夹爪抽象接口。界面侧通过 slots 触发动作，通过 signals 接收结果。
 *
 * 注意: connect()/disconnect() 与 QObject 同名，绑定按钮时请使用:
 *   QObject::connect(btn, &QPushButton::clicked, gripper, [gripper]() { gripper->connect(); });
 */
class IGripper : public QObject {
    Q_OBJECT

public:
    explicit IGripper(QObject* parent = nullptr) : QObject(parent) {}
    ~IGripper() override = default;

    virtual void setPort(const std::string& port) = 0;
    virtual void setSpeed(uint16_t speed) = 0;
    virtual void setPower(uint16_t power) = 0;

public slots:
    virtual void search() = 0;
    virtual void connect() = 0;
    virtual void disconnect() = 0;
    virtual void enable() = 0;
    virtual void disenable() = 0;
    virtual void open_gripper(const std::string& tag) = 0;
    virtual void close_gripper(const std::string& tag) = 0;

signals:
    void searchFinished(const QStringList& ports);
    void connectStateChanged(bool connected, const QString& message);
    void enableStateChanged(bool enabled, const QString& message);
    void motionFinished(const QString& action, const QString& result, int position);
    void errorOccurred(const QString& message);
};

#endif // IGRIPPER_H
