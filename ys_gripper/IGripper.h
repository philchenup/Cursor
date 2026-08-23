#ifndef IGRIPPER_H
#define IGRIPPER_H

#include <QObject>
#include <QString>
#include <cstdint>
#include <string>

class IGripper : public QObject {
    Q_OBJECT

public:
    explicit IGripper(QObject* parent = nullptr) : QObject(parent) {}

    virtual ~IGripper() = default;

    virtual void setPort(const std::string& port) = 0;
    virtual void setSpeed(uint16_t speed) = 0;
    virtual void setPower(uint16_t power) = 0;

signals:
    void sendConnectStatus(const bool& status);

    void sendEnableStatus(const bool& status);

    void sendSearchCom(const int& com);

    void sendErrorMsg(const QString& msg);

    void sendInfoMsg(const QString& msg);

public slots:
    virtual void search() = 0;

    virtual void connect() = 0;

    virtual void disconnect() = 0;

    virtual void enable() = 0;

    virtual void disenable() = 0;

    virtual void open_gripper() = 0;

    virtual void close_gripper() = 0;
};

#endif // IGRIPPER_H
