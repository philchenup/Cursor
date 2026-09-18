#ifndef TEKGRIPPER_H
#define TEKGRIPPER_H

#include "IGripper.h"

#include "Changingtek_rtu_psdk.h"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <memory>

#ifdef _WIN32
#include <windows.h> // 为了解决中文乱码问题
#endif


class TekGripper : public IGripper {
    Q_OBJECT

public:
    explicit TekGripper(const std::string& port = "",
        QObject* parent = nullptr);

    ~TekGripper() override;

    // Non-copyable
    TekGripper(const TekGripper&) = delete;
    TekGripper& operator=(const TekGripper&) = delete;

    void setPort(const std::string& port)override;
    void setSpeed(uint16_t speed) override;
    void setPower(uint16_t power) override;

public slots:
    // IGripper pure-virtual slots
    void search()        override;
    void connect()       override;
    void disconnect()    override;
    void enable()        override;
    void disenable()     override;
    void open_gripper(const std::string& tag)  override;   // catch_gripper(1.0)
    void close_gripper(const std::string& tag) override;   // catch_gripper(0.0)

private:

    void disconnectSerial();
    bool checkReady();
    bool portExists(const std::string& name);
    void recreateSdk();
    int clampPercent(int value) const;
    void reportError(const std::string& message);

    int SLAVE_ID = 1;

    int POS_OPEN = 0;          // 打开位置 (0.00mm)
    int POS_CLOSE = 10000;     // 闭合位置 (120.00mm)。以CTAG2F120s为例，行程120mm，范围0-12000，以此类推。

    std::string  port_;
    bool         connected_;
    bool         enabled_;

    int m_speed;
    int m_torque;

    // Changingtek_rtu_psdk 没有默认构造函数，且 setPort() 需要按新串口重建实例
    std::unique_ptr<Changingtek_rtu_psdk> gripper_sdk;
};

#endif // TEKGRIPPER_H
