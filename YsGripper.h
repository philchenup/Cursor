#ifndef YSGRIPPER_H
#define YSGRIPPER_H

#include "IGripper.h"

#include <boost/asio.hpp>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

class YsGripper : public IGripper {
    Q_OBJECT

public:
    explicit YsGripper(const std::string& port = "",
        uint16_t           speed = 800,
        uint16_t           power = 100,
        QObject* parent = nullptr);

    ~YsGripper() override;

    // Non-copyable
    YsGripper(const YsGripper&) = delete;
    YsGripper& operator=(const YsGripper&) = delete;

    void setPort(const std::string& port) override;
    void setSpeed(uint16_t speed) override;
    void setPower(uint16_t power) override;

public slots:
    void search()        override;
    void connect()       override;
    void disconnect()    override;
    void enable()        override;
    void disenable()     override;
    void open_gripper()  override;
    void close_gripper() override;

private:
    void disconnectSerial();
    bool checkReady();
    bool sendFrame(const std::vector<uint8_t>& frame);
    bool portExists(const std::string& name);
    bool openAndConfigure(boost::system::error_code& ec);
    bool reopenPort(boost::system::error_code& ec);
    void markDisconnected();

    std::string  port_;
    uint16_t     speed_;
    uint16_t     power_;
    bool         connected_;
    bool         enabled_;
    bool         shutting_down_;

    boost::asio::io_context  io_ctx_;
    boost::asio::serial_port serial_;

    std::recursive_mutex mutex_;
    std::chrono::steady_clock::time_point last_send_{};
};

#endif // YSGRIPPER_H
