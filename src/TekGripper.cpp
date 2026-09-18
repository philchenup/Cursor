/**
 * TekGripper.cpp
 *
 * 知行手闭环步进夹爪的 Qt 封装。
 * 控制流程与 main.cpp 示例一致:
 *   连接串口 -> 使能 -> temp_move 打开/闭合 -> 等待位置或力控到位
 *
 * Qt 界面绑定示例:
 *   auto* gripper = new TekGripper("/dev/ttyUSB0", this);  // Windows 下如 "COM3"
 *   QObject::connect(btnSearch,  &QPushButton::clicked, gripper, &TekGripper::search);
 *   QObject::connect(btnConnect, &QPushButton::clicked, gripper, [gripper]() { gripper->connect(); });
 *   QObject::connect(btnEnable,  &QPushButton::clicked, gripper, &TekGripper::enable);
 *   QObject::connect(btnOpen,    &QPushButton::clicked, gripper, [gripper]() { gripper->open_gripper("ui"); });
 *   QObject::connect(btnClose,   &QPushButton::clicked, gripper, [gripper]() { gripper->close_gripper("ui"); });
 *   QObject::connect(gripper, &IGripper::searchFinished, comboPort, ...);
 *   QObject::connect(gripper, &IGripper::errorOccurred, this, ...);
 */

#include "TekGripper.h"

#include <QString>
#include <QStringList>
#include <algorithm>
#include <cctype>
#include <cstring>
#include <exception>
#include <vector>

#ifdef _WIN32
#include <winbase.h>
#else
#include <glob.h>
#include <unistd.h>
#endif

namespace {

constexpr int kDefaultSpeed = 100;
constexpr int kDefaultTorque = 60;
constexpr int kAccelPct = 100;
constexpr int kDecelPct = 100;
constexpr double kMoveTimeoutSec = 10.0;
constexpr int kBaudrate = 115200;
constexpr double kSdkTimeoutSec = 0.5;

QString toQString(const std::string& text) {
    return QString::fromStdString(text);
}

}  // namespace

TekGripper::TekGripper(const std::string& port, QObject* parent)
    : IGripper(parent)
    , port_(port)
    , connected_(false)
    , enabled_(false)
    , m_speed(kDefaultSpeed)
    , m_torque(kDefaultTorque) {
#ifdef _WIN32
    SetConsoleOutputCP(65001);
#endif
    recreateSdk();
}

TekGripper::~TekGripper() {
    try {
        if (enabled_ && gripper_sdk) {
            gripper_sdk->enable(false);
        }
    } catch (...) {
        // 析构中忽略通信异常
    }
    enabled_ = false;
    disconnectSerial();
}

void TekGripper::recreateSdk() {
    gripper_sdk.reset();
    if (port_.empty()) {
        return;
    }
    gripper_sdk = std::make_unique<Changingtek_rtu_psdk>(
        port_, SLAVE_ID, kBaudrate, kSdkTimeoutSec);
}

int TekGripper::clampPercent(int value) const {
    return std::max(0, std::min(100, value));
}

void TekGripper::reportError(const std::string& message) {
    std::cerr << message << std::endl;
    emit errorOccurred(toQString(message));
}

void TekGripper::setPort(const std::string& port) {
    if (port_ == port && gripper_sdk) {
        return;
    }

    if (connected_) {
        disconnect();
    }

    port_ = port;
    recreateSdk();
}

void TekGripper::setSpeed(uint16_t speed) {
    m_speed = clampPercent(static_cast<int>(speed));
}

void TekGripper::setPower(uint16_t power) {
    m_torque = clampPercent(static_cast<int>(power));
}

void TekGripper::search() {
    QStringList ports;

#ifdef _WIN32
    std::vector<char> buffer(65536, 0);
    const DWORD n = QueryDosDeviceA(nullptr, buffer.data(), static_cast<DWORD>(buffer.size()));
    if (n != 0) {
        for (char* p = buffer.data(); *p != '\0'; p += std::strlen(p) + 1) {
            if (_strnicmp(p, "COM", 3) == 0 && std::isdigit(static_cast<unsigned char>(p[3])) != 0) {
                ports << QString::fromLocal8Bit(p);
            }
        }
        ports.removeDuplicates();
        ports.sort();
    } else {
        reportError("搜索串口失败: QueryDosDevice");
    }
#else
    glob_t glob_result{};
    int flags = 0;
    const char* patterns[] = {"/dev/ttyUSB*", "/dev/ttyACM*", "/dev/ttyS*"};
    for (const char* pattern : patterns) {
        const int rc = glob(pattern, flags, nullptr, &glob_result);
        if (rc == 0) {
            flags |= GLOB_APPEND;
        }
    }
    if (flags & GLOB_APPEND) {
        for (size_t i = 0; i < glob_result.gl_pathc; ++i) {
            ports << QString::fromLocal8Bit(glob_result.gl_pathv[i]);
        }
        globfree(&glob_result);
    }
    ports.removeDuplicates();
    ports.sort();
#endif

    std::cout << "搜索到串口 " << ports.size() << " 个:" << std::endl;
    for (const QString& port : ports) {
        std::cout << "  - " << port.toStdString() << std::endl;
    }
    emit searchFinished(ports);
}

void TekGripper::connect() {
    if (connected_) {
        emit connectStateChanged(true, QStringLiteral("夹爪已连接"));
        return;
    }

    if (port_.empty()) {
        reportError("连接失败: 串口未设置，请先 setPort() 或调用 search() 后选择端口");
        emit connectStateChanged(false, QStringLiteral("串口未设置"));
        return;
    }

    if (!portExists(port_)) {
        reportError("连接失败: 串口不存在 " + port_);
        emit connectStateChanged(false, toQString("串口不存在: " + port_));
        return;
    }

    try {
        if (!gripper_sdk) {
            recreateSdk();
        }
        if (!gripper_sdk) {
            reportError("连接失败: SDK 创建失败");
            emit connectStateChanged(false, QStringLiteral("SDK 创建失败"));
            return;
        }

        std::cout << "正在连接 " << port_ << " ..." << std::endl;
        if (!gripper_sdk->connect()) {
            connected_ = false;
            reportError("连接失败: " + port_);
            emit connectStateChanged(false, toQString("连接失败: " + port_));
            return;
        }

        connected_ = true;
        enabled_ = false;
        std::cout << "已连接。" << std::endl;
        emit connectStateChanged(true, toQString("已连接: " + port_));
    } catch (const std::exception& e) {
        connected_ = false;
        reportError(std::string("连接异常: ") + e.what());
        emit connectStateChanged(false, toQString(e.what()));
    }
}

void TekGripper::disconnect() {
    try {
        if (enabled_ && gripper_sdk) {
            gripper_sdk->enable(false);
        }
    } catch (const std::exception& e) {
        reportError(std::string("失能异常: ") + e.what());
    }

    enabled_ = false;
    emit enableStateChanged(false, QStringLiteral("已失能"));
    disconnectSerial();
    emit connectStateChanged(false, QStringLiteral("已断开"));
}

void TekGripper::enable() {
    if (!connected_ || !gripper_sdk) {
        reportError("使能失败: 夹爪未连接");
        emit enableStateChanged(false, QStringLiteral("夹爪未连接"));
        return;
    }

    try {
        std::cout << "正在使能执行器..." << std::endl;
        gripper_sdk->enable(true);
        enabled_ = true;
        std::cout << "执行器已使能。" << std::endl;
        emit enableStateChanged(true, QStringLiteral("执行器已使能"));
    } catch (const std::exception& e) {
        enabled_ = false;
        reportError(std::string("使能异常: ") + e.what());
        emit enableStateChanged(false, toQString(e.what()));
    }
}

void TekGripper::disenable() {
    if (!gripper_sdk) {
        enabled_ = false;
        emit enableStateChanged(false, QStringLiteral("SDK 未创建"));
        return;
    }

    try {
        gripper_sdk->enable(false);
        enabled_ = false;
        std::cout << "执行器已失能。" << std::endl;
        emit enableStateChanged(false, QStringLiteral("执行器已失能"));
    } catch (const std::exception& e) {
        reportError(std::string("失能异常: ") + e.what());
        emit enableStateChanged(enabled_, toQString(e.what()));
    }
}

void TekGripper::open_gripper(const std::string& tag) {
    if (!checkReady()) {
        return;
    }

    try {
        std::cout << "[打开] tag=" << tag << " 移动到 " << POS_OPEN << " ..." << std::endl;
        gripper_sdk->temp_move(POS_OPEN, m_speed, m_torque, kAccelPct, kDecelPct, true);

        const std::string result = gripper_sdk->wait_until_pos_or_torque(kMoveTimeoutSec);
        const int feedback = gripper_sdk->feedback_position();
        std::cout << "  -> 结果: " << result
                  << " [位置反馈: " << feedback << "]" << std::endl;

        if (result == "timeout") {
            reportError("打开夹爪超时");
        }
        emit motionFinished(QStringLiteral("open"), toQString(result), feedback);
    } catch (const std::exception& e) {
        reportError(std::string("打开夹爪失败: ") + e.what());
    }
}

void TekGripper::close_gripper(const std::string& tag) {
    if (!checkReady()) {
        return;
    }

    try {
        std::cout << "[闭合] tag=" << tag << " 移动到 " << POS_CLOSE << " ..." << std::endl;
        gripper_sdk->temp_move(POS_CLOSE, m_speed, m_torque, kAccelPct, kDecelPct, true);

        // 夹取物体时可能触发 torque 力控到位
        const std::string result = gripper_sdk->wait_until_pos_or_torque(kMoveTimeoutSec);
        const int feedback = gripper_sdk->feedback_position();
        const bool torque_ok = gripper_sdk->torque_reached();
        std::cout << "  -> 结果: " << result
                  << " [位置反馈: " << feedback << "]"
                  << " [力控到位: " << (torque_ok ? "是" : "否") << "]" << std::endl;

        if (result == "timeout") {
            reportError("闭合夹爪超时");
        }
        emit motionFinished(QStringLiteral("close"), toQString(result), feedback);
    } catch (const std::exception& e) {
        reportError(std::string("闭合夹爪失败: ") + e.what());
    }
}

void TekGripper::disconnectSerial() {
    if (gripper_sdk) {
        gripper_sdk->disconnect();
    }
    connected_ = false;
    enabled_ = false;
    std::cout << "串口已断开。" << std::endl;
}

bool TekGripper::checkReady() {
    if (!connected_ || !gripper_sdk) {
        reportError("夹爪未连接，无法运动");
        return false;
    }
    if (!enabled_) {
        reportError("夹爪未使能，无法运动");
        return false;
    }
    return true;
}

bool TekGripper::portExists(const std::string& name) {
    if (name.empty()) {
        return false;
    }

#ifdef _WIN32
    std::string full_name = name;
    if (full_name.find("\\\\.\\") == std::string::npos) {
        full_name = "\\\\.\\" + full_name;
    }

    HANDLE handle = CreateFileA(full_name.c_str(),
                                GENERIC_READ | GENERIC_WRITE,
                                0,
                                nullptr,
                                OPEN_EXISTING,
                                0,
                                nullptr);
    if (handle != INVALID_HANDLE_VALUE) {
        CloseHandle(handle);
        return true;
    }

    const DWORD err = GetLastError();
    // 端口存在但被占用时仍视为可用设备
    return err == ERROR_ACCESS_DENIED || err == ERROR_SHARING_VIOLATION;
#else
    return ::access(name.c_str(), F_OK) == 0;
#endif
}
