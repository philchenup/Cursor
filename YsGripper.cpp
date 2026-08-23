#include "YsGripper.h"

#include <QStringList>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <thread>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <termios.h>
#include <unistd.h>
#endif

namespace {

constexpr int kMaxWriteAttempts = 3;
constexpr int kMaxOpenAttempts = 3;
constexpr auto kUsbSettleDelay = std::chrono::milliseconds(80);
constexpr auto kReopenDelay = std::chrono::milliseconds(120);
constexpr auto kInterCommandGap = std::chrono::milliseconds(20);

#if defined(_WIN32)
constexpr int kWinErrorInvalidHandle = 6;
constexpr int kWinErrorFileNotFound = 2;
constexpr int kWinErrorBadCommand = 22;
constexpr int kWinErrorCrc = 23;
constexpr int kWinErrorGenFailure = 31;
constexpr int kWinErrorSemTimeout = 121;
constexpr int kWinErrorNoSuchDevice = 433;
constexpr int kWinErrorOperationAborted = 995;
constexpr int kWinErrorDeviceNotConnected = 1167;
#endif

QString systemMessage(const boost::system::error_code& ec)
{
#if defined(_WIN32)
    // Boost/Win32 messages use the ANSI code page (e.g. GBK), not UTF-8.
    return QString::fromLocal8Bit(ec.message().c_str());
#else
    return QString::fromStdString(ec.message());
#endif
}

QString formatAsioError(const char* op, const boost::system::error_code& ec)
{
    return QString("%1 error: %2 [%3:%4]")
        .arg(QLatin1String(op),
             systemMessage(ec),
             QLatin1String(ec.category().name()),
             QString::number(ec.value()));
}

std::string trimCopy(std::string s)
{
    const auto is_space = [](unsigned char c) { return std::isspace(c) != 0; };
    while (!s.empty() && is_space(static_cast<unsigned char>(s.front()))) {
        s.erase(s.begin());
    }
    while (!s.empty() && is_space(static_cast<unsigned char>(s.back()))) {
        s.pop_back();
    }
    return s;
}

bool isAllDigits(const std::string& s)
{
    return !s.empty() && std::all_of(s.begin(), s.end(),
        [](unsigned char c) { return std::isdigit(c) != 0; });
}

std::string toUpperAscii(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(),
        [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return s;
}

bool looksLikeComToken(const std::string& token)
{
    if (token.empty()) {
        return false;
    }
    if (isAllDigits(token)) {
        return true;
    }
    const std::string upper = toUpperAscii(token);
    if (upper.size() > 3 && upper.compare(0, 3, "COM") == 0 && isAllDigits(upper.substr(3))) {
        return true;
    }
    return token.compare(0, 4, "\\\\.\\") == 0
        || token.compare(0, 4, "//./") == 0
        || token.compare(0, 4, "\\\\?\\") == 0;
}

bool isMissingSerialDevice(const boost::system::error_code& ec)
{
    if (!ec) {
        return false;
    }
#if defined(_WIN32)
    switch (ec.value()) {
    case kWinErrorFileNotFound:
    case kWinErrorNoSuchDevice:
    case kWinErrorDeviceNotConnected:
        return true;
    default:
        return false;
    }
#else
    return ec == boost::system::errc::no_such_device
        || ec == boost::asio::error::no_such_device;
#endif
}

std::string asioPortName(const std::string& port)
{
#if defined(_WIN32)
    if (port.empty()) {
        return {};
    }
    // CreateFile requires \\.\COMx. Boost.Asio also prepends this when the
    // first character is not '\', but we pass the Win32 path explicitly so
    // COM10+ and already-prefixed Qt systemLocation() strings both work.
    if (port.compare(0, 4, "\\\\.\\") == 0 || port.compare(0, 4, "\\\\?\\") == 0) {
        return port;
    }
    return "\\\\.\\" + port;
#else
    return port;
#endif
}

void clearNativeCommError(boost::asio::serial_port& serial)
{
    if (!serial.is_open()) {
        return;
    }
#if defined(_WIN32)
    DWORD errors = 0;
    COMSTAT stat{};
    ClearCommError(serial.native_handle(), &errors, &stat);
#else
    (void)serial;
#endif
}

void purgeNativeBuffers(boost::asio::serial_port& serial)
{
    if (!serial.is_open()) {
        return;
    }
#if defined(_WIN32)
    PurgeComm(serial.native_handle(),
        PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
#else
    tcflush(serial.native_handle(), TCIOFLUSH);
#endif
}

bool applyNativePortTuning(boost::asio::serial_port& serial, boost::system::error_code& ec)
{
    ec.clear();
    if (!serial.is_open()) {
        ec = boost::asio::error::bad_descriptor;
        return false;
    }

#if defined(_WIN32)
    const HANDLE handle = serial.native_handle();
    if (handle == INVALID_HANDLE_VALUE) {
        ec = boost::system::error_code(kWinErrorInvalidHandle,
            boost::asio::error::get_system_category());
        return false;
    }

    // Boost.Asio win_iocp_handle_service uses overlapped WriteFile. USB-UART
    // drivers commonly return ERROR_GEN_FAILURE (system:31) unless COMMTIMEOUTS
    // are set and fAbortOnError is cleared.
    DCB dcb{};
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(handle, &dcb)) {
        ec = boost::system::error_code(static_cast<int>(GetLastError()),
            boost::asio::error::get_system_category());
        return false;
    }
    dcb.fBinary = TRUE;
    dcb.fAbortOnError = FALSE;
    if (!SetCommState(handle, &dcb)) {
        ec = boost::system::error_code(static_cast<int>(GetLastError()),
            boost::asio::error::get_system_category());
        return false;
    }

    COMMTIMEOUTS timeouts{};
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.ReadTotalTimeoutConstant = 100;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 200;
    if (!SetCommTimeouts(handle, &timeouts)) {
        ec = boost::system::error_code(static_cast<int>(GetLastError()),
            boost::asio::error::get_system_category());
        return false;
    }

    SetupComm(handle, 4096, 4096);
    clearNativeCommError(serial);
    purgeNativeBuffers(serial);
    return true;
#else
    const int fd = serial.native_handle();
    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        ec = boost::system::error_code(errno, boost::asio::error::get_system_category());
        return false;
    }
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 2; // 200 ms
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        ec = boost::system::error_code(errno, boost::asio::error::get_system_category());
        return false;
    }
    purgeNativeBuffers(serial);
    return true;
#endif
}

bool isRecoverableSerialError(const boost::system::error_code& ec)
{
    if (!ec) {
        return false;
    }
#if defined(_WIN32)
    switch (ec.value()) {
    case kWinErrorFileNotFound:
    case kWinErrorInvalidHandle:
    case kWinErrorBadCommand:
    case kWinErrorCrc:
    case kWinErrorGenFailure:
    case kWinErrorSemTimeout:
    case kWinErrorNoSuchDevice:
    case kWinErrorOperationAborted:
    case kWinErrorDeviceNotConnected:
        return true;
    default:
        break;
    }
#endif
    return ec == boost::asio::error::broken_pipe
        || ec == boost::asio::error::connection_reset
        || ec == boost::asio::error::fault
        || ec == boost::asio::error::io_error
        || ec == boost::asio::error::eof
        || ec == boost::asio::error::timed_out;
}

void closeQuietly(boost::asio::serial_port& serial)
{
    boost::system::error_code ignore;
    serial.close(ignore);
}

} // namespace

YsGripper::YsGripper(const std::string& port,
    uint16_t           speed,
    uint16_t           power,
    QObject* parent)
    : IGripper(parent)
    , port_(normalizePortName(port))
    , speed_(speed)
    , power_(power)
    , connected_(false)
    , enabled_(false)
    , shutting_down_(false)
    , io_ctx_()
    , serial_(io_ctx_)
{
}

YsGripper::~YsGripper()
{
    shutting_down_ = true;
    disconnectSerial();
}

// ========================================================================== //
//  Setters
// ========================================================================== //

void YsGripper::setPort(const std::string& port)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (connected_ || serial_.is_open()) {
        emit sendErrorMsg("Cannot change port while connected. Disconnect first.");
        return;
    }
    port_ = normalizePortName(port);
    if (port_.empty() && !port.empty()) {
        emit sendErrorMsg("Invalid serial port name: \""
            + QString::fromStdString(port) + "\"");
    }
}

void YsGripper::setSpeed(uint16_t speed)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    speed_ = speed;
}

void YsGripper::setPower(uint16_t power)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    power_ = power;
}

// ========================================================================== //
//  IGripper slots
// ========================================================================== //

void YsGripper::search()
{
    const std::vector<std::string> ports = listAvailablePorts();
    if (ports.empty()) {
        emit sendInfoMsg("No serial ports found.");
        return;
    }

    QStringList names;
    for (const auto& name : ports) {
        names << QString::fromStdString(name);
#if defined(_WIN32)
        if (name.size() > 3 && (name[0] == 'C' || name[0] == 'c')) {
            bool ok = false;
            const int n = QString::fromStdString(name.substr(3)).toInt(&ok);
            if (ok) {
                emit sendSearchCom(n);
            }
        }
#else
        const auto pos = name.find_last_not_of("0123456789");
        if (pos != std::string::npos && pos + 1 < name.size()) {
            emit sendSearchCom(std::stoi(name.substr(pos + 1)));
        }
#endif
    }
    emit sendInfoMsg("Found ports: " + names.join(", "));
}

void YsGripper::connect()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (connected_ && serial_.is_open()) {
        return;
    }

    port_ = normalizePortName(port_);
    if (port_.empty()) {
        emit sendConnectStatus(false);
        emit sendErrorMsg("Connect failed: serial port is empty. "
                         "Call setPort(\"COMx\") or search() first.");
        return;
    }

    boost::system::error_code ec;
    if (serial_.is_open()) {
        serial_.close(ec);
        ec.clear();
    }

    if (!tryOpenWithRetries(ec)) {
        if (isMissingSerialDevice(ec)) {
            const std::vector<std::string> available = listAvailablePorts();
            if (available.size() == 1 && available.front() != port_) {
                const std::string fallback = available.front();
                emit sendInfoMsg(QString("Port %1 not found, trying %2")
                    .arg(QString::fromStdString(port_),
                         QString::fromStdString(fallback)));
                port_ = fallback;
                ec.clear();
                if (tryOpenWithRetries(ec)) {
                    connected_ = true;
                    emit sendConnectStatus(true);
                    emit sendInfoMsg("YsGripper connected on "
                        + QString::fromStdString(port_));
                    return;
                }
            }
        }
        connected_ = false;
        enabled_ = false;
        emit sendConnectStatus(false);
        emit sendErrorMsg(formatConnectError(ec));
        return;
    }

    connected_ = true;
    emit sendConnectStatus(true);
    emit sendInfoMsg("YsGripper connected on " + QString::fromStdString(port_)
        + " (" + QString::fromStdString(asioPortName(port_)) + ")");
}

void YsGripper::disconnect()
{
    disconnectSerial();
}

void YsGripper::enable()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!connected_ || !serial_.is_open()) {
        emit sendErrorMsg("Cannot enable: gripper not connected.");
        return;
    }
    enabled_ = true;
    emit sendEnableStatus(true);
    emit sendInfoMsg("YsGripper enabled.");
}

void YsGripper::disenable()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    enabled_ = false;
    emit sendEnableStatus(false);
    emit sendInfoMsg("YsGripper disabled.");
}

void YsGripper::open_gripper()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!checkReady()) return;

    // Packet: [0xEB][0x90][0x01][0x03][0x11][speed_lo][speed_hi][checksum]
    constexpr uint8_t b2 = 0x01;
    constexpr uint8_t b3 = 0x03;
    constexpr uint8_t b4 = 0x11;

    const uint8_t speed_lo = static_cast<uint8_t>(speed_ & 0x00FFu);
    const uint8_t speed_hi = static_cast<uint8_t>(speed_ >> 8u);
    const uint8_t checksum = static_cast<uint8_t>(
        (b2 + b3 + b4 + speed_lo + speed_hi) & 0x00FFu);

    if (sendFrame({ 0xEB, 0x90, b2, b3, b4, speed_lo, speed_hi, checksum })) {
        emit sendInfoMsg("Gripper opened.");
    }
}

void YsGripper::close_gripper()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!checkReady()) return;

    // Packet: [0xEB][0x90][0x01][0x05][0x10]
    //         [speed_lo][speed_hi][power_lo][power_hi][checksum]
    constexpr uint8_t b2 = 0x01;
    constexpr uint8_t b3 = 0x05;
    constexpr uint8_t b4 = 0x10;

    const uint8_t speed_lo = static_cast<uint8_t>(speed_ & 0x00FFu);
    const uint8_t speed_hi = static_cast<uint8_t>(speed_ >> 8u);
    const uint8_t power_lo = static_cast<uint8_t>(power_ & 0x00FFu);
    const uint8_t power_hi = static_cast<uint8_t>(power_ >> 8u);
    const uint8_t checksum = static_cast<uint8_t>(
        (b2 + b3 + b4 + speed_lo + speed_hi + power_lo + power_hi) & 0x00FFu);

    if (sendFrame({ 0xEB, 0x90, b2, b3, b4,
                    speed_lo, speed_hi, power_lo, power_hi, checksum })) {
        emit sendInfoMsg("Gripper closed.");
    }
}

void YsGripper::disconnectSerial()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const bool was_connected = connected_ || serial_.is_open();
    if (serial_.is_open()) {
        clearNativeCommError(serial_);
        purgeNativeBuffers(serial_);
        closeQuietly(serial_);
    }
    connected_ = false;
    enabled_ = false;

    if (was_connected && !shutting_down_) {
        emit sendConnectStatus(false);
        emit sendEnableStatus(false);
        emit sendInfoMsg("YsGripper disconnected.");
    }
}

bool YsGripper::checkReady()
{
    if (!connected_ || !serial_.is_open()) {
        emit sendErrorMsg("Gripper not connected.");
        return false;
    }
    if (!enabled_) {
        emit sendErrorMsg("Gripper not enabled.");
        return false;
    }
    return true;
}

bool YsGripper::sendFrame(const std::vector<uint8_t>& frame)
{
    if (frame.empty()) {
        emit sendErrorMsg("Serial write error: empty frame.");
        return false;
    }

    boost::system::error_code ec;
    for (int attempt = 1; attempt <= kMaxWriteAttempts; ++attempt) {
        if (!serial_.is_open()) {
            if (!reopenPort(ec)) {
                markDisconnected();
                emit sendErrorMsg(formatAsioError("Reconnect", ec));
                emit sendConnectStatus(false);
                emit sendEnableStatus(false);
                return false;
            }
        }

        // Unlatch a previous UART error (ERROR_GEN_FAILURE / system:31) before write.
        clearNativeCommError(serial_);

        if (last_send_.time_since_epoch().count() != 0) {
            const auto elapsed = std::chrono::steady_clock::now() - last_send_;
            if (elapsed < kInterCommandGap) {
                std::this_thread::sleep_for(kInterCommandGap - elapsed);
            }
        }

        const std::size_t written = boost::asio::write(
            serial_, boost::asio::buffer(frame), ec);

        if (!ec && written == frame.size()) {
            last_send_ = std::chrono::steady_clock::now();
            return true;
        }

        if (!ec && written != frame.size()) {
            ec = boost::asio::error::eof;
        }

        const QString err = formatAsioError("Serial write", ec);
        const bool recoverable = isRecoverableSerialError(ec);

        if (attempt < kMaxWriteAttempts && recoverable) {
            emit sendInfoMsg(err + QString(" (retry %1/%2)")
                .arg(attempt).arg(kMaxWriteAttempts - 1));
            clearNativeCommError(serial_);
            purgeNativeBuffers(serial_);
            if (attempt >= 2 && !reopenPort(ec)) {
                markDisconnected();
                emit sendErrorMsg(formatAsioError("Reconnect", ec));
                emit sendConnectStatus(false);
                emit sendEnableStatus(false);
                return false;
            }
            ec.clear();
            continue;
        }

        emit sendErrorMsg(err);
        if (recoverable) {
            markDisconnected();
            emit sendConnectStatus(false);
            emit sendEnableStatus(false);
        }
        return false;
    }

    markDisconnected();
    emit sendErrorMsg("Serial write error: retries exhausted.");
    emit sendConnectStatus(false);
    emit sendEnableStatus(false);
    return false;
}

bool YsGripper::portExists(const std::string& name) const
{
#if defined(_WIN32)
    // QueryDosDevice does not open the handle, so search will not reset USB-UART chips.
    const std::string dos_name = normalizePortName(name);
    if (dos_name.empty()) {
        return false;
    }
    char buf[256] = {};
    return QueryDosDeviceA(dos_name.c_str(), buf, static_cast<DWORD>(sizeof(buf))) != 0;
#else
    return ::access(name.c_str(), F_OK) == 0;
#endif
}

std::vector<std::string> YsGripper::listAvailablePorts() const
{
    std::vector<std::string> ports;
#if defined(_WIN32)
    for (int i = 1; i <= 256; ++i) {
        std::string name = "COM" + std::to_string(i);
        if (portExists(name)) {
            ports.push_back(name);
        }
    }
#else
    const char* prefixes[] = { "/dev/ttyUSB", "/dev/ttyACM", "/dev/ttyS" };
    for (const char* prefix : prefixes) {
        for (int i = 0; i < 32; ++i) {
            std::string name = std::string(prefix) + std::to_string(i);
            if (portExists(name)) {
                ports.push_back(name);
            }
        }
    }
#endif
    return ports;
}

QString YsGripper::formatConnectError(const boost::system::error_code& ec) const
{
    const std::string device = asioPortName(port_);
    QString msg = QString("Connect failed: cannot open %1")
        .arg(QString::fromStdString(port_.empty() ? "<empty>" : port_));
    if (!device.empty() && device != port_) {
        msg += QString(" (CreateFile %1)").arg(QString::fromStdString(device));
    }
    if (ec) {
        msg += QString(" — %1 [system:%2]")
            .arg(systemMessage(ec), QString::number(ec.value()));
    }

#if defined(_WIN32)
    if (isMissingSerialDevice(ec)) {
        const std::vector<std::string> available = listAvailablePorts();
        if (available.empty()) {
            msg += " No COM ports found. Plug in the gripper USB adapter "
                   "and install its driver (CH340 / CP2102 / FT232), "
                   "then check Device Manager.";
        } else {
            QStringList ports;
            for (const auto& p : available) {
                ports << QString::fromStdString(p);
            }
            msg += QString(" Available ports: %1. Call setPort() with one of these.")
                .arg(ports.join(", "));
        }
    } else if (ec.value() == 5 || ec.value() == 32) {
        msg += " The port is in use by another program. Close the serial "
               "monitor / previous connection and retry.";
    }
#endif
    return msg;
}

std::string YsGripper::normalizePortName(const std::string& raw)
{
    std::string port = trimCopy(raw);
    if (port.empty()) {
        return {};
    }

#if defined(_WIN32)
    // Combo-box text such as "COM3 - USB-SERIAL CH340".
    const auto cut = port.find_first_of(" \t(");
    if (cut != std::string::npos) {
        const std::string head = port.substr(0, cut);
        if (looksLikeComToken(head)) {
            port = head;
        }
    }

    if (port.compare(0, 4, "\\\\.\\") == 0 || port.compare(0, 4, "\\\\?\\") == 0
        || port.compare(0, 4, "//./") == 0) {
        port.erase(0, 4);
    }

    if (!port.empty() && port.back() == ':') {
        port.pop_back();
    }
    port = trimCopy(port);
    if (port.empty()) {
        return {};
    }

    if (isAllDigits(port)) {
        return "COM" + port;
    }

    const std::string upper = toUpperAscii(port);
    if (upper.size() > 3 && upper.compare(0, 3, "COM") == 0
        && isAllDigits(upper.substr(3))) {
        return upper;
    }
    return upper;
#else
    return port;
#endif
}

bool YsGripper::tryOpenWithRetries(boost::system::error_code& ec)
{
    for (int attempt = 1; attempt <= kMaxOpenAttempts; ++attempt) {
        if (openAndConfigure(ec)) {
            return true;
        }
        closeQuietly(serial_);
        const bool retry = isMissingSerialDevice(ec)
#if defined(_WIN32)
            || ec.value() == kWinErrorGenFailure
            || ec.value() == kWinErrorOperationAborted
#endif
            || ec == boost::asio::error::try_again;
        if (!retry || attempt == kMaxOpenAttempts) {
            return false;
        }
        std::this_thread::sleep_for(kReopenDelay);
        ec.clear();
    }
    return false;
}

bool YsGripper::openAndConfigure(boost::system::error_code& ec)
{
    ec.clear();
    const std::string device = asioPortName(port_);
    if (device.empty()) {
        ec = boost::asio::error::invalid_argument;
        return false;
    }
    serial_.open(device, ec);
    if (ec) {
        return false;
    }

    serial_.set_option(boost::asio::serial_port_base::baud_rate(115200), ec);
    if (ec) {
        closeQuietly(serial_);
        return false;
    }
    serial_.set_option(boost::asio::serial_port_base::character_size(8), ec);
    if (ec) {
        closeQuietly(serial_);
        return false;
    }
    serial_.set_option(boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none), ec);
    if (ec) {
        closeQuietly(serial_);
        return false;
    }
    serial_.set_option(boost::asio::serial_port_base::stop_bits(
        boost::asio::serial_port_base::stop_bits::one), ec);
    if (ec) {
        closeQuietly(serial_);
        return false;
    }
    serial_.set_option(boost::asio::serial_port_base::flow_control(
        boost::asio::serial_port_base::flow_control::none), ec);
    if (ec) {
        closeQuietly(serial_);
        return false;
    }

    if (!applyNativePortTuning(serial_, ec)) {
        closeQuietly(serial_);
        return false;
    }

    std::this_thread::sleep_for(kUsbSettleDelay);
    return true;
}

bool YsGripper::reopenPort(boost::system::error_code& ec)
{
    if (serial_.is_open()) {
        clearNativeCommError(serial_);
        purgeNativeBuffers(serial_);
        serial_.close(ec);
    }
    ec.clear();
    std::this_thread::sleep_for(kReopenDelay);
    return openAndConfigure(ec);
}

void YsGripper::markDisconnected()
{
    if (serial_.is_open()) {
        closeQuietly(serial_);
    }
    connected_ = false;
    enabled_ = false;
}
