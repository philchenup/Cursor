#include "YsGripper.h"
#include "YsGripperProtocol.h"

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

} // namespace

YsGripper::YsGripper(const std::string& port,
    uint16_t           speed,
    uint16_t           power,
    QObject* parent)
    : IGripper(parent)
    , port_(port)
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
    disconnectSerial(/*emit_signals=*/false);
}

// ========================================================================== //
//  Setters
// ========================================================================== //

void YsGripper::setPort(const std::string& port)
{
    bool busy = false;
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (connected_ || serial_.is_open()) {
            busy = true;
        } else {
            port_ = port;
        }
    }
    if (busy) {
        emit sendErrorMsg("Cannot change port while connected. Disconnect first.");
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
#if defined(_WIN32)
    for (int i = 1; i <= 256; ++i) {
        const std::string name = "COM" + std::to_string(i);
        if (portExists(name)) {
            emit sendSearchCom(i);
        }
    }
#else
    const char* prefixes[] = { "/dev/ttyUSB", "/dev/ttyACM" };
    for (const char* prefix : prefixes) {
        for (int i = 0; i < 32; ++i) {
            const std::string name = std::string(prefix) + std::to_string(i);
            if (portExists(name)) {
                emit sendSearchCom(i);
                emit sendInfoMsg(QString::fromStdString("Found port: " + name));
            }
        }
    }
#endif
}

void YsGripper::connect()
{
    IoResult result;
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (connected_ && serial_.is_open()) {
            return;
        }

        if (port_.empty()) {
            result.error = "Connect failed: serial port is empty.";
        } else {
            boost::system::error_code ec;
            if (serial_.is_open()) {
                serial_.close(ec);
                ec.clear();
            }

            if (!openAndConfigure(ec)) {
                connected_ = false;
                enabled_ = false;
                result.error = formatAsioError("Connect", ec);
                result.lost_connection = true;
            } else {
                connected_ = true;
                result.ok = true;
                result.info = "YsGripper connected on " + QString::fromStdString(port_);
            }
        }
    }

    if (result.ok) {
        emit sendConnectStatus(true);
        emit sendInfoMsg(result.info);
    } else if (!result.error.isEmpty()) {
        emit sendConnectStatus(false);
        emit sendErrorMsg(result.error);
    }
}

void YsGripper::disconnect()
{
    disconnectSerial(/*emit_signals=*/true);
}

void YsGripper::enable()
{
    QString error;
    bool ok = false;
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (!connected_ || !serial_.is_open()) {
            error = "Cannot enable: gripper not connected.";
        } else {
            enabled_ = true;
            ok = true;
        }
    }

    if (ok) {
        emit sendEnableStatus(true);
        emit sendInfoMsg("YsGripper enabled.");
    } else {
        emit sendErrorMsg(error);
    }
}

void YsGripper::disenable()
{
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        enabled_ = false;
    }
    emit sendEnableStatus(false);
    emit sendInfoMsg("YsGripper disabled.");
}

void YsGripper::open_gripper()
{
    QString ready_error;
    uint16_t speed = 0;
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (!checkReadyLocked(&ready_error)) {
            speed = 0;
        } else {
            speed = speed_;
        }
    }
    if (!ready_error.isEmpty()) {
        emit sendErrorMsg(ready_error);
        return;
    }

    const IoResult result = sendFrame(ys_gripper::buildOpenFrame(speed));
    emitIoResult(result);
    if (result.ok) {
        emit sendInfoMsg("Gripper opened.");
    }
}

void YsGripper::close_gripper()
{
    QString ready_error;
    uint16_t speed = 0;
    uint16_t power = 0;
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (!checkReadyLocked(&ready_error)) {
            speed = 0;
        } else {
            speed = speed_;
            power = power_;
        }
    }
    if (!ready_error.isEmpty()) {
        emit sendErrorMsg(ready_error);
        return;
    }

    const IoResult result = sendFrame(ys_gripper::buildCloseFrame(speed, power));
    emitIoResult(result);
    if (result.ok) {
        emit sendInfoMsg("Gripper closed.");
    }
}

void YsGripper::disconnectSerial(bool emit_signals)
{
    bool was_connected = false;
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        was_connected = connected_ || serial_.is_open();
        if (serial_.is_open()) {
            boost::system::error_code ec;
            clearNativeCommError();
            purgeNativeBuffers();
            serial_.close(ec);
        }
        connected_ = false;
        enabled_ = false;
    }

    if (emit_signals && was_connected && !shutting_down_) {
        emit sendConnectStatus(false);
        emit sendEnableStatus(false);
        emit sendInfoMsg("YsGripper disconnected.");
    }
}

bool YsGripper::checkReadyLocked(QString* error)
{
    if (!connected_ || !serial_.is_open()) {
        if (error) {
            *error = QStringLiteral("Gripper not connected.");
        }
        return false;
    }
    if (!enabled_) {
        if (error) {
            *error = QStringLiteral("Gripper not enabled.");
        }
        return false;
    }
    return true;
}

YsGripper::IoResult YsGripper::sendFrame(const std::vector<uint8_t>& frame)
{
    IoResult result;
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (frame.empty()) {
        result.error = "Serial write error: empty frame.";
        return result;
    }

    boost::system::error_code ec;
    for (int attempt = 1; attempt <= kMaxWriteAttempts; ++attempt) {
        if (!serial_.is_open()) {
            if (!reopenPort(ec)) {
                result.error = formatAsioError("Reconnect", ec);
                markDisconnectedLocked();
                result.lost_connection = true;
                return result;
            }
        }

        // Unlatch a previous UART error (ERROR_GEN_FAILURE / system:31) before write.
        clearNativeCommError();
        waitInterCommandGap();

        const std::size_t written = boost::asio::write(
            serial_, boost::asio::buffer(frame), ec);

        if (!ec && written == frame.size()) {
            last_send_ = std::chrono::steady_clock::now();
            result.ok = true;
            return result;
        }

        if (!ec && written != frame.size()) {
            ec = boost::asio::error::eof;
        }

        const QString err = formatAsioError("Serial write", ec);
        const bool recoverable = isRecoverableSerialError(ec);

        if (attempt < kMaxWriteAttempts && recoverable) {
            result.info = err + QString(" (retry %1/%2)").arg(attempt).arg(kMaxWriteAttempts - 1);
            clearNativeCommError();
            purgeNativeBuffers();

            // First retry: clear the driver error latch. Later retries reopen the port.
            if (attempt >= 2) {
                boost::system::error_code rec_ec;
                if (!reopenPort(rec_ec)) {
                    result.error = formatAsioError("Reconnect", rec_ec);
                    markDisconnectedLocked();
                    result.lost_connection = true;
                    return result;
                }
            }
            ec.clear();
            continue;
        }

        result.error = err;
        if (recoverable) {
            markDisconnectedLocked();
            result.lost_connection = true;
        }
        return result;
    }

    result.error = "Serial write error: retries exhausted.";
    markDisconnectedLocked();
    result.lost_connection = true;
    return result;
}

void YsGripper::emitIoResult(const IoResult& result)
{
    if (!result.info.isEmpty()) {
        emit sendInfoMsg(result.info);
    }
    if (!result.error.isEmpty()) {
        emit sendErrorMsg(result.error);
    }
    if (result.lost_connection && !shutting_down_) {
        emit sendConnectStatus(false);
        emit sendEnableStatus(false);
    }
}

bool YsGripper::portExists(const std::string& name) const
{
#if defined(_WIN32)
    // QueryDosDevice does not open the handle, so search will not reset USB-UART chips.
    char buf[256] = {};
    return QueryDosDeviceA(name.c_str(), buf, static_cast<DWORD>(sizeof(buf))) != 0;
#else
    return ::access(name.c_str(), F_OK) == 0;
#endif
}

std::string YsGripper::asioPortName(const std::string& port) const
{
#if defined(_WIN32)
    // CreateFile requires \\.\COMx for COM10+; the prefix is also safer for COM1-9.
    if (port.rfind("\\\\.\\", 0) == 0) {
        return port;
    }
    return "\\\\.\\" + port;
#else
    return port;
#endif
}

bool YsGripper::openAndConfigure(boost::system::error_code& ec)
{
    ec.clear();
    serial_.open(asioPortName(port_), ec);
    if (ec) {
        return false;
    }

    auto closeQuietly = [this]() {
        boost::system::error_code ignore;
        serial_.close(ignore);
    };

    serial_.set_option(boost::asio::serial_port_base::baud_rate(115200), ec);
    if (ec) {
        closeQuietly();
        return false;
    }
    serial_.set_option(boost::asio::serial_port_base::character_size(8), ec);
    if (ec) {
        closeQuietly();
        return false;
    }
    serial_.set_option(boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none), ec);
    if (ec) {
        closeQuietly();
        return false;
    }
    serial_.set_option(boost::asio::serial_port_base::stop_bits(
        boost::asio::serial_port_base::stop_bits::one), ec);
    if (ec) {
        closeQuietly();
        return false;
    }
    serial_.set_option(boost::asio::serial_port_base::flow_control(
        boost::asio::serial_port_base::flow_control::none), ec);
    if (ec) {
        closeQuietly();
        return false;
    }

    if (!applyNativePortTuning(ec)) {
        closeQuietly();
        return false;
    }

    std::this_thread::sleep_for(kUsbSettleDelay);
    return true;
}

bool YsGripper::applyNativePortTuning(boost::system::error_code& ec)
{
    ec.clear();
    if (!serial_.is_open()) {
        ec = boost::asio::error::bad_descriptor;
        return false;
    }

#if defined(_WIN32)
    const HANDLE handle = serial_.native_handle();
    if (handle == INVALID_HANDLE_VALUE) {
        ec = boost::system::error_code(kWinErrorInvalidHandle,
            boost::asio::error::get_system_category());
        return false;
    }

    // Boost.Asio's win_iocp_handle_service uses overlapped WriteFile. USB-UART
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
    clearNativeCommError();
    purgeNativeBuffers();
    return true;
#else
    const int fd = serial_.native_handle();
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
    purgeNativeBuffers();
    return true;
#endif
}

void YsGripper::clearNativeCommError()
{
    if (!serial_.is_open()) {
        return;
    }
#if defined(_WIN32)
    DWORD errors = 0;
    COMSTAT stat{};
    ClearCommError(serial_.native_handle(), &errors, &stat);
#else
    (void)0;
#endif
}

void YsGripper::purgeNativeBuffers()
{
    if (!serial_.is_open()) {
        return;
    }
#if defined(_WIN32)
    PurgeComm(serial_.native_handle(),
        PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
#else
    tcflush(serial_.native_handle(), TCIOFLUSH);
#endif
}

bool YsGripper::reopenPort(boost::system::error_code& ec)
{
    if (serial_.is_open()) {
        clearNativeCommError();
        purgeNativeBuffers();
        serial_.close(ec);
    }
    ec.clear();
    std::this_thread::sleep_for(kReopenDelay);
    return openAndConfigure(ec);
}

void YsGripper::markDisconnectedLocked()
{
    if (serial_.is_open()) {
        boost::system::error_code ec;
        serial_.close(ec);
    }
    connected_ = false;
    enabled_ = false;
}

void YsGripper::waitInterCommandGap()
{
    if (last_send_.time_since_epoch().count() == 0) {
        return;
    }
    const auto elapsed = std::chrono::steady_clock::now() - last_send_;
    if (elapsed < kInterCommandGap) {
        std::this_thread::sleep_for(kInterCommandGap - elapsed);
    }
}

bool YsGripper::isRecoverableSerialError(const boost::system::error_code& ec) const
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

QString YsGripper::formatAsioError(const char* op, const boost::system::error_code& ec) const
{
    return QString("%1 error: %2 [%3:%4]")
        .arg(QLatin1String(op),
             systemMessage(ec),
             QLatin1String(ec.category().name()),
             QString::number(ec.value()));
}

