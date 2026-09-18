#include "Changingtek_rtu_psdk.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <sstream>
#include <iomanip>
#ifndef _WIN32
#include <sys/select.h>
#endif

// 辅助函数: 字节交换 (如果需要处理大小端问题，通常Modbus是大端，主机可能是小端)
// 本实现中在组包时手动处理了高低字节，因此不需要额外的 swap 函数
static uint16_t swap_bytes(uint16_t val) {
    return (val << 8) | (val >> 8);
}

Changingtek_rtu_psdk::Changingtek_rtu_psdk(const std::string& port, int slave_id, int baudrate, double timeout)
    : port_name(port), slave_id(slave_id), baudrate(baudrate), timeout(timeout), hSerial(INVALID_SERIAL_HANDLE) {
}

Changingtek_rtu_psdk::~Changingtek_rtu_psdk() {
    disconnect();
}

bool Changingtek_rtu_psdk::connect() {
#ifdef _WIN32
    // 打开串口
    // 使用 \\.\COMxx 格式以支持 COM10 以上的端口
    std::string full_port_name = port_name;
    if (full_port_name.find("\\\\.\\") == std::string::npos) {
        full_port_name = "\\\\.\\" + port_name;
    }
    
    hSerial = CreateFileA(full_port_name.c_str(),
                          GENERIC_READ | GENERIC_WRITE,
                          0,
                          NULL,
                          OPEN_EXISTING,
                          0,
                          NULL);

    if (hSerial == INVALID_HANDLE_VALUE) {
        std::cerr << "Error opening serial port: " << port_name << " (Error: " << GetLastError() << ")" << std::endl;
        return false;
    }

    // 配置串口参数
    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(hSerial, &dcbSerialParams)) {
        std::cerr << "Error getting serial state" << std::endl;
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
        return false;
    }

    dcbSerialParams.BaudRate = baudrate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    if (!SetCommState(hSerial, &dcbSerialParams)) {
        std::cerr << "Error setting serial state" << std::endl;
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
        return false;
    }

    // 设置超时
    COMMTIMEOUTS timeouts = {0};
    DWORD timeout_ms = static_cast<DWORD>(timeout * 1000);
    
    // ReadIntervalTimeout: 字节间最大间隔时间
    timeouts.ReadIntervalTimeout = 50; 
    // 总读取超时 = Multiplier * 字节数 + Constant
    timeouts.ReadTotalTimeoutConstant = timeout_ms;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = timeout_ms;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    if (!SetCommTimeouts(hSerial, &timeouts)) {
        std::cerr << "Error setting serial timeouts" << std::endl;
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
        return false;
    }

    return true;
#else
    // Linux implementation
    hSerial = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (hSerial < 0) {
        std::cerr << "Error opening serial port: " << port_name << " (Error: " << strerror(errno) << ")" << std::endl;
        return false;
    }

    struct termios tty;
    if (tcgetattr(hSerial, &tty) != 0) {
        std::cerr << "Error from tcgetattr: " << strerror(errno) << std::endl;
        close(hSerial);
        hSerial = INVALID_SERIAL_HANDLE;
        return false;
    }

    cfsetospeed(&tty, B115200); // 默认先设为 115200, 后面根据参数调整
    cfsetispeed(&tty, B115200);

    // 设置自定义波特率
    speed_t speed;
    switch (baudrate) {
        case 9600:   speed = B9600; break;
        case 19200:  speed = B19200; break;
        case 38400:  speed = B38400; break;
        case 57600:  speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        default:     speed = B115200; break;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                         // disable break processing
    tty.c_lflag = 0;                                // no signaling chars, no echo,
                                                    // no canonical processing
    tty.c_oflag = 0;                                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;                            // read doesn't block
    tty.c_cc[VTIME] = 5;                            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);                // ignore modem controls,
                                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);              // shut off parity
    tty.c_cflag &= ~CSTOPB;                         // 1 stop bit
    // tty.c_cflag &= ~CRTSCTS;                        // no flow control

    if (tcsetattr(hSerial, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
        close(hSerial);
        hSerial = INVALID_SERIAL_HANDLE;
        return false;
    }
    return true;
#endif
}

void Changingtek_rtu_psdk::disconnect() {
    if (hSerial != INVALID_SERIAL_HANDLE) {
#ifdef _WIN32
        CloseHandle(hSerial);
#else
        close(hSerial);
#endif
        hSerial = INVALID_SERIAL_HANDLE;
    }
}

// -----------------------------
// Modbus 核心实现
// -----------------------------

// 计算 Modbus CRC16
uint16_t Changingtek_rtu_psdk::calculate_crc(const std::vector<uint8_t>& data) {
    uint16_t crc = 0xFFFF;
    for (uint8_t byte : data) {
        crc ^= byte;
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void Changingtek_rtu_psdk::send_frame(const std::vector<uint8_t>& frame) {
    if (hSerial == INVALID_SERIAL_HANDLE) {
        if (!connect()) throw_error("Port not open and failed to connect");
    }

#ifdef _WIN32
    // 清空缓冲区
    PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);

    DWORD bytes_written;
    if (!WriteFile(hSerial, frame.data(), static_cast<DWORD>(frame.size()), &bytes_written, NULL)) {
        throw_error("Write failed");
    }
    if (bytes_written != frame.size()) {
        throw_error("Write incomplete");
    }
#else
    tcflush(hSerial, TCIOFLUSH); // 清空缓冲区
    ssize_t bytes_written = write(hSerial, frame.data(), frame.size());
    if (bytes_written < 0) {
        throw_error("Write failed: " + std::string(strerror(errno)));
    }
    if (static_cast<size_t>(bytes_written) != frame.size()) {
        throw_error("Write incomplete");
    }
#endif
}

std::vector<uint8_t> Changingtek_rtu_psdk::receive_response(int expected_min_bytes) {
    std::vector<uint8_t> buffer;
    buffer.reserve(256);
    uint8_t tmp_buf[256];
    
#ifdef _WIN32
    DWORD bytes_read;
    // 简单的读取循环
    // 由于设置了 SetCommTimeouts，ReadFile 会等待直到数据到达或超时
    if (!ReadFile(hSerial, tmp_buf, 256, &bytes_read, NULL)) {
        throw_error("Read failed");
    }
    
    if (bytes_read == 0) {
        throw_error("Read timeout (no data)");
    }

    for (DWORD i = 0; i < bytes_read; i++) {
        buffer.push_back(tmp_buf[i]);
    }
#else
    // Linux implementation using select() for timeout
    fd_set set;
    struct timeval timeout_tv;
    
    // 初始化超时时间
    timeout_tv.tv_sec = static_cast<long>(timeout);
    timeout_tv.tv_usec = static_cast<long>((timeout - static_cast<long>(timeout)) * 1000000);

    FD_ZERO(&set);
    FD_SET(hSerial, &set);

    // 等待数据可读
    int rv = select(hSerial + 1, &set, NULL, NULL, &timeout_tv);
    if (rv == -1) {
        throw_error("Select failed: " + std::string(strerror(errno)));
    } else if (rv == 0) {
        throw_error("Read timeout (no data)");
    } else {
        // 数据可读，尝试读取
        // 注意：这里可能一次读不完所有数据，对于 Modbus 这种简单的协议，
        // 我们可以尝试循环读取或者依赖 VTIME/VMIN 设置。
        // 为了简单起见，这里尝试尽可能多读，或者直到读够 expected_min_bytes
        // 但由于不知道确切的总长度 (取决于响应类型)，我们这里做一个简单的读取。
        // 如果需要更健壮的实现，应该根据功能码解析头部来确定需要读取多少字节。
        
        // 稍微延时以确保数据帧到达完全 (简单粗暴但有效)
        // usleep(10000); 
        
        ssize_t n = read(hSerial, tmp_buf, sizeof(tmp_buf));
        if (n < 0) {
             throw_error("Read failed: " + std::string(strerror(errno)));
        }
        if (n == 0) {
             throw_error("Read returned 0 (EOF?)");
        }
        for (ssize_t i = 0; i < n; i++) {
            buffer.push_back(tmp_buf[i]);
        }
        
        // 如果读取的数据少于最小预期值，可能需要继续读取
        // 这里为了保持与 Windows 版本逻辑一致（依赖底层超时设置），我们暂不进行复杂的循环读取
        // 除非发现数据确实不够。
    }
#endif

    // 校验 CRC
    if (buffer.size() < 2) throw_error("Response too short");
    
    uint16_t received_crc = buffer[buffer.size() - 2] | (buffer[buffer.size() - 1] << 8);
    std::vector<uint8_t> data_for_crc(buffer.begin(), buffer.end() - 2);
    uint16_t calc_crc = calculate_crc(data_for_crc);

    if (received_crc != calc_crc) {
        // 如果 CRC 错误，可能是数据没收完，在 Linux 上可能需要处理分包。
        // 但对于短帧 Modbus RTU，通常一次 read 就能读完 (波特率较高时)。
        throw_error("CRC Error");
    }

    return buffer;
}

void Changingtek_rtu_psdk::throw_error(const std::string& msg) {
    throw std::runtime_error("Changingtek_rtu_psdk Error: " + msg);
}

// -----------------------------
// Modbus 辅助方法
// -----------------------------

void Changingtek_rtu_psdk::_w1(uint16_t addr, uint16_t value) {
    std::vector<uint8_t> frame;
    frame.push_back(static_cast<uint8_t>(slave_id));
    frame.push_back(0x06); // 功能码: 写单个寄存器
    frame.push_back(addr >> 8);
    frame.push_back(addr & 0xFF);
    frame.push_back(value >> 8);
    frame.push_back(value & 0xFF);
    
    uint16_t crc = calculate_crc(frame);
    frame.push_back(crc & 0xFF);
    frame.push_back(crc >> 8);

    send_frame(frame);
    
    // 响应是请求的回显 (8 字节)
    std::vector<uint8_t> resp = receive_response(8);
    
    // 检查异常响应
    if (resp[1] & 0x80) throw_error("Modbus Exception: " + std::to_string(resp[2]));
}

void Changingtek_rtu_psdk::_wn(uint16_t addr, const std::vector<uint16_t>& values) {
    std::vector<uint8_t> frame;
    frame.push_back(static_cast<uint8_t>(slave_id));
    frame.push_back(0x10); // 功能码: 写多个寄存器
    frame.push_back(addr >> 8);
    frame.push_back(addr & 0xFF);
    
    uint16_t quantity = static_cast<uint16_t>(values.size());
    frame.push_back(quantity >> 8);
    frame.push_back(quantity & 0xFF);
    
    uint8_t byte_count = static_cast<uint8_t>(quantity * 2);
    frame.push_back(byte_count);
    
    for (uint16_t v : values) {
        frame.push_back(v >> 8);
        frame.push_back(v & 0xFF);
    }
    
    uint16_t crc = calculate_crc(frame);
    frame.push_back(crc & 0xFF);
    frame.push_back(crc >> 8);

    send_frame(frame);

    // 响应: SlaveID(1) + Func(1) + Addr(2) + Qty(2) + CRC(2) = 8 字节
    std::vector<uint8_t> resp = receive_response(8);
    
    if (resp[1] & 0x80) throw_error("Modbus Exception: " + std::to_string(resp[2]));
}

std::vector<uint16_t> Changingtek_rtu_psdk::_r(uint16_t addr, int count) {
    std::vector<uint8_t> frame;
    frame.push_back(static_cast<uint8_t>(slave_id));
    frame.push_back(0x03); // 功能码: 读保持寄存器
    frame.push_back(addr >> 8);
    frame.push_back(addr & 0xFF);
    frame.push_back(count >> 8);
    frame.push_back(count & 0xFF);
    
    uint16_t crc = calculate_crc(frame);
    frame.push_back(crc & 0xFF);
    frame.push_back(crc >> 8);

    send_frame(frame);

    // 响应: SlaveID(1) + Func(1) + ByteCount(1) + Data(count*2) + CRC(2)
    // 最小长度 = 3 + 2 + 2 = 7 字节 (当 count=1 时)
    std::vector<uint8_t> resp = receive_response(5 + count * 2);
    
    if (resp[1] & 0x80) throw_error("Modbus Exception: " + std::to_string(resp[2]));
    
    int byte_count = resp[2];
    if (byte_count != count * 2) throw_error("Unexpected byte count in response");

    std::vector<uint16_t> result;
    for (int i = 0; i < count; ++i) {
        uint16_t val = (resp[3 + i*2] << 8) | resp[3 + i*2 + 1];
        result.push_back(val);
    }
    return result;
}

bool Changingtek_rtu_psdk::_read_bool(uint16_t addr) {
    std::vector<uint16_t> res = _r(addr, 1);
    return res[0] != 0;
}

// -----------------------------
// 高级方法实现
// -----------------------------

void Changingtek_rtu_psdk::enable(bool enable) {
    _w1(REG_ENABLE, enable ? 1 : 0);
}

void Changingtek_rtu_psdk::set_temp_position_mm(int position_mm) {
    // 32位位置分为两个16位寄存器写入
    // Python 代码逻辑: 检查是否超出 32 位范围，并拆分为高低位
    // 实际上通常位置可以为负，但参考 Python 代码中的 check:
    // if position_mm < 0 or > 0xFFFFFFFF: raise ValueError
    // 意味着它将位置视为无符号 32 位整数处理
    
    if (position_mm < 0) {
        throw_error("position_mm must be non-negative (based on Python SDK)");
    }
    
    uint32_t pos = static_cast<uint32_t>(position_mm);
    uint16_t hi = (pos >> 16) & 0xFFFF;
    uint16_t lo = pos & 0xFFFF;
    
    std::vector<uint16_t> values = {hi, lo};
    _wn(REG_TMP_POS_H, values);
}

void Changingtek_rtu_psdk::set_temp_speed_pct(int speed_pct) {
    if (speed_pct < 0 || speed_pct > 100) throw_error("speed_pct must be 0..100");
    _w1(REG_TMP_SPEED, static_cast<uint16_t>(speed_pct));
}

void Changingtek_rtu_psdk::set_temp_force_pct(int force_pct) {
    if (force_pct < 0 || force_pct > 100) throw_error("force_pct must be 0..100");
    _w1(REG_TMP_FORCE, static_cast<uint16_t>(force_pct));
}

void Changingtek_rtu_psdk::set_temp_accel(int accel_pct) {
    if (accel_pct < 0 || accel_pct > 100) throw_error("accel_pct must be 0..100");
    _w1(REG_TMP_ACCEL, static_cast<uint16_t>(accel_pct));
}

void Changingtek_rtu_psdk::set_temp_decel(int decel_pct) {
    if (decel_pct < 0 || decel_pct > 100) throw_error("decel_pct must be 0..100");
    _w1(REG_TMP_DECEL, static_cast<uint16_t>(decel_pct));
}

void Changingtek_rtu_psdk::trigger_temp_move() {
    _w1(REG_TMP_TRIGGER, 1);
}

void Changingtek_rtu_psdk::temp_move(int position_mm, int speed_pct, int force_pct, int accel_pct, int decel_pct, bool trigger) {
    // 验证参数
    if (position_mm < 0) throw_error("position_mm must be non-negative");
    if (speed_pct < 0 || speed_pct > 100) throw_error("speed_pct must be 0..100");
    if (force_pct < 0 || force_pct > 100) throw_error("force_pct must be 0..100");
    if (accel_pct < 0 || accel_pct > 100) throw_error("accel_pct must be 0..100");
    if (decel_pct < 0 || decel_pct > 100) throw_error("decel_pct must be 0..100");

    uint32_t pos = static_cast<uint32_t>(position_mm);
    uint16_t hi = (pos >> 16) & 0xFFFF;
    uint16_t lo = pos & 0xFFFF;

    // 优化: 使用一次多寄存器写入 (Function 0x10) 设置所有参数并触发
    // 寄存器从 REG_TMP_POS_H (0x0102) 到 REG_TMP_TRIGGER (0x0108) 是连续的
    std::vector<uint16_t> values;
    values.push_back(hi);                               // 0x0102
    values.push_back(lo);                               // 0x0103
    values.push_back(static_cast<uint16_t>(speed_pct)); // 0x0104
    values.push_back(static_cast<uint16_t>(force_pct)); // 0x0105
    values.push_back(static_cast<uint16_t>(accel_pct)); // 0x0106
    values.push_back(static_cast<uint16_t>(decel_pct)); // 0x0107
    values.push_back(trigger ? 1 : 0);                  // 0x0108

    _wn(REG_TMP_POS_H, values);
}

void Changingtek_rtu_psdk::set_cmd_update_mode(int mode) {
    if (mode != 0 && mode != 1) throw_error("mode must be 0 or 1");
    _w1(REG_CMD_UPDATE_MODE, static_cast<uint16_t>(mode));
}

void Changingtek_rtu_psdk::set_multi_mode(int mode) {
    if (mode < 0 || mode > 2) throw_error("mode must be 0, 1 or 2");
    _w1(REG_MULTI_MODE, static_cast<uint16_t>(mode));
}

void Changingtek_rtu_psdk::set_multi_range(int start_seg, int end_seg) {
    _w1(REG_MULTI_START_SEG, static_cast<uint16_t>(start_seg));
    _w1(REG_MULTI_END_SEG, static_cast<uint16_t>(end_seg));
}

void Changingtek_rtu_psdk::set_multi_resume_policy(int policy) {
    if (policy != 0 && policy != 1) throw_error("policy must be 0 or 1");
    _w1(REG_MULTI_RESUME_POLICY, static_cast<uint16_t>(policy));
}

void Changingtek_rtu_psdk::set_multi_loop_count(int count) {
    if (count < 0 || count > 0xFFFF) throw_error("count must be 0..0xFFFF");
    _w1(REG_MULTI_LOOP_COUNT, static_cast<uint16_t>(count));
}

void Changingtek_rtu_psdk::set_multi_select_segment(int seg) {
    _w1(REG_MULTI_SELECT_SEG, static_cast<uint16_t>(seg));
}

void Changingtek_rtu_psdk::trigger_multi() {
    _w1(REG_MULTI_TRIGGER, 1);
}

void Changingtek_rtu_psdk::pause_multi() {
    _w1(REG_MULTI_PAUSE, 1);
}

// -----------------------------
// 状态读取方法实现
// -----------------------------

bool Changingtek_rtu_psdk::torque_reached() { return _read_bool(REG_TORQUE_REACHED); }
bool Changingtek_rtu_psdk::position_reached() { return _read_bool(REG_POS_REACHED); }
bool Changingtek_rtu_psdk::speed_max_reached() { return _read_bool(REG_SPEED_MAX_REACHED); }
bool Changingtek_rtu_psdk::ready() { return _read_bool(REG_READY); }

int Changingtek_rtu_psdk::current_loop_count() {
    return _r(REG_CURR_LOOP_COUNT, 1)[0];
}

int Changingtek_rtu_psdk::current_segment() {
    return _r(REG_CURR_SEG, 1)[0];
}

int Changingtek_rtu_psdk::feedback_position() {
    std::vector<uint16_t> vals = _r(REG_POS_FB_H, 2);
    uint32_t hi = vals[0];
    uint32_t lo = vals[1];
    return static_cast<int>((hi << 16) | lo);
}

int Changingtek_rtu_psdk::feedback_speed() {
    return static_cast<int>(_r(REG_SPEED_FB, 1)[0]);
}

int Changingtek_rtu_psdk::feedback_current() {
    return static_cast<int>(_r(REG_CURRENT_FB, 1)[0]);
}

int Changingtek_rtu_psdk::read_alarm() {
    return static_cast<int>(_r(REG_ALARM, 1)[0]);
}

bool Changingtek_rtu_psdk::param_changed() {
    return _read_bool(REG_PARAM_CHANGED);
}

// -----------------------------
// 工具方法实现
// -----------------------------

bool Changingtek_rtu_psdk::wait_until_ready(double timeout_sec, double poll_sec) {
    auto start = std::chrono::steady_clock::now();
    auto deadline = start + std::chrono::duration<double>(timeout_sec);
    
    while (std::chrono::steady_clock::now() < deadline) {
        try {
            if (ready()) return true;
        } catch (...) {
            // 忽略临时通信错误
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(poll_sec));
    }
    return false;
}

std::string Changingtek_rtu_psdk::wait_until_pos_or_torque(double timeout_sec, double poll_sec) {
    auto start = std::chrono::steady_clock::now();
    auto deadline = start + std::chrono::duration<double>(timeout_sec);
    
    while (std::chrono::steady_clock::now() < deadline) {
        try {
            if (position_reached()) return "position";
            if (torque_reached()) return "torque";
        } catch (...) {
            // 忽略临时通信错误
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(poll_sec));
    }
    return "timeout";
}
