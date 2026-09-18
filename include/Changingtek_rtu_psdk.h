#ifndef CHANGINGTEK_RTU_PSDK_H
#define CHANGINGTEK_RTU_PSDK_H

#include <string>
#include <vector>
#include <cstdint>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#ifdef _WIN32
#include <windows.h>
typedef HANDLE SerialHandle;
#define INVALID_SERIAL_HANDLE INVALID_HANDLE_VALUE
#else
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <cstring>
typedef int SerialHandle;
#define INVALID_SERIAL_HANDLE -1
#endif
#include <iostream>

// -----------------------------
// 寄存器地址映射 (保持寄存器, 功能码 0x03/0x06/0x10)
// -----------------------------

// 使能 / 失能
const uint16_t REG_ENABLE                = 0x0100;

// 临时位置模式参数 (写入)
const uint16_t REG_TMP_POS_H             = 0x0102;  // 目标位置高16位 (以CTAG2F120s为例，行程120mm，范围0-12000，以此类推)
const uint16_t REG_TMP_POS_L             = 0x0103;  // 目标位置低16位
const uint16_t REG_TMP_SPEED             = 0x0104;  // 速度百分比 0~100 (% of max speed @ 0x0305)
const uint16_t REG_TMP_FORCE             = 0x0105;  // 力矩/电流百分比 0~100 (% of max torque @ 0x0306)
const uint16_t REG_TMP_ACCEL             = 0x0106;  // 加速度百分比 0~100 (% of max accel @ 0x0307)
const uint16_t REG_TMP_DECEL             = 0x0107;  // 减速度百分比 0~100 (% of max decel @ 0x0308)
const uint16_t REG_TMP_TRIGGER           = 0x0108;  // 触发信号: 0:空闲, 1:触发运动

// 多点运动模式 (写入)
const uint16_t REG_CMD_UPDATE_MODE       = 0x010F;  // 0: 立即更新, 1: 忽略更新直到运动结束
const uint16_t REG_MULTI_MODE            = 0x0110;  // 0: 顺序执行, 1: 循环执行, 2: 指定点位
const uint16_t REG_MULTI_START_SEG       = 0x0111;  // 起始段号
const uint16_t REG_MULTI_END_SEG         = 0x0112;  // 结束段号
const uint16_t REG_MULTI_RESUME_POLICY   = 0x0113;  // 0: 继续剩余段, 1: 从头重新开始
const uint16_t REG_MULTI_LOOP_COUNT      = 0x0114;  // 循环次数, 0xFFFF 表示无限循环
const uint16_t REG_MULTI_SELECT_SEG      = 0x0116;  // 指定段号 (当 REG_MULTI_MODE == 2 时有效)
const uint16_t REG_MULTI_TRIGGER         = 0x0117;  // 触发信号: 0:空闲, 1:触发
const uint16_t REG_MULTI_PAUSE           = 0x0118;  // 暂停信号: 0:空闲, 1:暂停

// -----------------------------
// 状态 / 反馈 (只读, 功能码 0x03)
// -----------------------------
const uint16_t REG_TORQUE_REACHED        = 0x0601;  // 0/1: 力控到位
const uint16_t REG_POS_REACHED           = 0x0602;  // 0/1: 位置到位
const uint16_t REG_SPEED_MAX_REACHED     = 0x0603;  // 0/1: 达到最大速度
const uint16_t REG_READY                 = 0x0604;  // 0/1: 就绪 (力控 OR 位置到位)
const uint16_t REG_CURR_LOOP_COUNT       = 0x0606;  // 当前循环计数 (多点运动中)
const uint16_t REG_CURR_SEG              = 0x0607;  // 当前运行段号
const uint16_t REG_SPEED_FB              = 0x060B;  // 速度反馈
const uint16_t REG_CURRENT_FB            = 0x060C;  // 电流/力矩反馈
const uint16_t REG_POS_FB_H              = 0x060D;  // 32位位置反馈高16位
const uint16_t REG_POS_FB_L              = 0x060E;  // 32位位置反馈低16位
const uint16_t REG_ALARM                 = 0x0612;  // 报警位掩码: 0x01过温, 0x02堵转, 0x04超速, 0x08初始化错误, 0x10限位, 0x20掉电
const uint16_t REG_PARAM_CHANGED         = 0x0614;  // 0/1: 存在未保存的参数

/**
 * Changingtek_rtu_psdk 类
 * 
 * 封装了基于 Modbus RTU (RS-485) 的执行器控制协议。
 * 提供了连接、使能、临时位置控制、多点运动控制以及状态读取功能。
 */
class Changingtek_rtu_psdk {
public:
    /**
     * 构造函数
     * @param port 串口名称，例如 "COM4"
     * @param slave_id Modbus 从站地址 (默认 1)
     * @param baudrate 波特率 (默认 115200)
     * @param timeout 读写超时时间 (秒, 默认 0.3)
     */
    Changingtek_rtu_psdk(const std::string& port, int slave_id = 1, int baudrate = 115200, double timeout = 0.3);
    ~Changingtek_rtu_psdk();

    /**
     * 连接串口
     * @return 成功返回 true，失败返回 false
     */
    bool connect();

    /**
     * 断开串口连接
     */
    void disconnect();

    // ------------- 使能 / 失能 -------------
    
    /**
     * 使能或失能执行器
     * @param enable true 为使能，false 为失能
     */
    void enable(bool enable = true);

    // ------------- 临时位置模式 (Temporary-Zone Motion) -------------
    
    /**
     * 设置临时目标位置 (毫米)
     * @param position_mm 位置值 (0..0xFFFFFFFF)
     */
    void set_temp_position_mm(int position_mm);

    /**
     * 设置临时速度百分比
     * @param speed_pct 速度百分比 (0~100)
     */
    void set_temp_speed_pct(int speed_pct);

    /**
     * 设置临时力矩/电流百分比
     * @param force_pct 力矩百分比 (0~100)
     */
    void set_temp_force_pct(int force_pct);

    /**
     * 设置临时加速度
     * @param accel 加速度值百分比 (0~100)
     */
    void set_temp_accel(int accel_pct);

    /**
     * 设置临时减速度
     * @param decel_pct 减速度值百分比 (0~100)
     */
    void set_temp_decel(int decel_pct);

    /**
     * 触发临时位置运动
     * 发送触发信号，执行器将按照设定的临时参数开始运动
     */
    void trigger_temp_move();
    
    /**
     * 便捷函数：设置所有临时参数并(可选)触发运动
     * @param position_mm 目标位置。以CTAG2F120s为例，行程120mm，范围0-12000，以此类推。
     * @param speed_pct 速度百分比 (默认 100)
     * @param force_pct 力矩百分比 (默认 60)
     * @param accel_pct 加速度值百分比 (默认 100)
     * @param decel_pct 减速度值百分比 (默认 100)
     * @param trigger 是否立即触发 (默认 true)
     */
    void temp_move(int position_mm, int speed_pct = 100, int force_pct = 60, int accel_pct = 100, int decel_pct = 100, bool trigger = true);

    // ------------- 多点运动模式 (Multipoint Motion) -------------
    
    /**
     * 设置命令更新模式
     * @param mode 0: 立即更新; 1: 忽略更新直到运动结束
     */
    void set_cmd_update_mode(int mode);

    /**
     * 设置多点运动模式
     * @param mode 0: 顺序执行, 1: 循环执行, 2: 指定点位
     */
    void set_multi_mode(int mode);

    /**
     * 设置多点运动的起始和结束段
     * @param start_seg 起始段号
     * @param end_seg 结束段号
     */
    void set_multi_range(int start_seg, int end_seg);

    /**
     * 设置多点运动恢复策略
     * @param policy 0: 继续剩余段; 1: 从头重新开始
     */
    void set_multi_resume_policy(int policy);

    /**
     * 设置多点循环次数
     * @param count 循环次数 (0~0xFFFF, 0xFFFF表示无限循环)
     */
    void set_multi_loop_count(int count);

    /**
     * 设置指定执行的段号 (仅当模式为2时有效)
     * @param seg 段号
     */
    void set_multi_select_segment(int seg);

    /**
     * 触发多点运动
     */
    void trigger_multi();

    /**
     * 暂停多点运动
     */
    void pause_multi();

    // ------------- 状态 / 反馈 (Status / Feedback) -------------
    
    bool torque_reached();      // 是否力控到位
    bool position_reached();    // 是否位置到位
    bool speed_max_reached();   // 是否达到最大速度
    bool ready();               // 是否就绪 (力控或位置到位)
    
    int current_loop_count();   // 当前循环计数
    int current_segment();      // 当前运行段号
    int feedback_position();    // 位置反馈值
    int feedback_speed();       // 速度反馈值
    int feedback_current();     // 电流/力矩反馈值
    int read_alarm();           // 读取报警码
    bool param_changed();       // 是否有参数未保存

    // ------------- 工具函数 (Utilities) -------------
    
    /**
     * 等待直到就绪 (ready 为 true)
     * @param timeout 超时时间 (秒)
     * @param poll 轮询间隔 (秒)
     * @return 成功返回 true, 超时返回 false
     */
    bool wait_until_ready(double timeout = 5.0, double poll = 0.02);
    
    /**
     * 等待直到位置到位或力控到位
     * @param timeout 超时时间 (秒)
     * @param poll 轮询间隔 (秒)
     * @return 字符串: "position" (位置到位), "torque" (力控到位), 或 "timeout" (超时)
     */
    std::string wait_until_pos_or_torque(double timeout = 5.0, double poll = 0.02);

private:
    std::string port_name;
    int slave_id;
    int baudrate;
    double timeout;
    SerialHandle hSerial;

    // Modbus 辅助函数
    void _w1(uint16_t addr, uint16_t value);
    void _wn(uint16_t addr, const std::vector<uint16_t>& values);
    std::vector<uint16_t> _r(uint16_t addr, int count = 1);
    bool _read_bool(uint16_t addr);

    // 串口通信底层函数
    void send_frame(const std::vector<uint8_t>& frame);
    std::vector<uint8_t> receive_response(int expected_bytes);
    uint16_t calculate_crc(const std::vector<uint8_t>& data);
    
    void throw_error(const std::string& msg);
};

#endif // CHANGINGTEK_RTU_PSDK_H
