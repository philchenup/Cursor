#ifndef YSGRIPPER_PROTOCOL_H
#define YSGRIPPER_PROTOCOL_H

#include <cstddef>
#include <cstdint>
#include <vector>

namespace ys_gripper {

inline uint8_t checksum(const uint8_t* data, std::size_t n)
{
    unsigned sum = 0;
    for (std::size_t i = 0; i < n; ++i) {
        sum += data[i];
    }
    return static_cast<uint8_t>(sum & 0xFFu);
}

// Packet: [0xEB][0x90][0x01][0x03][0x11][speed_lo][speed_hi][checksum]
inline std::vector<uint8_t> buildOpenFrame(uint16_t speed)
{
    constexpr uint8_t b2 = 0x01;
    constexpr uint8_t b3 = 0x03;
    constexpr uint8_t b4 = 0x11;
    const uint8_t speed_lo = static_cast<uint8_t>(speed & 0x00FFu);
    const uint8_t speed_hi = static_cast<uint8_t>(speed >> 8u);
    const uint8_t payload[] = { b2, b3, b4, speed_lo, speed_hi };
    return { 0xEB, 0x90, b2, b3, b4, speed_lo, speed_hi,
             checksum(payload, sizeof(payload)) };
}

// Packet: [0xEB][0x90][0x01][0x05][0x10]
//         [speed_lo][speed_hi][power_lo][power_hi][checksum]
inline std::vector<uint8_t> buildCloseFrame(uint16_t speed, uint16_t power)
{
    constexpr uint8_t b2 = 0x01;
    constexpr uint8_t b3 = 0x05;
    constexpr uint8_t b4 = 0x10;
    const uint8_t speed_lo = static_cast<uint8_t>(speed & 0x00FFu);
    const uint8_t speed_hi = static_cast<uint8_t>(speed >> 8u);
    const uint8_t power_lo = static_cast<uint8_t>(power & 0x00FFu);
    const uint8_t power_hi = static_cast<uint8_t>(power >> 8u);
    const uint8_t payload[] = { b2, b3, b4, speed_lo, speed_hi, power_lo, power_hi };
    return { 0xEB, 0x90, b2, b3, b4, speed_lo, speed_hi, power_lo, power_hi,
             checksum(payload, sizeof(payload)) };
}

} // namespace ys_gripper

#endif // YSGRIPPER_PROTOCOL_H
