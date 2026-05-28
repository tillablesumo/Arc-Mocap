#pragma once

#include <cstdint>
#include <string>
#include <windows.h>

#pragma pack(push, 1)
struct MocapPacket {
    uint8_t node_id;
    uint8_t seq;
    uint64_t timestamp;
    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
    float mag_x, mag_y, mag_z;
    uint16_t buttons;
    float joystick_x;
    float joystick_y;
};
#pragma pack(pop)

static_assert(sizeof(MocapPacket) == 54, "MocapPacket size should be 54 bytes");