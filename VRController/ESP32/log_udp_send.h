#pragma once

#include <cstdint>
#include <string>

// 摇杆数据结构体
typedef struct {
    float x;
    float y;
} JoystickData;

// 按键数据结构体
typedef struct {
    bool trigger;
    bool grip;
    bool a;
    bool x;
    bool b;
    bool y;
    bool menu;
    bool joystick_click;
} ButtonData;

// 初始化日志UDP发送
bool log_udp_init(void);

// 发送摇杆日志
void send_joystick_log(uint8_t controller_id, const JoystickData& joystick);

// 发送按键日志
void send_button_log(uint8_t controller_id, const ButtonData& buttons);

// 检查并发送摇杆日志（带防抖）
void check_and_send_joystick_log(uint8_t controller_id, float x, float y);

// 检查并发送按键日志（状态变化时）
void check_and_send_button_log(uint8_t controller_id, const ButtonData& buttons);
