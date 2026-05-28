#include "log_udp_send.h"

#include "esp_system.h"
#include "lwip/udp.h"
#include "lwip/ip_addr.h"
#include "esp_log.h"

#include <cstring>

static const char* TAG = "LogUDP";

// UDP配置
#define LOG_UDP_PORT 8081
#define LOG_UDP_TARGET_IP "192.168.1.100" // 目标PC IP地址

// 缓存上一次的摇杆值和按键状态
static JoystickData last_joystick[2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
static ButtonData last_buttons[2] = {0};

// UDP套接字
static udp_pcb* log_udp_pcb = nullptr;
static ip_addr_t log_target_ip;

// 初始化日志UDP发送
bool log_udp_init(void) {
    // 创建UDP PCB
    log_udp_pcb = udp_new();
    if (!log_udp_pcb) {
        ESP_LOGE(TAG, "Failed to create UDP PCB");
        return false;
    }

    // 设置目标IP地址
    ipaddr_aton(LOG_UDP_TARGET_IP, &log_target_ip);

    ESP_LOGI(TAG, "Log UDP initialized successfully");
    return true;
}

// 发送UDP数据包
static void send_udp_packet(const std::string& data) {
    if (!log_udp_pcb) {
        ESP_LOGE(TAG, "UDP PCB not initialized");
        return;
    }

    // 创建数据包
    struct pbuf* p = pbuf_alloc(PBUF_TRANSPORT, data.length(), PBUF_RAM);
    if (!p) {
        ESP_LOGE(TAG, "Failed to allocate pbuf");
        return;
    }

    // 复制数据到pbuf
    memcpy(p->payload, data.c_str(), data.length());

    // 发送数据包
    err_t err = udp_sendto(log_udp_pcb, p, &log_target_ip, LOG_UDP_PORT);
    if (err != ERR_OK) {
        ESP_LOGE(TAG, "Failed to send UDP packet: %d", err);
    }

    // 释放pbuf
    pbuf_free(p);
}

// 发送摇杆日志
void send_joystick_log(uint8_t controller_id, const JoystickData& joystick) {
    // 构造JSON字符串
    std::string log = R"({
  "node_type": "vr_controller",
  "controller_id": )" + std::to_string(controller_id) + R"(,
  "timestamp": )" + std::to_string(esp_timer_get_time()) + R"(,
  "log_type": "joystick",
  "joystick": { "x": )" + std::to_string(joystick.x) + R"(, "y": )" + std::to_string(joystick.y) + R"( },
  "level": "INFO"
})";

    send_udp_packet(log);
}

// 发送按键日志
void send_button_log(uint8_t controller_id, const ButtonData& buttons) {
    // 构造JSON字符串
    std::string log = R"({
  "node_type": "vr_controller",
  "controller_id": )" + std::to_string(controller_id) + R"(,
  "timestamp": )" + std::to_string(esp_timer_get_time()) + R"(,
  "log_type": "button",
  "buttons": { "trigger": )" + std::to_string(buttons.trigger) + R"(, "grip": )" + std::to_string(buttons.grip) + R"(, "a": )" + std::to_string(buttons.a) + R"(, "x": )" + std::to_string(buttons.x) + R"(, "b": )" + std::to_string(buttons.b) + R"(, "y": )" + std::to_string(buttons.y) + R"(, "menu": )" + std::to_string(buttons.menu) + R"(, "joystick_click": )" + std::to_string(buttons.joystick_click) + R"( },
  "level": "INFO"
})";

    send_udp_packet(log);
}

// 检查并发送摇杆日志（带防抖）
void check_and_send_joystick_log(uint8_t controller_id, float x, float y) {
    if (controller_id >= 2) {
        return;
    }

    // 检查数值变化是否超过±0.05
    if (fabs(x - last_joystick[controller_id].x) > 0.05f || 
        fabs(y - last_joystick[controller_id].y) > 0.05f) {
        
        // 更新缓存值
        last_joystick[controller_id].x = x;
        last_joystick[controller_id].y = y;
        
        // 发送日志
        JoystickData joystick = {x, y};
        send_joystick_log(controller_id, joystick);
    }
}

// 检查并发送按键日志（状态变化时）
void check_and_send_button_log(uint8_t controller_id, const ButtonData& buttons) {
    if (controller_id >= 2) {
        return;
    }

    // 检查按键状态是否变化
    if (buttons.trigger != last_buttons[controller_id].trigger ||
        buttons.grip != last_buttons[controller_id].grip ||
        buttons.a != last_buttons[controller_id].a ||
        buttons.x != last_buttons[controller_id].x ||
        buttons.b != last_buttons[controller_id].b ||
        buttons.y != last_buttons[controller_id].y ||
        buttons.menu != last_buttons[controller_id].menu ||
        buttons.joystick_click != last_buttons[controller_id].joystick_click) {
        
        // 更新缓存状态
        last_buttons[controller_id] = buttons;
        
        // 发送日志
        send_button_log(controller_id, buttons);
    }
}
