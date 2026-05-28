#include "mocap_udp_send.hpp"
#include "icm20948_driver.h"
#include "log_udp_send.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "lwip/udp.h"
#include "lwip/ip_addr.h"
#include "esp_log.h"

#include <cstring>

// UdpSender 实现细节
class UdpSender::Impl {
private:
    udp_pcb* pcb;

public:
    Impl() : pcb(nullptr) {}
    
    ~Impl() {
        if (pcb) {
            udp_remove(pcb);
        }
    }
    
    bool init() {
        pcb = udp_new();
        if (!pcb) {
            return false;
        }
        return true;
    }
    
    bool send_to(const MocapPacket& packet, const std::string& target_ip, uint16_t target_port) {
        if (!pcb) {
            return false;
        }
        
        struct pbuf* p = pbuf_alloc(PBUF_TRANSPORT, sizeof(MocapPacket), PBUF_RAM);
        if (!p) {
            return false;
        }
        
        memcpy(p->payload, &packet, sizeof(MocapPacket));
        
        ip_addr_t target_addr;
        ipaddr_aton(target_ip.c_str(), &target_addr);
        
        err_t err = udp_sendto(pcb, p, &target_addr, target_port);
        pbuf_free(p);
        
        return err == ERR_OK;
    }
};

// UdpSender 构造函数和析构函数
UdpSender::UdpSender() : impl(nullptr) {
    impl.reset(new Impl());
}
UdpSender::~UdpSender() = default;

// 初始化UDP
bool UdpSender::init() {
    return impl->init();
}

// 发送UDP数据包到指定IP:端口
bool UdpSender::send_broadcast(const MocapPacket& packet, uint16_t port) {
    // 这里改为发送到固定IP和端口
    return impl->send_to(packet, TARGET_IP, port);
}

// 发送SlimeVR格式数据到指定IP:端口（支持VR控制器）
bool UdpSender::send_slimevr_data(const std::string& target_ip, uint16_t target_port, 
                                 const Quaternion& corrected_q, uint8_t node_id, 
                                 uint16_t buttons, float joystick_x, float joystick_y) {
    // 构建SlimeVR数据包
    struct SlimeVRPacket {
        uint8_t node_id;
        float qx;
        float qy;
        float qz;
        float qw;
        uint16_t buttons;
        float joystick_x;
        float joystick_y;
    } __attribute__((packed));
    
    SlimeVRPacket packet;
    packet.node_id = node_id;
    packet.qx = corrected_q.x;
    packet.qy = corrected_q.y;
    packet.qz = corrected_q.z;
    packet.qw = corrected_q.w;
    packet.buttons = buttons;
    packet.joystick_x = joystick_x;
    packet.joystick_y = joystick_y;
    
    // 此函数不再使用，因为EKF计算移到了Windows端
    return false;
}

// ICM20948数据读取函数（真实传感器读取）
MPU9250RawData read_mpu9250_data(uint8_t node_id) {
    MPU9250RawData data = {0};
    
    // 读取真实ICM20948数据
    ICM20948Data sensor_data = icm20948_read_data();
    
    // 将数据转换为MPU9250RawData格式
    data.gyro_x = sensor_data.gyro_x;
    data.gyro_y = sensor_data.gyro_y;
    data.gyro_z = sensor_data.gyro_z;
    data.accel_x = sensor_data.accel_x;
    data.accel_y = sensor_data.accel_y;
    data.accel_z = sensor_data.accel_z;
    data.mag_x = sensor_data.mag_x;
    data.mag_y = sensor_data.mag_y;
    data.mag_z = sensor_data.mag_z;
    
    return data;
}

// 主发送任务
void mocap_send_task(void* pvParameters) {
    UdpSender sender;
    if (!sender.init()) {
        vTaskDelete(nullptr);
        return;
    }
    
    // 初始化日志UDP发送
    if (!log_udp_init()) {
        ESP_LOGE("VR_CONTROLLER", "Failed to initialize log UDP");
        vTaskDelete(nullptr);
        return;
    }
    
    // 初始化ICM20948传感器
    if (!icm20948_init()) {
        ESP_LOGE("VR_CONTROLLER", "Failed to initialize ICM20948 sensor");
        vTaskDelete(nullptr);
        return;
    }
    
    const int MAX_NODES = 16;
    uint8_t seq[MAX_NODES] = {0};
    
    while (true) {
        for (uint8_t node_id = 0; node_id < MAX_NODES; node_id++) {
            // 读取ICM20948原始数据
            MPU9250RawData raw_data = read_mpu9250_data(node_id);
            
            // 构建原始数据包（包含完整原始数据、时间戳和VR控制数据）
            MocapPacket packet;
            packet.node_id = node_id;
            packet.seq = seq[node_id];
            // 添加高精度时间戳（微秒），使用64位无符号整数，避免溢出
            packet.timestamp = static_cast<uint64_t>(esp_timer_get_time());
            packet.gyro_x = raw_data.gyro_x;
            packet.gyro_y = raw_data.gyro_y;
            packet.gyro_z = raw_data.gyro_z;
            packet.accel_x = raw_data.accel_x;
            packet.accel_y = raw_data.accel_y;
            packet.accel_z = raw_data.accel_z;
            packet.mag_x = raw_data.mag_x;
            packet.mag_y = raw_data.mag_y;
            packet.mag_z = raw_data.mag_z;
            
            // 模拟VR控制数据
            // 按钮状态：模拟各种VR控制器按钮
            uint16_t buttons = 0;
            
            // 每10帧切换一次按钮状态
            int button_state = (seq[node_id] / 10) % 8;
            
            switch (button_state) {
                case 0:
                    buttons |= (1 << 0); // Trigger
                    break;
                case 1:
                    buttons |= (1 << 1); // Grip
                    break;
                case 2:
                    buttons |= (1 << 2); // Joystick Click
                    break;
                case 3:
                    buttons |= (1 << 3); // A/X
                    break;
                case 4:
                    buttons |= (1 << 4); // B/Y
                    break;
                case 5:
                    buttons |= (1 << 5); // Menu
                    break;
                case 6:
                    // 组合按钮
                    buttons |= (1 << 0) | (1 << 1); // Trigger + Grip
                    break;
                case 7:
                    // 组合按钮
                    buttons |= (1 << 3) | (1 << 4); // A/X + B/Y
                    break;
            }
            
            packet.buttons = buttons;
            
            // 摇杆值：基于时间戳的正弦和余弦值
            float t = static_cast<float>(esp_timer_get_time()) / 1000000.0f + node_id;
            packet.joystick_x = static_cast<float>(sin(t * 0.5f));
            packet.joystick_y = static_cast<float>(cos(t * 0.3f));
            
            // 发送数据包到固定IP和端口
            sender.send_broadcast(packet, TARGET_PORT);
            
            // 对于VR控制器节点（ID=0和1），发送摇杆和按键日志
            if (node_id < 2) {
                // 发送摇杆日志（带防抖）
                check_and_send_joystick_log(node_id, packet.joystick_x, packet.joystick_y);
                
                // 解析按钮状态并发送按键日志
                ButtonData buttons;
                buttons.trigger = (packet.buttons & (1 << 0)) != 0;
                buttons.grip = (packet.buttons & (1 << 1)) != 0;
                buttons.joystick_click = (packet.buttons & (1 << 2)) != 0;
                buttons.a = (packet.buttons & (1 << 3)) != 0;
                buttons.b = (packet.buttons & (1 << 4)) != 0;
                buttons.menu = (packet.buttons & (1 << 5)) != 0;
                buttons.x = false; // 模拟数据中未使用
                buttons.y = false; // 模拟数据中未使用
                
                check_and_send_button_log(node_id, buttons);
            }
            
            // 更新序列号
            seq[node_id] = (seq[node_id] + 1) % 256;
        }
        
        // 保证3.90625ms/包的发送间隔
        vTaskDelay(pdMS_TO_TICKS(3));
    }
}