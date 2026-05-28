#include "mocap_udp_send.hpp"
#include "mpu9250_driver.h"

#include <WiFiUdp.h>
#include <Arduino.h>

// UdpSender 实现细节
class UdpSender::Impl {
private:
    WiFiUDP udp;

public:
    Impl() {}
    
    ~Impl() {}
    
    bool init() {
        return true; // WiFiUDP不需要显式初始化
    }
    
    bool send_to(const MocapPacket& packet, const std::string& target_ip, uint16_t target_port) {
        udp.beginPacket(target_ip.c_str(), target_port);
        udp.write((const uint8_t*)&packet, sizeof(MocapPacket));
        return udp.endPacket() == 1;
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
    // 发送到固定IP和端口
    return impl->send_to(packet, TARGET_IP, port);
}

// MPU9250数据读取函数
MPU9250RawData read_mpu9250_data(uint8_t node_id) {
    MPU9250RawData data = {0};
    
    // 读取MPU9250/MPU6500数据
    MPU9250Data sensor_data = mpu9250_read_data();
    
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
        Serial.println("Failed to initialize UDP sender");
        vTaskDelete(nullptr);
        return;
    }
    
    const int MAX_NODES = 16;
    uint8_t seq[MAX_NODES] = {0};
    
    while (true) {
        for (uint8_t node_id = 0; node_id < 1; node_id++) { // 只发送一个节点
            // 读取传感器数据
            MPU9250RawData raw_data = read_mpu9250_data(node_id);
            
            // 构建数据包
            MocapPacket packet;
            packet.node_id = node_id;
            packet.seq = seq[node_id]++;
            packet.timestamp = micros();
            packet.gyro_x = raw_data.gyro_x;
            packet.gyro_y = raw_data.gyro_y;
            packet.gyro_z = raw_data.gyro_z;
            packet.accel_x = raw_data.accel_x;
            packet.accel_y = raw_data.accel_y;
            packet.accel_z = raw_data.accel_z;
            packet.mag_x = raw_data.mag_x;
            packet.mag_y = raw_data.mag_y;
            packet.mag_z = raw_data.mag_z;
            packet.buttons = 0;
            packet.joystick_x = 0.0f;
            packet.joystick_y = 0.0f;
            
            // 发送数据包
            bool success = sender.send_broadcast(packet, TARGET_PORT);
            if (!success) {
                Serial.println("Failed to send UDP packet");
            }
        }
        
        // 100Hz发送频率
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
