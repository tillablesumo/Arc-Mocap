#pragma once

#include <cstdint>
#include <string>
#include <memory>

// 数据包结构体定义（包含完整原始数据、时间戳和VR控制数据，小端序，1字节对齐）
#pragma pack(push, 1)
struct MocapPacket {
    uint8_t node_id;     // 节点编号 0~15
    uint8_t seq;         // 帧序号 0~255循环
    uint64_t timestamp;  // 高精度时间戳（微秒）
    float gyro_x;        // 陀螺仪 x 原始值 (rad/s)
    float gyro_y;        // 陀螺仪 y 原始值 (rad/s)
    float gyro_z;        // 陀螺仪 z 原始值 (rad/s)
    float accel_x;       // 加速度计 x 原始值 (m/s²)
    float accel_y;       // 加速度计 y 原始值 (m/s²)
    float accel_z;       // 加速度计 z 原始值 (m/s²)
    float mag_x;         // 磁力计 x 原始值
    float mag_y;         // 磁力计 y 原始值
    float mag_z;         // 磁力计 z 原始值
    uint16_t buttons;    // 按钮状态（位掩码）
    float joystick_x;    // 摇杆X轴值 (-1.0 ~ 1.0)
    float joystick_y;    // 摇杆Y轴值 (-1.0 ~ 1.0)
};
#pragma pack(pop)

// static_assert(sizeof(MocapPacket) == 54, "MocapPacket size should be 54 bytes"); // 1+1+8+9*4+2+2*4=1+1+8+36+2+8=54

// 四元数结构体
struct Quaternion {
    float x, y, z, w;
    Quaternion() : x(0), y(0), z(0), w(1) {}
    Quaternion(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
};

// MPU9250原始数据结构体
struct MPU9250RawData {
    float gyro_x, gyro_y, gyro_z;     // 陀螺仪数据 (rad/s)
    float accel_x, accel_y, accel_z;  // 加速度计数据 (m/s²)
    float mag_x, mag_y, mag_z;        // 磁力计原始数据
};

// UDP发送类
class UdpSender {
private:
    class Impl; // 实现细节
    std::unique_ptr<Impl> impl;

public:
    UdpSender();
    ~UdpSender();
    
    // 初始化UDP
    bool init();
    
    // 发送UDP数据包到指定IP:端口
    bool send_broadcast(const MocapPacket& packet, uint16_t port = 8888);
};

// MPU9250数据读取函数
MPU9250RawData read_mpu9250_data(uint8_t node_id);

// 主发送任务
void mocap_send_task(void* pvParameters);

// 目标IP和端口常量
#define TARGET_IP "192.168.1.101"  // 可配置目标IP
#define TARGET_PORT 8888            // 可配置目标端口
