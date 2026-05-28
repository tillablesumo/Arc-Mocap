#pragma once

#include <cstdint>
#include <string>
#include <memory>
#include <array>
#include <system_error>
#include <span>
#include <format>
#include <expected>

// 数据包结构体定义（包含完整原始数据、时间戳和VR控制数据，小端序，1字节对齐）

// 动作捕捉数据包结构体（与ESP32端一致）
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

static_assert(sizeof(MocapPacket) == 54, "MocapPacket size should be 54 bytes"); // 1+1+8+9*4+2+2*4=1+1+8+36+2+8=54

// SlimeVR数据包结构体（支持VR控制器）
#pragma pack(push, 1)
struct SlimeVRPacket {
    uint8_t node_id;     // 节点编号
    double qx;            // 四元数 x
    double qy;            // 四元数 y
    double qz;            // 四元数 z
    double qw;            // 四元数 w
    uint16_t buttons;    // 按钮状态（位掩码）
    double joystick_x;    // 摇杆X轴值 (-1.0 ~ 1.0)
    double joystick_y;    // 摇杆Y轴值 (-1.0 ~ 1.0)
};
#pragma pack(pop)

static_assert(sizeof(SlimeVRPacket) == 51, "SlimeVRPacket size should be 51 bytes"); // 1+4*8+2+2*8=1+32+2+16=51

// SOCKET删除器
struct SocketDeleter {
    void operator()(SOCKET socket) const noexcept {
        if (socket != INVALID_SOCKET) {
            closesocket(socket);
        }
    }
};

// UDP接收类
class CppUdpReceiver {
private:
    using SocketPtr = std::unique_ptr<SOCKET, SocketDeleter>;
    
    SocketPtr raw_socket;      // 接收原始数据的socket
    SocketPtr slimevr_socket;  // 接收SlimeVR数据的socket
    
    std::expected<bool, std::error_code> init_socket(SocketPtr& socket_ptr, uint16_t port, bool enable_broadcast = false);
    
public:
    CppUdpReceiver();
    ~CppUdpReceiver();
    
    // 初始化UDP接收
    std::expected<bool, std::error_code> init(uint16_t raw_port = 8888, uint16_t slimevr_port = 6969);
    
    // 接收原始MPU9250数据
    std::expected<bool, std::error_code> receive_raw_data(MocapPacket& packet, std::string& sender_ip, uint16_t& sender_port);
    
    // 接收SlimeVR格式数据
    std::expected<bool, std::error_code> receive_slimevr_data(SlimeVRPacket& packet, std::string& sender_ip, uint16_t& sender_port);
    
    // 关闭所有socket
    void close() noexcept;
};

// 解析二进制数据的辅助函数
namespace utils {
    // 从字节数组中解析数据
    template <typename T>
    T parse_data(const std::span<const std::byte, sizeof(T)> data) {
        T result;
        std::memcpy(&result, data.data(), sizeof(T));
        return result;
    }
    
    // 将数据转换为字节数组
    template <typename T>
    std::array<std::byte, sizeof(T)> to_bytes(const T& data) {
        std::array<std::byte, sizeof(T)> result;
        std::memcpy(result.data(), &data, sizeof(T));
        return result;
    }
}

// 编译配置：MSVC/MinGW
// MSVC: cl /std:c++23 /EHsc mocap_udp_recv.cpp /link ws2_32.lib
// MinGW: g++ -std=c++23 mocap_udp_recv.cpp -o mocap_recv -lws2_32
