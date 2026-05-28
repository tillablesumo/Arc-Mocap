#pragma once

#include <cstdint>
#include <string>
#include <memory>
#include <array>
#include <system_error>
#include <span>
#include <format>
#include <expected>
#include "../../../shared/protocol/mocap_protocol.hpp"

// 使用共享协议中的VRNodePacket作为MocapPacket
using MocapPacket = MotionSnap::VRNodePacket;

// SlimeVR数据包结构体
#pragma pack(push, 1)
struct SlimeVRPacket {
    uint8_t node_id;     // 节点编号
    double qx;            // 四元数 x
    double qy;            // 四元数 y
    double qz;            // 四元数 z
    double qw;            // 四元数 w
};
#pragma pack(pop)

static_assert(sizeof(SlimeVRPacket) == 33, "SlimeVRPacket size should be 33 bytes");

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
    
    // 检查是否为VR Node数据包（使用共享协议）
    static bool is_vrnode_packet(const uint8_t* data, size_t length) {
        return MotionSnap::utils::is_vrnode_packet(data, length);
    }
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
