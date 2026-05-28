#include "mocap_udp_recv.hpp"
#include "ekf_mag_calib.hpp"

#include <winsock2.h>
#include <ws2tcpip.h>
#include <iostream>
#include <iomanip>
#include <map>
#include <ranges>

#pragma comment(lib, "ws2_32.lib")

// CppUdpReceiver 构造函数
CppUdpReceiver::CppUdpReceiver() : raw_socket(nullptr), slimevr_socket(nullptr) {
    // 初始化Winsock
    WSADATA wsaData;
    int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (result != 0) {
        std::cerr << std::format("WSAStartup failed: {}", result) << std::endl;
    }
}

// CppUdpReceiver 析构函数
CppUdpReceiver::~CppUdpReceiver() {
    close();
    WSACleanup();
}

// 初始化socket
std::expected<bool, std::error_code> CppUdpReceiver::init_socket(SocketPtr& socket_ptr, uint16_t port, bool enable_broadcast) {
    // 创建socket
    SOCKET s = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s == INVALID_SOCKET) {
        int error = WSAGetLastError();
        return std::unexpected(std::error_code(error, std::system_category()));
    }
    
    // 启用广播
    if (enable_broadcast) {
        BOOL bBroadcast = TRUE;
        if (setsockopt(s, SOL_SOCKET, SO_BROADCAST, (char*)&bBroadcast, sizeof(bBroadcast)) == SOCKET_ERROR) {
            int error = WSAGetLastError();
            closesocket(s);
            return std::unexpected(std::error_code(error, std::system_category()));
        }
    }
    
    // 绑定端口
    sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);
    
    if (bind(s, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
        int error = WSAGetLastError();
        closesocket(s);
        return std::unexpected(std::error_code(error, std::system_category()));
    }
    
    socket_ptr.reset(s);
    return true;
}

// 初始化UDP接收
std::expected<bool, std::error_code> CppUdpReceiver::init(uint16_t raw_port, uint16_t slimevr_port) {
    // 初始化原始数据socket（启用广播）
    auto raw_result = init_socket(raw_socket, raw_port, true);
    if (!raw_result) {
        return raw_result;
    }
    
    // 初始化SlimeVR数据socket
    auto slimevr_result = init_socket(slimevr_socket, slimevr_port);
    if (!slimevr_result) {
        return slimevr_result;
    }
    
    return true;
}

// 接收原始MPU9250数据
std::expected<bool, std::error_code> CppUdpReceiver::receive_raw_data(MocapPacket& packet, std::string& sender_ip, uint16_t& sender_port) {
    if (!raw_socket) {
        return false;
    }
    
    char buffer[1024];
    sockaddr_in sender_addr;
    int sender_addr_len = sizeof(sender_addr);
    
    int bytes_received = recvfrom(*raw_socket, buffer, sizeof(buffer), 0, 
                                (sockaddr*)&sender_addr, &sender_addr_len);
    
    if (bytes_received == SOCKET_ERROR) {
        int error = WSAGetLastError();
        return std::unexpected(std::error_code(error, std::system_category()));
    }
    
    // 检查数据包长度
    if (bytes_received != sizeof(MocapPacket)) {
        std::cerr << std::format("Invalid packet size: {} expected: {}", bytes_received, sizeof(MocapPacket)) << std::endl;
        return false;
    }
    
    // 解析数据包
    std::memcpy(&packet, buffer, sizeof(MocapPacket));
    
    // 获取发送者IP和端口
    char ip_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &(sender_addr.sin_addr), ip_str, INET_ADDRSTRLEN);
    sender_ip = ip_str;
    sender_port = ntohs(sender_addr.sin_port);
    
    return true;
}

// 接收SlimeVR格式数据
std::expected<bool, std::error_code> CppUdpReceiver::receive_slimevr_data(SlimeVRPacket& packet, std::string& sender_ip, uint16_t& sender_port) {
    if (!slimevr_socket) {
        return false;
    }
    
    char buffer[1024];
    sockaddr_in sender_addr;
    int sender_addr_len = sizeof(sender_addr);
    
    int bytes_received = recvfrom(*slimevr_socket, buffer, sizeof(buffer), 0, 
                                (sockaddr*)&sender_addr, &sender_addr_len);
    
    if (bytes_received == SOCKET_ERROR) {
        int error = WSAGetLastError();
        return std::unexpected(std::error_code(error, std::system_category()));
    }
    
    // 检查数据包长度
    if (bytes_received != sizeof(SlimeVRPacket)) {
        std::cerr << std::format("Invalid SlimeVR packet size: {} expected: {}", bytes_received, sizeof(SlimeVRPacket)) << std::endl;
        return false;
    }
    
    // 解析数据包
    std::memcpy(&packet, buffer, sizeof(SlimeVRPacket));
    
    // 获取发送者IP和端口
    char ip_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &(sender_addr.sin_addr), ip_str, INET_ADDRSTRLEN);
    sender_ip = ip_str;
    sender_port = ntohs(sender_addr.sin_port);
    
    return true;
}

// 关闭所有socket
void CppUdpReceiver::close() noexcept {
    raw_socket.reset();
    slimevr_socket.reset();
}

// 主函数
int main() {
    CppUdpReceiver receiver;
    
    // 初始化接收
    auto init_result = receiver.init(8888, 6969);
    if (!init_result) {
        std::cerr << std::format("Failed to initialize receiver: {}", init_result.error().message()) << std::endl;
        return 1;
    }
    
    // 为每个节点创建EKF实例
    std::map<uint8_t, EKFMagCalib> ekf_map;
    
    // 配置EKF参数
    for (uint8_t i : std::views::iota(0, 16)) {
        ekf_map[i].set_process_noise(0.001, 0.0001, 0.0001); // q_gyro, q_mag, q_gyro_bias
        ekf_map[i].set_observation_noise(0.1, 0.1); // r_accel, r_mag
    }
    
    // 跟踪每个节点的上一个序列号和连续丢失计数
    std::map<uint8_t, uint8_t> last_seq_map;
    std::map<uint8_t, int> lost_count_map;
    std::map<uint8_t, bool> ekf_paused_map;
    
    // 初始化跟踪数据
    for (uint8_t i : std::views::iota(0, 16)) {
        last_seq_map[i] = 0;
        lost_count_map[i] = 0;
        ekf_paused_map[i] = false;
    }
    
    std::cout << "Receiver initialized. Waiting for data..." << std::endl;
    
    while (true) {
        // 接收原始数据
        MocapPacket raw_packet;
        std::string raw_sender_ip;
        uint16_t raw_sender_port;
        
        auto receive_result = receiver.receive_raw_data(raw_packet, raw_sender_ip, raw_sender_port);
        if (receive_result) {
            if (*receive_result) {
                // 提取节点ID
                uint8_t node_id = raw_packet.node_id;
                
                // 检查数据包是否丢失
                uint8_t current_seq = raw_packet.seq;
                uint8_t last_seq = last_seq_map[node_id];
                
                // 计算序列号差异（考虑循环）
                int seq_diff = (current_seq - last_seq + 256) % 256;
                
                if (seq_diff == 1) {
                    // 序列号连续，重置丢失计数
                    lost_count_map[node_id] = 0;
                    ekf_paused_map[node_id] = false;
                } else if (seq_diff > 1) {
                    // 有数据包丢失
                    lost_count_map[node_id] += seq_diff - 1;
                    std::cout << std::format("Node {} lost {} packets", static_cast<int>(node_id), (seq_diff - 1)) << std::endl;
                    
                    // 检查是否连续丢失2个或更多数据包
                    if (lost_count_map[node_id] >= 2) {
                        ekf_paused_map[node_id] = true;
                        std::cout << std::format("Node {} EKF paused due to packet loss", static_cast<int>(node_id)) << std::endl;
                    }
                }
                
                // 更新最后序列号
                last_seq_map[node_id] = current_seq;
                
                // 从数据包中提取MPU9250原始数据
                double gyro_x = raw_packet.gyro_x;
                double gyro_y = raw_packet.gyro_y;
                double gyro_z = raw_packet.gyro_z;
                double accel_x = raw_packet.accel_x;
                double accel_y = raw_packet.accel_y;
                double accel_z = raw_packet.accel_z;
                double mag_x = raw_packet.mag_x;
                double mag_y = raw_packet.mag_y;
                double mag_z = raw_packet.mag_z;
                
                // 只有当EKF未暂停时才运行EKF算法
                if (!ekf_paused_map[node_id]) {
                    // 运行EKF算法
                    Quaternion corrected_q = ekf_map[node_id].update(
                        gyro_x, gyro_y, gyro_z,
                        accel_x, accel_y, accel_z,
                        mag_x, mag_y, mag_z,
                        1.0 / 256.0 // 采样时间 3.90625ms
                    );
                    
                    std::cout << "=== Raw Data ===" << std::endl;
                    std::cout << std::format("Sender: {}:{}", raw_sender_ip, raw_sender_port) << std::endl;
                    std::cout << std::format("Node ID: {}", static_cast<int>(node_id)) << std::endl;
                    std::cout << std::format("Sequence: {}", static_cast<int>(raw_packet.seq)) << std::endl;
                    std::cout << std::format("Timestamp: {} us", raw_packet.timestamp) << std::endl;
                    std::cout << std::format("Raw Magnetometer: [{}, {}, {}]", 
                                         raw_packet.mag_x, 
                                         raw_packet.mag_y, 
                                         raw_packet.mag_z) << std::endl;
                    std::cout << std::endl;
                    
                    std::cout << "=== EKF Processed Data ===" << std::endl;
                    std::cout << std::format("Node ID: {}", static_cast<int>(node_id)) << std::endl;
                    std::cout << std::format("Corrected Quaternion: [{}, {}, {}, {}]", 
                                         std::fixed << std::setprecision(3) << corrected_q.x, 
                                         corrected_q.y, 
                                         corrected_q.z, 
                                         corrected_q.w) << std::endl;
                    
                    // 获取磁力计偏置
                    auto mag_bias = ekf_map[node_id].get_mag_bias();
                    std::cout << std::format("Magnetometer Bias: [{}, {}, {}]", 
                                         std::fixed << std::setprecision(3) << mag_bias[0], 
                                         mag_bias[1], 
                                         mag_bias[2]) << std::endl;
                    
                    // 获取陀螺仪偏置
                    auto gyro_bias = ekf_map[node_id].get_gyro_bias();
                    std::cout << std::format("Gyroscope Bias: [{}, {}, {}]", 
                                         std::fixed << std::setprecision(3) << gyro_bias[0], 
                                         gyro_bias[1], 
                                         gyro_bias[2]) << std::endl;
                    std::cout << std::endl;
                } else {
                    std::cout << "=== Raw Data ===" << std::endl;
                    std::cout << std::format("Sender: {}:{}", raw_sender_ip, raw_sender_port) << std::endl;
                    std::cout << std::format("Node ID: {}", static_cast<int>(node_id)) << std::endl;
                    std::cout << std::format("Sequence: {}", static_cast<int>(raw_packet.seq)) << std::endl;
                    std::cout << std::format("Timestamp: {} us", raw_packet.timestamp) << std::endl;
                    std::cout << std::format("Raw Magnetometer: [{}, {}, {}]", 
                                         raw_packet.mag_x, 
                                         raw_packet.mag_y, 
                                         raw_packet.mag_z) << std::endl;
                    std::cout << "EKF is paused due to packet loss" << std::endl;
                    std::cout << std::endl;
                }
            }
        } else {
            std::cerr << std::format("Receive error: {}", receive_result.error().message()) << std::endl;
        }
        
        // 接收SlimeVR数据（可选，用于验证）
        SlimeVRPacket slimevr_packet;
        std::string slimevr_sender_ip;
        uint16_t slimevr_sender_port;
        
        auto slimevr_result = receiver.receive_slimevr_data(slimevr_packet, slimevr_sender_ip, slimevr_sender_port);
        if (slimevr_result && *slimevr_result) {
            std::cout << "=== SlimeVR Data ===" << std::endl;
            std::cout << std::format("Sender: {}:{}", slimevr_sender_ip, slimevr_sender_port) << std::endl;
            std::cout << std::format("Node ID: {}", static_cast<int>(slimevr_packet.node_id)) << std::endl;
            std::cout << std::format("Quaternion: [{}, {}, {}, {}]", 
                                 std::fixed << std::setprecision(3) << slimevr_packet.qx, 
                                 slimevr_packet.qy, 
                                 slimevr_packet.qz, 
                                 slimevr_packet.qw) << std::endl;
            std::cout << std::endl;
        }
    }
    
    return 0;
}
