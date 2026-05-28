#include "mocap_data.hpp"
#include "ekf_mag_calib.hpp"

#include <iostream>
#include <iomanip>
#include <map>
#include <ranges>
#include <windows.h>

class CppSerialReceiver {
private:
    HANDLE hSerial;
    bool is_open;

public:
    CppSerialReceiver() : hSerial(INVALID_HANDLE_VALUE), is_open(false) {}
    ~CppSerialReceiver() { close(); }

    bool init(const char* port_name, uint32_t baud_rate = 115200) {
        DCB dcbSerialParams = {0};
        COMMTIMEOUTS timeouts = {0};

        hSerial = CreateFileA(port_name, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
        if (hSerial == INVALID_HANDLE_VALUE) {
            std::cerr << "Failed to open serial port: " << port_name << std::endl;
            return false;
        }

        dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
        if (!GetCommState(hSerial, &dcbSerialParams)) {
            std::cerr << "Failed to get comm state" << std::endl;
            return false;
        }

        dcbSerialParams.BaudRate = baud_rate;
        dcbSerialParams.ByteSize = 8;
        dcbSerialParams.StopBits = ONESTOPBIT;
        dcbSerialParams.Parity = NOPARITY;

        if (!SetCommState(hSerial, &dcbSerialParams)) {
            std::cerr << "Failed to set comm state" << std::endl;
            return false;
        }

        timeouts.ReadIntervalTimeout = 50;
        timeouts.ReadTotalTimeoutConstant = 50;
        timeouts.ReadTotalTimeoutMultiplier = 10;

        if (!SetCommTimeouts(hSerial, &timeouts)) {
            std::cerr << "Failed to set timeouts" << std::endl;
            return false;
        }

        is_open = true;
        std::cout << "Serial port opened: " << port_name << std::endl;
        return true;
    }

    bool receive_raw_data(MocapPacket& packet) {
        if (!is_open) return false;

        uint8_t buffer[sizeof(MocapPacket)];
        DWORD bytes_read = 0;

        if (!ReadFile(hSerial, buffer, sizeof(MocapPacket), &bytes_read, NULL)) {
            return false;
        }

        if (bytes_read != sizeof(MocapPacket)) {
            return false;
        }

        memcpy(&packet, buffer, sizeof(MocapPacket));
        return true;
    }

    void close() {
        if (hSerial != INVALID_HANDLE_VALUE) {
            CloseHandle(hSerial);
            hSerial = INVALID_HANDLE_VALUE;
        }
        is_open = false;
    }

    bool isConnected() const { return is_open; }
};

int main(int argc, char* argv[]) {
    const char* port_name = (argc > 1) ? argv[1] : "\\\\.\\COM7";

    CppSerialReceiver receiver;
    if (!receiver.init(port_name, 115200)) {
        std::cerr << "Failed to initialize serial port" << std::endl;
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
    
    std::cout << "Serial receiver initialized. Waiting for data..." << std::endl;
    
    while (true) {
        // 接收串口数据
        MocapPacket raw_packet;
        
        if (receiver.receive_raw_data(raw_packet)) {
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
    }
    
    return 0;
}