#pragma once

#include <cstdint>
#include <string>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// BLE 服务和特征 UUID
#define BLE_SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// MocapPacket 结构 (54 bytes) - 与现有协议保持一致
#pragma pack(push, 1)
struct MocapPacket {
    uint8_t node_id;        // 节点编号 0~15
    uint8_t seq;            // 帧序号
    uint64_t timestamp;     // 时间戳（微秒）
    float gyro_x, gyro_y, gyro_z;      // 陀螺仪 (rad/s)
    float accel_x, accel_y, accel_z;   // 加速度计 (m/s²)
    float mag_x, mag_y, mag_z;         // 磁力计
    uint16_t buttons;    // 按钮状态
    float joystick_x;    // 摇杆X
    float joystick_y;    // 摇杆Y
};
#pragma pack(pop)

static_assert(sizeof(MocapPacket) == 54, "MocapPacket size should be 54 bytes");

// BLE 发送服务器类
class BLESender {
private:
    BLEServer* pServer;
    BLEService* pService;
    BLECharacteristic* pCharacteristic;
    bool isRunning;
    uint16_t connectionCount;

public:
    BLESender();
    ~BLESender();

    bool init(const char* deviceName);
    void send(const MocapPacket& packet);
    bool isConnected() const { return connectionCount > 0; }
    uint16_t getConnectionCount() const { return connectionCount; }

private:
    static void onConnect(BLEServerCallbacks* pCallback);
    static void onDisconnect(BLEServerCallbacks* pCallback);
};

// 全局服务器指针（用于回调）
extern BLEServer* g_pServer;
extern uint16_t g_connectionCount;