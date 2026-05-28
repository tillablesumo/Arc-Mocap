#include <Arduino.h>
#include "mocap_udp_send.hpp"
#include "mpu9250_driver.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
 
#define BLE_SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BLE_OTA_CHARACTERISTIC_UUID "a3c87500-8ed3-4bdf-8a39-a01bebede295"

#include "ota_update.h"

BLEServer* pServer = nullptr;
BLEService* pService = nullptr;
BLECharacteristic* pMocapCharacteristic = nullptr;
BLECharacteristic* pOTACharacteristic = nullptr;
uint16_t connectionCount = 0;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        connectionCount++;
        Serial.println("Central connected");
    }
    void onDisconnect(BLEServer* pServer) {
        if (connectionCount > 0) connectionCount--;
        Serial.println("Central disconnected");
    }
};

class MyOTACharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() == sizeof(OTAPacket)) {
            otaUpdate.processOTAPacket((const uint8_t*)value.c_str(), value.length());
        }
    }
};

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("=== MotionSnap VR Node (BLE) ===");
    Serial.print("Chip Model: ");
    Serial.println(ESP.getChipModel());

    if (!mpu9250_init()) {
        Serial.println("Failed to initialize MPU9250");
        return;
    }
    Serial.println("MPU9250 initialized");

    // 初始化 OTA
    otaUpdate.begin();

    BLEDevice::init("MotionSnap-VRNode");
    
    // 设置 BLE 发射功率 (0 dBm)
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P7);
    
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    pService = pServer->createService(BLE_SERVICE_UUID);

    // Mocap 特征
    pMocapCharacteristic = pService->createCharacteristic(
        BLE_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pMocapCharacteristic->addDescriptor(new BLE2902());
    pMocapCharacteristic->setNotificationsEnabled(true);

    // OTA 特征
    pOTACharacteristic = pService->createCharacteristic(
        BLE_OTA_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pOTACharacteristic->addDescriptor(new BLE2902());
    pOTACharacteristic->setCallbacks(new MyOTACharacteristicCallbacks());

    // 发射功率特征
    BLECharacteristic* pPowerCharacteristic = pService->createCharacteristic(
        "a3c87501-8ed3-4bdf-8a39-a01bebede296",
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE
    );
    pPowerCharacteristic->addDescriptor(new BLE2902());
    pPowerCharacteristic->setValue((uint8_t*)&esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_DEFAULT), 1);
    pPowerCharacteristic->setCallbacks(new BLECharacteristicCallbacks() {
        void onWrite(BLECharacteristic* pCharacteristic) {
            std::string value = pCharacteristic->getValue();
            if (value.length() == 1) {
                uint8_t power_level = value[0];
                if (power_level >= 0 && power_level <= 7) {
                    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, (esp_power_level_t)power_level);
                    Serial.printf("Power level set to: %d (0-7)\n", power_level);
                    pCharacteristic->setValue(value);
                }
            }
        }
    });

    // 设置 OTA 特征
    otaUpdate.setOTACharacteristic(pOTACharacteristic);

    pService->start();

    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();

    Serial.println("BLE initialized, waiting for central...");
    Serial.printf("Current BLE transmit power: %d dBm\n", esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_DEFAULT) - 4);
}

void loop() {
    static uint8_t seq = 0;
    static uint32_t lastSend = 0;

    // 处理 OTA
    otaUpdate.loop();

    if (millis() - lastSend >= 10) {
        MPU9250Data data = mpu9250_read_data();

        MocapPacket packet;
        packet.node_id = 1;
        packet.seq = seq++;
        packet.timestamp = micros();
        packet.gyro_x = data.gyro_x;
        packet.gyro_y = data.gyro_y;
        packet.gyro_z = data.gyro_z;
        packet.accel_x = data.accel_x;
        packet.accel_y = data.accel_y;
        packet.accel_z = data.accel_z;
        packet.mag_x = data.mag_x;
        packet.mag_y = data.mag_y;
        packet.mag_z = data.mag_z;
        packet.buttons = 0;
        packet.joystick_x = 0;
        packet.joystick_y = 0;

        if (connectionCount > 0 && pMocapCharacteristic) {
            pMocapCharacteristic->setValue((uint8_t*)&packet, sizeof(MocapPacket));
            pMocapCharacteristic->notify();
        }

        lastSend = millis();
    }

    delay(10);
}