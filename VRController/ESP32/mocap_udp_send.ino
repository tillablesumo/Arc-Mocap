#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define BLE_SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

const int SDA_PIN = 1;
const int SCL_PIN = 2;

const uint8_t ICM20948_ADDR = 0x68;
const uint8_t ICM20948_WHO_AM_I = 0x00;
const uint8_t ICM20948_USER_CTRL = 0x03;
const uint8_t ICM20948_PWR_MGMT_1 = 0x06;
const uint8_t ICM20948_PWR_MGMT_2 = 0x07;
const uint8_t ICM20948_ACCEL_XOUT_H = 0x2D;
const uint8_t ICM20948_GYRO_XOUT_H = 0x33;

const uint8_t AK09916_ADDR = 0x0C;
const uint8_t AK09916_WHO_AM_I = 0x01;
const uint8_t AK09916_ST1 = 0x10;
const uint8_t AK09916_HXL = 0x11;
const uint8_t AK09916_CNTL2 = 0x31;

#pragma pack(push, 1)
struct MocapPacket {
    uint8_t node_id;
    uint8_t seq;
    uint64_t timestamp;
    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
    float mag_x, mag_y, mag_z;
    uint16_t buttons;
    float joystick_x, joystick_y;
};
#pragma pack(pop)

struct ICM20948RawData {
    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
    int16_t mag_x, mag_y, mag_z;
};

BLEServer* pServer = nullptr;
BLEService* pService = nullptr;
BLECharacteristic* pCharacteristic = nullptr;
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

uint8_t seq = 0;
uint8_t node_id = 1;

float gyro_scale = 1.0 / 131.0 * M_PI / 180.0;
float accel_scale = 1.0 / 16384.0 * 9.81;

bool initICM20948() {
    Wire.beginTransmission(ICM20948_ADDR);
    Wire.write(ICM20948_WHO_AM_I);
    Wire.endTransmission(false);
    Wire.requestFrom(ICM20948_ADDR, 1);
    if (Wire.available()) {
        uint8_t who_am_i = Wire.read();
        if (who_am_i != 0xEA) {
            Serial.print("ICM20948 not found! WHO_AM_I = 0x");
            Serial.println(who_am_i, HEX);
            return false;
        }
    } else {
        Serial.println("Failed to read ICM20948 WHO_AM_I register!");
        return false;
    }

    Wire.beginTransmission(ICM20948_ADDR);
    Wire.write(ICM20948_PWR_MGMT_1);
    Wire.write(0x80);
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(ICM20948_ADDR);
    Wire.write(ICM20948_PWR_MGMT_1);
    Wire.write(0x01);
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(ICM20948_ADDR);
    Wire.write(ICM20948_PWR_MGMT_2);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(ICM20948_ADDR);
    Wire.write(ICM20948_USER_CTRL);
    Wire.write(0x20);
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(AK09916_ADDR);
    Wire.write(AK09916_WHO_AM_I);
    Wire.endTransmission(false);
    Wire.requestFrom(AK09916_ADDR, 1);
    if (Wire.available()) {
        uint8_t who_am_i = Wire.read();
        if (who_am_i != 0x09) {
            Serial.print("AK09916 not found! WHO_AM_I = 0x");
            Serial.println(who_am_i, HEX);
            return false;
        }
    } else {
        Serial.println("Failed to read AK09916 WHO_AM_I register!");
        return false;
    }

    Wire.beginTransmission(AK09916_ADDR);
    Wire.write(AK09916_CNTL2);
    Wire.write(0x08);
    Wire.endTransmission();
    delay(100);

    Serial.println("ICM20948 initialized successfully!");
    return true;
}

ICM20948RawData readICM20948Data() {
    ICM20948RawData data;

    Wire.beginTransmission(ICM20948_ADDR);
    Wire.write(ICM20948_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(ICM20948_ADDR, 12);

    if (Wire.available() == 12) {
        int16_t accel_x = Wire.read() << 8 | Wire.read();
        int16_t accel_y = Wire.read() << 8 | Wire.read();
        int16_t accel_z = Wire.read() << 8 | Wire.read();
        int16_t gyro_x = Wire.read() << 8 | Wire.read();
        int16_t gyro_y = Wire.read() << 8 | Wire.read();
        int16_t gyro_z = Wire.read() << 8 | Wire.read();

        data.accel_x = accel_x * accel_scale;
        data.accel_y = accel_y * accel_scale;
        data.accel_z = accel_z * accel_scale;
        data.gyro_x = gyro_x * gyro_scale;
        data.gyro_y = gyro_y * gyro_scale;
        data.gyro_z = gyro_z * gyro_scale;
    }

    Wire.beginTransmission(AK09916_ADDR);
    Wire.write(AK09916_ST1);
    Wire.endTransmission(false);
    Wire.requestFrom(AK09916_ADDR, 1);
    if (Wire.available()) {
        uint8_t st1 = Wire.read();
        if (st1 & 0x01) {
            Wire.beginTransmission(AK09916_ADDR);
            Wire.write(AK09916_HXL);
            Wire.endTransmission(false);
            Wire.requestFrom(AK09916_ADDR, 6);

            if (Wire.available() == 6) {
                data.mag_x = Wire.read() | (Wire.read() << 8);
                data.mag_y = Wire.read() | (Wire.read() << 8);
                data.mag_z = Wire.read() | (Wire.read() << 8);
            }

            Wire.beginTransmission(AK09916_ADDR);
            Wire.write(0x18);
            Wire.endTransmission(false);
            Wire.requestFrom(AK09916_ADDR, 1);
            if (Wire.available()) {
                Wire.read();
            }
        }
    }

    return data;
}

void readVRControlData(uint16_t& buttons, float& joystick_x, float& joystick_y) {
    int button_state = (seq / 10) % 8;

    buttons = 0;
    switch (button_state) {
        case 0: buttons |= (1 << 0); break;
        case 1: buttons |= (1 << 1); break;
        case 2: buttons |= (1 << 2); break;
        case 3: buttons |= (1 << 3); break;
        case 4: buttons |= (1 << 4); break;
        case 5: buttons |= (1 << 5); break;
        case 6: buttons |= (1 << 0) | (1 << 1); break;
        case 7: buttons |= (1 << 3) | (1 << 4); break;
    }

    float t = static_cast<float>(micros()) / 1000000.0f + node_id;
    joystick_x = static_cast<float>(sin(t * 0.5f));
    joystick_y = static_cast<float>(cos(t * 0.3f));
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("ESP32-S3 ICM20948 VR Controller (BLE)");

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    Serial.println("I2C initialized");

    if (!initICM20948()) {
        Serial.println("Failed to initialize ICM20948, halting...");
        while (1) { delay(1000); }
    }

    BLEDevice::init("MotionSnap-Controller");
    
    // 设置 BLE 发射功率 (0 dBm)
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P7);
    
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    pService = pServer->createService(BLE_SERVICE_UUID);

    pCharacteristic = pService->createCharacteristic(
        BLE_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY
    );

    pCharacteristic->addDescriptor(new BLE2902());
    pCharacteristic->setNotificationsEnabled(true);

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

    pService->start();

    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();

    Serial.println("BLE initialized, waiting for central...");
    Serial.println("GPIO1(SDA) <-> SDA, GPIO2(SCL) <-> SCL, 3.3V, GND");
    Serial.printf("Current BLE transmit power: %d dBm\n", esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_DEFAULT) - 4);
}

void loop() {
    ICM20948RawData raw_data = readICM20948Data();

    uint16_t buttons;
    float joystick_x, joystick_y;
    readVRControlData(buttons, joystick_x, joystick_y);

    MocapPacket packet;
    packet.node_id = node_id;
    packet.seq = seq;
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
    packet.buttons = buttons;
    packet.joystick_x = joystick_x;
    packet.joystick_y = joystick_y;

    if (connectionCount > 0 && pCharacteristic) {
        pCharacteristic->setValue((uint8_t*)&packet, sizeof(MocapPacket));
        pCharacteristic->notify();
    }

    if (seq % 100 == 0) {
        Serial.printf("Controller seq:%d gyro:[%.3f,%.3f,%.3f] accel:[%.3f,%.3f,%.3f]\n",
                     seq, raw_data.gyro_x, raw_data.gyro_y, raw_data.gyro_z,
                     raw_data.accel_x, raw_data.accel_y, raw_data.accel_z);
    }

    seq = (seq + 1) % 256;
    delayMicroseconds(3906);
}