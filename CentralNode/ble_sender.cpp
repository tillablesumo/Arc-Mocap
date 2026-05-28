#include "ble_sender.hpp"

BLEServer* g_pServer = nullptr;
uint16_t g_connectionCount = 0;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        g_connectionCount++;
        Serial.println("BLE client connected");
    }

    void onDisconnect(BLEServer* pServer) {
        if (g_connectionCount > 0) {
            g_connectionCount--;
        }
        Serial.println("BLE client disconnected");
    }
};

BLESender::BLESender() : pServer(nullptr), pService(nullptr), pCharacteristic(nullptr), isRunning(false), connectionCount(0) {}

BLESender::~BLESender() {
    if (pServer) {
        delete pServer;
    }
}

bool BLESender::init(const char* deviceName) {
    BLEDevice::init(deviceName);

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    g_pServer = pServer;

    pService = pServer->createService(BLE_SERVICE_UUID);

    pCharacteristic = pService->createCharacteristic(
        BLE_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY |
        BLECharacteristic::PROPERTY_INDICATE
    );

    pCharacteristic->addDescriptor(new BLE2902());
    pCharacteristic->setNotificationsEnabled(true);

    pService->start();

    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();

    isRunning = true;
    connectionCount = 0;
    g_connectionCount = 0;

    Serial.println("BLE sender initialized");
    Serial.print("Device name: ");
    Serial.println(deviceName);

    return true;
}

void BLESender::send(const MocapPacket& packet) {
    if (!isRunning || !pCharacteristic) return;

    if (g_connectionCount > 0) {
        pCharacteristic->setValue((uint8_t*)&packet, sizeof(MocapPacket));
        pCharacteristic->notify();
    }
}