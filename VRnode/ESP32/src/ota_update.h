#pragma once

#include "ota_protocol.h"
#include <Arduino.h>
#include <esp_ota_ops.h>
#include <BLECharacteristic.h>

class OTAUpdate {
private:
    bool _updating;
    bool _updateStarted;
    uint32_t _firmwareSize;
    uint16_t _currentPacketIndex;
    uint32_t _receivedSize;
    esp_ota_handle_t _otaHandle;
    const esp_partition_t* _updatePartition;
    BLECharacteristic* _otaCharacteristic;

public:
    OTAUpdate();
    ~OTAUpdate();

    void begin();
    void setOTACharacteristic(BLECharacteristic* characteristic);
    void processOTAPacket(const uint8_t* data, size_t size);
    void sendAck(uint16_t packetIndex, uint8_t errorCode);
    bool isUpdating() const;
    float getProgress() const;
    void abortUpdate();

private:
    bool startUpdate(uint32_t firmwareSize);
    bool writeData(const uint8_t* data, size_t size);
    bool finishUpdate();
    void sendPacket(const void* data, size_t size);
};

extern OTAUpdate otaUpdate;