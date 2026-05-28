#include "ota_update.h"

// 全局 OTA 更新实例
OTAUpdate otaUpdate;

OTAUpdate::OTAUpdate() : 
    _updating(false),
    _updateStarted(false),
    _firmwareSize(0),
    _currentPacketIndex(0),
    _receivedSize(0),
    _otaHandle(0),
    _updatePartition(nullptr),
    _otaCharacteristic(nullptr) {
}

OTAUpdate::~OTAUpdate() {
    abortUpdate();
}

void OTAUpdate::begin() {
    Serial.println("OTA update initialized");
}

// 设置 OTA 特征
void OTAUpdate::setOTACharacteristic(BLECharacteristic* characteristic) {
    _otaCharacteristic = characteristic;
    Serial.println("OTA characteristic set");
}

// 处理 OTA 数据包
void OTAUpdate::processOTAPacket(const uint8_t* data, size_t size) {
    if (size < sizeof(OTAPacketHeader)) {
        sendAck(0, OTA_ERR_INVALID_SIZE);
        return;
    }
    
    OTAPacketHeader* header = (OTAPacketHeader*)data;
    
    // 验证校验和
    if (!verifyChecksum(data + sizeof(OTAPacketHeader), header->dataSize, header->checksum)) {
        sendAck(header->packetIndex, OTA_ERR_INVALID_CHECKSUM);
        return;
    }
    
    switch (header->command) {
        case OTA_CMD_START:
            // 开始升级
            if (_updating) {
                sendAck(header->packetIndex, OTA_ERR_ABORTED);
                return;
            }
            
            if (startUpdate(header->firmwareSize)) {
                _updating = true;
                _updateStarted = true;
                _firmwareSize = header->firmwareSize;
                _currentPacketIndex = 0;
                _receivedSize = 0;
                sendAck(header->packetIndex, OTA_ERR_NONE);
                Serial.println("OTA update started, firmware size: " + String(header->firmwareSize) + " bytes");
            } else {
                sendAck(header->packetIndex, OTA_ERR_MEMORY);
            }
            break;
            
        case OTA_CMD_DATA:
            // 数据传输
            if (!_updating) {
                sendAck(header->packetIndex, OTA_ERR_INVALID_COMMAND);
                return;
            }
            
            if (header->packetIndex != _currentPacketIndex) {
                sendAck(header->packetIndex, OTA_ERR_INVALID_COMMAND);
                return;
            }
            
            if (writeData(data + sizeof(OTAPacketHeader), header->dataSize)) {
                _currentPacketIndex++;
                _receivedSize += header->dataSize;
                sendAck(header->packetIndex, OTA_ERR_NONE);
                
                // 打印进度
                float progress = (_receivedSize * 100.0f) / _firmwareSize;
                Serial.printf("OTA progress: %.1f%%\n", progress);
            } else {
                sendAck(header->packetIndex, OTA_ERR_FLASH_WRITE);
                abortUpdate();
            }
            break;
            
        case OTA_CMD_END:
            // 结束升级
            if (!_updating) {
                sendAck(header->packetIndex, OTA_ERR_INVALID_COMMAND);
                return;
            }
            
            if (finishUpdate()) {
                sendAck(header->packetIndex, OTA_ERR_NONE);
                Serial.println("OTA update completed successfully, rebooting...");
                ESP.restart();
            } else {
                sendAck(header->packetIndex, OTA_ERR_FLASH_WRITE);
                abortUpdate();
            }
            break;
            
        default:
            sendAck(header->packetIndex, OTA_ERR_INVALID_COMMAND);
            break;
    }
}

void OTAUpdate::sendAck(uint16_t packetIndex, uint8_t errorCode) {
    OTAAckPacket ack;
    ack.command = OTA_CMD_ACK;
    ack.node_id = 1; // 设备自身的 node_id
    ack.packetIndex = packetIndex;
    ack.errorCode = errorCode;
    ack.checksum = calculateChecksum((uint8_t*)&ack, sizeof(ack) - sizeof(ack.checksum));
    
    sendPacket(&ack, sizeof(ack));
}

bool OTAUpdate::isUpdating() const {
    return _updating;
}

float OTAUpdate::getProgress() const {
    if (!_updating || _firmwareSize == 0) {
        return 0.0f;
    }
    return (_receivedSize * 100.0f) / _firmwareSize;
}

void OTAUpdate::abortUpdate() {
    if (_updating) {
        if (_updateStarted) {
            esp_ota_abort(_otaHandle);
        }
        _updating = false;
        _updateStarted = false;
        _firmwareSize = 0;
        _currentPacketIndex = 0;
        _receivedSize = 0;
        _otaHandle = 0;
        _updatePartition = nullptr;
        Serial.println("OTA update aborted");
    }
}

bool OTAUpdate::startUpdate(uint32_t firmwareSize) {
    // 获取更新分区
    _updatePartition = esp_ota_get_next_update_partition(nullptr);
    if (!_updatePartition) {
        Serial.println("Failed to get update partition");
        return false;
    }
    
    // 开始 OTA 更新
    esp_err_t err = esp_ota_begin(_updatePartition, OTA_SIZE_UNKNOWN, &_otaHandle);
    if (err != ESP_OK) {
        Serial.printf("Failed to begin OTA update: %s\n", esp_err_to_name(err));
        return false;
    }
    
    return true;
}

bool OTAUpdate::writeData(const uint8_t* data, size_t size) {
    esp_err_t err = esp_ota_write(_otaHandle, data, size);
    if (err != ESP_OK) {
        Serial.printf("Failed to write OTA data: %s\n", esp_err_to_name(err));
        return false;
    }
    return true;
}

bool OTAUpdate::finishUpdate() {
    esp_err_t err = esp_ota_end(_otaHandle);
    if (err != ESP_OK) {
        Serial.printf("Failed to end OTA update: %s\n", esp_err_to_name(err));
        return false;
    }
    
    // 设置启动分区
    err = esp_ota_set_boot_partition(_updatePartition);
    if (err != ESP_OK) {
        Serial.printf("Failed to set boot partition: %s\n", esp_err_to_name(err));
        return false;
    }
    
    _updating = false;
    _updateStarted = false;
    return true;
}

void OTAUpdate::sendPacket(const void* data, size_t size) {
    if (_otaCharacteristic) {
        _otaCharacteristic->setValue((const uint8_t*)data, size);
        _otaCharacteristic->notify();
    }
}