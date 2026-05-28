#pragma once

#include <stdint.h>

#define OTA_UDP_PORT 8082

#define OTA_CMD_START 0x01
#define OTA_CMD_DATA 0x02
#define OTA_CMD_END 0x03
#define OTA_CMD_ACK 0x04
#define OTA_CMD_ERROR 0x05

#define OTA_ERR_NONE 0x00
#define OTA_ERR_INVALID_COMMAND 0x01
#define OTA_ERR_INVALID_SIZE 0x02
#define OTA_ERR_INVALID_CHECKSUM 0x03
#define OTA_ERR_FLASH_WRITE 0x04
#define OTA_ERR_MEMORY 0x05
#define OTA_ERR_TIMEOUT 0x06
#define OTA_ERR_ABORTED 0x07

#pragma pack(push, 1)
struct OTAPacketHeader {
    uint8_t command;
    uint32_t firmwareSize;
    uint16_t packetIndex;
    uint16_t dataSize;
    uint32_t checksum;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct OTAPacket {
    OTAPacketHeader header;
    uint8_t data[1024];
};
#pragma pack(pop)

#pragma pack(push, 1)
struct OTAAckPacket {
    uint8_t command;
    uint16_t packetIndex;
    uint8_t errorCode;
    uint32_t checksum;
};
#pragma pack(pop)

inline uint32_t calculateChecksum(const uint8_t* data, size_t size) {
    uint32_t crc = 0xFFFFFFFF;

    for (size_t i = 0; i < size; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }

    return ~crc;
}

inline bool verifyChecksum(const uint8_t* data, size_t size, uint32_t expectedChecksum) {
    return calculateChecksum(data, size) == expectedChecksum;
}