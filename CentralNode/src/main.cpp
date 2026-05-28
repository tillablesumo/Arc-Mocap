#include <Arduino.h>
#include <map>
#include "BLEDevice.h"
#include <esp_system.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

// OTA 协议
#include "../../ota_protocol.h"

// BLE 服务和特征 UUID（与从节点一致）
#define BLE_SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BLE_OTA_CHARACTERISTIC_UUID "a3c87500-8ed3-4bdf-8a39-a01bebede295"

// 数据包结构（复用现有格式）
#pragma pack(push, 1)
struct MocapPacket {
    uint8_t node_id;
    uint8_t seq;
    uint64_t timestamp;
    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
    float mag_x, mag_y, mag_z;
    uint16_t buttons;
    float joystick_x;
    float joystick_y;
};
#pragma pack(pop)

// 雷达数据结构
#pragma pack(push, 1)
struct RadarData {
    uint8_t node_id;
    uint8_t target_count;
    float distance_1;
    float distance_2;
    float distance_3;
    uint16_t reserved;
};
#pragma pack(pop)

// 扩展数据包（包含IMU+雷达）
#pragma pack(push, 1)
struct ExtendedMocapPacket {
    uint8_t packet_type;      // 0=纯IMU, 1=IMU+雷达
    uint8_t node_id;
    uint8_t seq;
    uint64_t timestamp;
    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
    float mag_x, mag_y, mag_z;
    uint16_t buttons;
    float joystick_x;
    float joystick_y;
    // 雷达数据
    uint8_t radar_target_count;
    float radar_distance[3];
    // 融合参数调整数据
    float weight_radar;
    float weight_imu;
    float weight_mag;
};
#pragma pack(pop)

// 全局变量
BLEScan* pBLEScan;
std::map<uint8_t, BLEClient*> connectedDevices;

// SH1106 OLED屏幕对象 (I2C接口)
// 使用GPIO11(SDA), GPIO12(SCL), GPIO10(RST)
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, 10, 12, 11);

// NeoPixel LED对象 (GPIO48 - 已焊接)
#define LED_PIN 48
Adafruit_NeoPixel led(1, LED_PIN, NEO_GRB + NEO_KHZ800);

// ============================================================================
// ⚠️  WARNING: GPIO4 and GPIO5 may be DAMAGED! ⚠️
// Reason: 5V was accidentally applied to these pins earlier.
// These pins may not work correctly for input/output operations.
// Use the following alternative pins instead:
// ============================================================================
// 按键定义 - 使用安全引脚 (避开可能损坏的GPIO4/GPIO5)
#define BUTTON_PAGE_UP   13   // GPIO13 - 下一页 (安全引脚)
#define BUTTON_PAGE_DOWN 14   // GPIO14 - 上一页 (安全引脚)
#define BUTTON_DEBOUNCE_MS 50 // 消抖时间
// ============================================================================
// ⚠️  DO NOT USE GPIO4 OR GPIO5 FOR CRITICAL FUNCTIONS ⚠️
// They may have been damaged by overvoltage and may behave unpredictably.
// ============================================================================

// 页面定义
enum DisplayPage {
    PAGE_STATUS = 0,    // 状态页
    PAGE_NODES,        // 节点列表页
    PAGE_RADAR,        // 雷达数据页
    PAGE_SETTINGS,     // 设置页
    PAGE_COUNT         // 页面总数
};

DisplayPage currentPage = PAGE_STATUS;

// 按键状态
struct ButtonState {
    bool pressed;
    unsigned long lastPressTime;
};

ButtonState btnUp = {false, 0};
ButtonState btnDown = {false, 0};

// LED状态控制
enum LEDState {
    LED_OFF,
    LED_GREEN_ON,      // 启动时绿色常亮
    LED_RED_FLASH,     // 连接成功时红色闪烁
    LED_GREEN_FLASH,   // 连接断开时绿色闪烁
    LED_BLUE_FLASH     // 数据传输时蓝色闪烁
};

LEDState currentLEDState = LED_OFF;
unsigned long ledStateStartTime = 0;
int ledFlashCount = 0;
bool ledFlashState = false;

// LED控制函数
void setLEDState(LEDState state, int flashes = 0) {
    currentLEDState = state;
    ledFlashCount = flashes;
    ledStateStartTime = millis();
    ledFlashState = true;
}

void updateLED() {
    switch (currentLEDState) {
        case LED_OFF:
            led.setPixelColor(0, led.Color(0, 0, 0));
            break;
        case LED_GREEN_ON:
            led.setPixelColor(0, led.Color(0, 255, 0));
            break;
        case LED_RED_FLASH:
            if (ledFlashCount > 0) {
                if (ledFlashState) {
                    led.setPixelColor(0, led.Color(255, 0, 0));
                } else {
                    led.setPixelColor(0, led.Color(0, 0, 0));
                }
                if (millis() - ledStateStartTime > 200) {
                    ledFlashState = !ledFlashState;
                    ledStateStartTime = millis();
                    if (!ledFlashState) {
                        ledFlashCount--;
                        if (ledFlashCount <= 0) {
                            currentLEDState = LED_OFF;
                        }
                    }
                }
            }
            break;
        case LED_GREEN_FLASH:
            if (ledFlashCount > 0) {
                if (ledFlashState) {
                    led.setPixelColor(0, led.Color(0, 255, 0));
                } else {
                    led.setPixelColor(0, led.Color(0, 0, 0));
                }
                if (millis() - ledStateStartTime > 200) {
                    ledFlashState = !ledFlashState;
                    ledStateStartTime = millis();
                    if (!ledFlashState) {
                        ledFlashCount--;
                        if (ledFlashCount <= 0) {
                            currentLEDState = LED_OFF;
                        }
                    }
                }
            }
            break;
        case LED_BLUE_FLASH:
            if (ledFlashState) {
                led.setPixelColor(0, led.Color(0, 0, 255));
            } else {
                led.setPixelColor(0, led.Color(0, 0, 0));
            }
            if (millis() - ledStateStartTime > 50) {
                ledFlashState = false;
                currentLEDState = LED_OFF;
            }
            break;
    }
    led.show();
}

// 按键扫描函数
void updateButtons() {
    unsigned long now = millis();

    bool upPressed = (digitalRead(BUTTON_PAGE_UP) == LOW);
    bool downPressed = (digitalRead(BUTTON_PAGE_DOWN) == LOW);

    if (upPressed && !btnUp.pressed) {
        if ((now - btnUp.lastPressTime) > BUTTON_DEBOUNCE_MS) {
            btnUp.pressed = true;
            btnUp.lastPressTime = now;
            currentPage = (DisplayPage)((currentPage + 1) % PAGE_COUNT);
        }
    } else if (!upPressed) {
        btnUp.pressed = false;
    }

    if (downPressed && !btnDown.pressed) {
        if ((now - btnDown.lastPressTime) > BUTTON_DEBOUNCE_MS) {
            btnDown.pressed = true;
            btnDown.lastPressTime = now;
            currentPage = (DisplayPage)((currentPage - 1 + PAGE_COUNT) % PAGE_COUNT);
        }
    } else if (!downPressed) {
        btnDown.pressed = false;
    }
}

// 通信队列
QueueHandle_t bleToUsbQueue;
QueueHandle_t usbToBleQueue;
QueueHandle_t radarToUsbQueue;

// 雷达数据
RadarData latestRadarData;
bool radarDataUpdated = false;

// 融合权重（从Win端接收）
float weightRadar = 0.3f;
float weightIMU = 0.6f;
float weightMag = 0.1f;

// 任务句柄
TaskHandle_t bleTaskHandle;
TaskHandle_t usbTaskHandle;
TaskHandle_t reconnectTaskHandle;
TaskHandle_t displayTaskHandle;
TaskHandle_t radarTaskHandle;

class MyClientCallbacks: public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) {
        Serial.println("Peripheral connected");
        setLEDState(LED_RED_FLASH, 4); // 红色闪烁2次（每个闪烁包含亮和灭，所以4次）
    }
    void onDisconnect(BLEClient* pclient) {
        Serial.println("Peripheral disconnected");
        setLEDState(LED_GREEN_FLASH, 2); // 绿色闪烁1次
    }
};

// 发射功率控制
void setDevicePowerLevel(uint8_t node_id, uint8_t power_level) {
    if (connectedDevices.find(node_id) != connectedDevices.end()) {
        BLEClient* pClient = connectedDevices[node_id];
        if (pClient->isConnected()) {
            BLERemoteService* pRemoteService = pClient->getService(BLE_SERVICE_UUID);
            if (pRemoteService) {
                BLERemoteCharacteristic* pPowerChar = pRemoteService->getCharacteristic("a3c87501-8ed3-4bdf-8a39-a01bebede296");
                if (pPowerChar) {
                    pPowerChar->writeValue(&power_level, 1);
                    Serial.printf("Set power level for node %d to %d\n", node_id, power_level);
                }
            }
        }
    }
}

bool connectToDevice(const String& deviceName, uint8_t node_id) {
    Serial.print("Searching for device: ");
    Serial.println(deviceName);

    BLEScanResults devices = pBLEScan->start(5);
    BLEAdvertisedDevice targetDevice;
    bool found = false;

    for (int i = 0; i < devices.getCount(); i++) {
        BLEAdvertisedDevice device = devices.getDevice(i);
        if (device.getName() == deviceName.c_str()) {
            targetDevice = device;
            found = true;
            break;
        }
    }

    if (!found) {
        Serial.println("Target device not found");
        return false;
    }

    Serial.print("Connecting to: ");
    Serial.println(targetDevice.getName().c_str());

    BLEClient* pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new MyClientCallbacks());
    pClient->connect(&targetDevice);

    BLERemoteService* pRemoteService = pClient->getService(BLE_SERVICE_UUID);
    if (pRemoteService == nullptr) {
        Serial.println("Failed to find service");
        pClient->disconnect();
        return false;
    }

    BLERemoteCharacteristic* pMocapChar = pRemoteService->getCharacteristic(BLE_CHARACTERISTIC_UUID);
    BLERemoteCharacteristic* pOTAChar = pRemoteService->getCharacteristic(BLE_OTA_CHARACTERISTIC_UUID);

    if (pMocapChar == nullptr || pOTAChar == nullptr) {
        Serial.println("Failed to find characteristics");
        pClient->disconnect();
        return false;
    }

    pMocapChar->registerForNotify([](BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
        if (length == sizeof(MocapPacket)) {
            xQueueSend(bleToUsbQueue, pData, portMAX_DELAY);
        }
    });

    pOTAChar->registerForNotify([](BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
        if (length == sizeof(OTAAckPacket)) {
            xQueueSend(bleToUsbQueue, pData, portMAX_DELAY);
        }
    });

    connectedDevices[node_id] = pClient;
    Serial.print("Device connected. Node ID: ");
    Serial.println(node_id);
    return true;
}

// BLE 处理任务（运行在 Core 0）
void bleTask(void* parameter) {
    Serial.println("BLE task started on Core 0");

    // 初始化 BLE
    BLEDevice::init("MotionSnap-Central");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setActiveScan(true);

    // 连接到所有已知设备
    connectToDevice("MotionSnap-VRNode", 1);
    connectToDevice("MotionSnap-Controller", 2);
    connectToDevice("MotionSnap-VHMD", 16);

    Serial.println("BLE initialized, waiting for devices...");

    while (1) {
        // 处理从 USB 来的命令（OTA 数据和功率控制）
        uint8_t usbBuffer[sizeof(OTAPacket)];
        if (xQueueReceive(usbToBleQueue, usbBuffer, 5)) {
            // 检查是否是功率控制命令
            if (usbBuffer[0] == 'P' && sizeof(usbBuffer) >= 3) {
                uint8_t node_id = usbBuffer[1];
                uint8_t power_level = usbBuffer[2];
                if (power_level >= 0 && power_level <= 7) {
                    setDevicePowerLevel(node_id, power_level);
                    // 发送确认
                    uint8_t ackBuffer[3] = {'A', node_id, power_level};
                    xQueueSend(bleToUsbQueue, ackBuffer, portMAX_DELAY);
                }
            } else {
                // 处理 OTA 数据
                OTAPacket* packet = (OTAPacket*)usbBuffer;
                uint8_t node_id = packet->header.node_id;

                Serial.print("OTA packet received for node: ");
                Serial.println(node_id);

                // 转发到对应的从节点
                if (connectedDevices.find(node_id) != connectedDevices.end()) {
                    BLEClient* pClient = connectedDevices[node_id];
                    if (pClient->isConnected()) {
                        BLERemoteService* pRemoteService = pClient->getService(BLE_SERVICE_UUID);
                        if (pRemoteService) {
                            BLERemoteCharacteristic* pOTAChar = pRemoteService->getCharacteristic(BLE_OTA_CHARACTERISTIC_UUID);
                            if (pOTAChar) {
                                pOTAChar->writeValue(usbBuffer, sizeof(OTAPacket));
                                Serial.println("OTA packet forwarded");
                            }
                        }
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// USB 处理任务（运行在 Core 1）
void usbTask(void* parameter) {
    Serial.println("USB task started on Core 1");

    static uint8_t currentSeq = 0;

    while (1) {
        // 处理从 BLE 来的数据（Mocap 数据包）
        uint8_t bleBuffer[sizeof(OTAPacket)];
        if (xQueueReceive(bleToUsbQueue, bleBuffer, 0)) {
            MocapPacket* mocapPacket = (MocapPacket*)bleBuffer;

            // 创建扩展数据包
            ExtendedMocapPacket extendedPacket;
            extendedPacket.packet_type = 1;  // 0=纯IMU, 1=IMU+雷达
            extendedPacket.node_id = mocapPacket->node_id;
            extendedPacket.seq = currentSeq++;
            extendedPacket.timestamp = mocapPacket->timestamp;
            extendedPacket.gyro_x = mocapPacket->gyro_x;
            extendedPacket.gyro_y = mocapPacket->gyro_y;
            extendedPacket.gyro_z = mocapPacket->gyro_z;
            extendedPacket.accel_x = mocapPacket->accel_x;
            extendedPacket.accel_y = mocapPacket->accel_y;
            extendedPacket.accel_z = mocapPacket->accel_z;
            extendedPacket.mag_x = mocapPacket->mag_x;
            extendedPacket.mag_y = mocapPacket->mag_y;
            extendedPacket.mag_z = mocapPacket->mag_z;
            extendedPacket.buttons = mocapPacket->buttons;
            extendedPacket.joystick_x = mocapPacket->joystick_x;
            extendedPacket.joystick_y = mocapPacket->joystick_y;

            // 添加雷达数据
            extendedPacket.radar_target_count = latestRadarData.target_count;
            extendedPacket.radar_distance[0] = latestRadarData.distance_1;
            extendedPacket.radar_distance[1] = latestRadarData.distance_2;
            extendedPacket.radar_distance[2] = latestRadarData.distance_3;

            // 添加融合权重
            extendedPacket.weight_radar = weightRadar;
            extendedPacket.weight_imu = weightIMU;
            extendedPacket.weight_mag = weightMag;

            // 发送扩展数据包
            Serial.write((uint8_t*)&extendedPacket, sizeof(ExtendedMocapPacket));
        }

        // 如果没有BLE数据，但有雷达数据更新，也可以单独发送雷达数据
        if (radarDataUpdated) {
            RadarData radarPacket = latestRadarData;
            // 发送雷达数据（使用自定义前缀）
            uint8_t radarPrefix[2] = {'R', 'D'};
            Serial.write(radarPrefix, 2);
            Serial.write((uint8_t*)&radarPacket, sizeof(RadarData));
            radarDataUpdated = false;
        }

        // 处理从 PC 来的命令
        if (Serial.available()) {
            // 检查是否是功率控制命令
            if (Serial.available() >= 3) {
                uint8_t command = Serial.read();
                if (command == 'P') { // Power control command
                    uint8_t node_id = Serial.read();
                    uint8_t power_level = Serial.read();
                    if (power_level >= 0 && power_level <= 7) {
                        uint8_t cmdBuffer[3] = {'P', node_id, power_level};
                        xQueueSend(usbToBleQueue, cmdBuffer, portMAX_DELAY);
                    }
                } else if (command == 'W') { // Weight adjustment command
                    // 格式: W + 3个float (12 bytes)
                    if (Serial.available() >= 12) {
                        uint8_t weightBuffer[12];
                        size_t bytesRead = Serial.readBytes(weightBuffer, 12);
                        if (bytesRead == 12) {
                            float* weights = (float*)weightBuffer;
                            weightRadar = weights[0];
                            weightIMU = weights[1];
                            weightMag = weights[2];
                            
                            // 发送确认
                            uint8_t ackBuffer[2] = {'W', 'A'};
                            Serial.write(ackBuffer, 2);
                        }
                    }
                } else {
                    // 回退读取，处理 OTA 数据
                    Serial.write(command); // 把命令字符写回缓冲区
                }
            }

            // 处理 OTA 数据
            if (Serial.available() >= sizeof(OTAPacket)) {
                uint8_t serialBuffer[sizeof(OTAPacket)];
                size_t bytesRead = Serial.readBytes(serialBuffer, sizeof(OTAPacket));

                if (bytesRead == sizeof(OTAPacket)) {
                    xQueueSend(usbToBleQueue, serialBuffer, portMAX_DELAY);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// 重连任务（运行在 Core 0）
void reconnectTask(void* parameter) {
    Serial.println("Reconnect task started on Core 0");

    unsigned long lastReconnectCheck = 0;

    while (1) {
        // 定期重连断开的设备（每 5 秒检查一次）
        if (millis() - lastReconnectCheck > 5000) {
            for (auto& device : connectedDevices) {
                uint8_t node_id = device.first;
                BLEClient* pClient = device.second;
                if (pClient && !pClient->isConnected()) {
                    Serial.print("Reconnecting to node: ");
                    Serial.println(node_id);
                    String deviceName;
                    switch (node_id) {
                        case 1: deviceName = "MotionSnap-VRNode"; break;
                        case 2: deviceName = "MotionSnap-Controller"; break;
                        case 16: deviceName = "MotionSnap-VHMD"; break;
                        default: deviceName = "MotionSnap-Node" + String(node_id);
                    }
                    connectToDevice(deviceName, node_id);
                }
            }
            lastReconnectCheck = millis();
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// 雷达数据读取任务（运行在 Core 1）
void radarTask(void* parameter) {
    Serial.println("Radar task started on Core 1");

    // 初始化UART1用于LD2402 (GPIO43=TX, GPIO44=RX)
    Serial1.begin(256000, SERIAL_8N1, 44, 43);

    while (1) {
        if (Serial1.available()) {
            String line = Serial1.readStringUntil('\n');

            if (line.startsWith("$LD2402")) {
                // 解析LD2402数据格式
                int firstComma = line.indexOf(',');
                if (firstComma != -1) {
                    String targetCountStr = line.substring(firstComma + 1, firstComma + 2);
                    int targetCount = targetCountStr.toInt();

                    latestRadarData.node_id = 0;  // 雷达属于中央节点
                    latestRadarData.target_count = targetCount;

                    // 解析距离数据
                    int lastPos = firstComma + 2;
                    for (int i = 0; i < targetCount && i < 3; i++) {
                        int commaPos = line.indexOf(',', lastPos);
                        if (commaPos == -1) break;
                        
                        String distStr = line.substring(lastPos, commaPos);
                        if (i == 0) latestRadarData.distance_1 = distStr.toFloat();
                        if (i == 1) latestRadarData.distance_2 = distStr.toFloat();
                        if (i == 2) latestRadarData.distance_3 = distStr.toFloat();
                        
                        lastPos = commaPos + 1;
                    }

                    // 填充剩余距离为0
                    if (targetCount < 3) {
                        if (targetCount < 2) latestRadarData.distance_2 = 0;
                        latestRadarData.distance_3 = 0;
                    }

                    radarDataUpdated = true;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// 显示任务（运行在 Core 1）
void drawStatusPage() {
    char buf[32];
    display.setFont(u8g2_font_5x7_tr);
    snprintf(buf, sizeof(buf), "Chip: %s", ESP.getChipModel());
    display.drawStr(0, 22, buf);
    snprintf(buf, sizeof(buf), "Cores: %d", ESP.getChipCores());
    display.drawStr(0, 32, buf);
    snprintf(buf, sizeof(buf), "Nodes: %d", connectedDevices.size());
    display.drawStr(0, 42, buf);
    
    int yPos = 54;
    display.setFont(u8g2_font_4x6_tr);
    for (auto& device : connectedDevices) {
        uint8_t node_id = device.first;
        BLEClient* pClient = device.second;
        if (pClient->isConnected()) {
            snprintf(buf, sizeof(buf), "N%d: OK", node_id);
        } else {
            snprintf(buf, sizeof(buf), "N%d: ERR", node_id);
        }
        display.drawStr(0, yPos, buf);
        yPos += 8;
    }
}

void drawNodesPage() {
    char buf[32];
    display.setFont(u8g2_font_5x7_tr);
    display.drawStr(0, 22, "Connected Nodes:");
    
    int yPos = 34;
    display.setFont(u8g2_font_4x6_tr);
    
    if (connectedDevices.empty()) {
        display.drawStr(0, yPos, "No nodes connected");
    } else {
        for (auto& device : connectedDevices) {
            uint8_t node_id = device.first;
            BLEClient* pClient = device.second;
            if (pClient->isConnected()) {
                snprintf(buf, sizeof(buf), "Node %d: Connected", node_id);
            } else {
                snprintf(buf, sizeof(buf), "Node %d: Disconnected", node_id);
            }
            display.drawStr(0, yPos, buf);
            yPos += 10;
            if (yPos > 60) break;
        }
    }
}

void drawRadarPage() {
    char buf[32];
    display.setFont(u8g2_font_5x7_tr);
    display.drawStr(0, 22, "Radar Data:");
    
    display.setFont(u8g2_font_4x6_tr);
    snprintf(buf, sizeof(buf), "Distance: -- cm");
    display.drawStr(0, 34, buf);
    snprintf(buf, sizeof(buf), "Velocity: -- m/s");
    display.drawStr(0, 46, buf);
    snprintf(buf, sizeof(buf), "Angle: -- deg");
    display.drawStr(0, 58, buf);
}

void drawSettingsPage() {
    char buf[32];
    display.setFont(u8g2_font_5x7_tr);
    display.drawStr(0, 22, "Settings:");
    
    display.setFont(u8g2_font_4x6_tr);
    snprintf(buf, sizeof(buf), "Btn1: GPIO%d", BUTTON_PAGE_UP);
    display.drawStr(0, 34, buf);
    snprintf(buf, sizeof(buf), "Btn2: GPIO%d", BUTTON_PAGE_DOWN);
    display.drawStr(0, 46, buf);
    snprintf(buf, sizeof(buf), "LED: GPIO%d", LED_PIN);
    display.drawStr(0, 58, buf);
}

void displayTask(void* parameter) {
    Serial.println("Display task started on Core 1");

    display.begin();
    display.clearBuffer();
    display.setFont(u8g2_font_ncenB08_tr);
    display.drawStr(0, 16, "MotionSnap Central");
    display.drawStr(0, 28, "ESP32 S3");
    display.drawStr(0, 40, "Initializing...");
    display.sendBuffer();

    while(1) {
        display.clearBuffer();
        
        display.setFont(u8g2_font_ncenB08_tr);
        display.drawStr(0, 10, "MotionSnap Central");
        
        switch(currentPage) {
            case PAGE_STATUS:
                drawStatusPage();
                break;
            case PAGE_NODES:
                drawNodesPage();
                break;
            case PAGE_RADAR:
                drawRadarPage();
                break;
            case PAGE_SETTINGS:
                drawSettingsPage();
                break;
            default:
                drawStatusPage();
                break;
        }
        
        display.sendBuffer();
        
        updateButtons();
        updateLED();
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void setup() {
    // 初始化 USB CDC 串口（ESP32 S3 专用）
    Serial.begin(115200);
    delay(1000);

    Serial.println("=== MotionSnap BLE Central Node (ESP32 S3 USB CDC) ===");
    Serial.print("Chip Model: ");
    Serial.println(ESP.getChipModel());
    Serial.print("Core count: ");
    Serial.println(ESP.getChipCores());

    // 初始化NeoPixel LED
    led.begin();
    led.setBrightness(50);
    led.setPixelColor(0, led.Color(0, 255, 0)); // 绿色常亮
    led.show();
    Serial.println("LED: Green (Starting...)");

    // 5秒后关闭绿色常亮
    delay(5000);
    led.setPixelColor(0, led.Color(0, 0, 0));
    led.show();
    Serial.println("LED: Ready");

    // 初始化按键（使用内部上拉，按下时为低电平）
    pinMode(BUTTON_PAGE_UP, INPUT_PULLUP);
    pinMode(BUTTON_PAGE_DOWN, INPUT_PULLUP);
    Serial.printf("Buttons initialized: GPIO%d(Next), GPIO%d(Prev)\n", BUTTON_PAGE_UP, BUTTON_PAGE_DOWN);

    // 创建队列
    bleToUsbQueue = xQueueCreate(20, sizeof(OTAPacket));
    usbToBleQueue = xQueueCreate(20, sizeof(OTAPacket));
    radarToUsbQueue = xQueueCreate(10, sizeof(RadarData));

    // 创建 BLE 任务（Core 0）
    xTaskCreatePinnedToCore(
        bleTask,
        "bleTask",
        8192,
        NULL,
        5,
        &bleTaskHandle,
        0
    );

    // 创建 USB 任务（Core 1）
    xTaskCreatePinnedToCore(
        usbTask,
        "usbTask",
        8192,
        NULL,
        4,
        &usbTaskHandle,
        1
    );

    // 创建重连任务（Core 0）
    xTaskCreatePinnedToCore(
        reconnectTask,
        "reconnectTask",
        4096,
        NULL,
        3,
        &reconnectTaskHandle,
        0
    );

    // 创建显示任务（Core 1）
    xTaskCreatePinnedToCore(
        displayTask,
        "displayTask",
        8192,
        NULL,
        2,
        &displayTaskHandle,
        1
    );

    // 创建雷达任务（Core 1）
    xTaskCreatePinnedToCore(
        radarTask,
        "radarTask",
        4096,
        NULL,
        2,
        &radarTaskHandle,
        1
    );

    Serial.println("Multi-core setup completed");
}

void loop() {
    // 主循环空闲
    delay(1000);
}