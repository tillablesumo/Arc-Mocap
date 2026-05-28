#ifndef USER_SETUP_H
#define USER_SETUP_H

// GM009605v4.3 4.3" LCD with ST1315 controller
// I2C Interface for ESP32-S3

#define TFT_I2C

// I2C Configuration for ESP32-S3 (使用安全引脚)
#define TFT_SDA 11   // GPIO11 - 安全引脚
#define TFT_SCL 12   // GPIO12 - 安全引脚
#define TFT_RST 10   // GPIO10 - 安全引脚

// Display dimensions
#define TFT_WIDTH 480
#define TFT_HEIGHT 272

// Driver configuration
#define ST7789_DRIVER

// Color configuration
#define TFT_BLACK       0x0000
#define TFT_WHITE       0xFFFF
#define TFT_RED         0xF800
#define TFT_GREEN       0x07E0
#define TFT_BLUE        0x001F
#define TFT_CYAN        0x07FF
#define TFT_YELLOW      0xFFE0
#define TFT_COLOR_MODE   16

#endif