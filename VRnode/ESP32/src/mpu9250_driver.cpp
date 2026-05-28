#include "mpu9250_driver.h"
#include <SPI.h>
#include <Arduino.h>

static const char* TAG = "MPU9250";

// SPI读写函数
static void spiWrite(uint8_t reg, uint8_t value) {
    digitalWrite(MPU9250_SPI_CS, LOW);
    SPI.transfer(reg);
    SPI.transfer(value);
    digitalWrite(MPU9250_SPI_CS, HIGH);
}

static uint8_t spiRead(uint8_t reg) {
    digitalWrite(MPU9250_SPI_CS, LOW);
    SPI.transfer(reg | 0x80); // 读操作
    uint8_t value = SPI.transfer(0);
    digitalWrite(MPU9250_SPI_CS, HIGH);
    return value;
}

static void spiReadBurst(uint8_t reg, uint8_t* data, size_t length) {
    digitalWrite(MPU9250_SPI_CS, LOW);
    SPI.transfer(reg | 0x80); // 读操作
    for (size_t i = 0; i < length; i++) {
        data[i] = SPI.transfer(0);
    }
    digitalWrite(MPU9250_SPI_CS, HIGH);
}

// 初始化SPI
static bool spi_init(void) {
    // 初始化SPI，高速模式（0.2mil线长差异支持高速传输）
    SPI.begin(MPU9250_SPI_SCK, MPU9250_SPI_MISO, MPU9250_SPI_MOSI, MPU9250_SPI_CS);
    SPI.setClockDivider(SPI_CLOCK_DIV2); // 8MHz - 高速模式，配合等长布线
    SPI.setDataMode(SPI_MODE3); // CPOL=1, CPHA=1 - MPU6050推荐模式
    SPI.setBitOrder(MSBFIRST);
    
    pinMode(MPU9250_SPI_CS, OUTPUT);
    digitalWrite(MPU9250_SPI_CS, HIGH);
    delay(100);
    return true;
}

// 初始化传感器
bool mpu9250_init(void) {
    // 初始化SPI
    if (!spi_init()) {
        Serial.println("Failed to initialize SPI");
        return false;
    }

    // 读取WHO_AM_I寄存器
    uint8_t who_am_i = spiRead(MPU9250_WHO_AM_I);
    // MPU9250的WHO_AM_I是0x71，MPU6500是0x70
    if (who_am_i != 0x71 && who_am_i != 0x70) {
        Serial.printf("MPU9250/MPU6500 not found! WHO_AM_I = 0x%02X\n", who_am_i);
        return false;
    }
    Serial.printf("MPU sensor found! WHO_AM_I = 0x%02X\n", who_am_i);

    // 复位MPU9250
    spiWrite(MPU9250_PWR_MGMT_1, 0x80); // 复位
    delay(100);

    // 唤醒MPU9250
    spiWrite(MPU9250_PWR_MGMT_1, 0x00); // 唤醒，使用内部时钟
    delay(100);

    // 设置采样率
    spiWrite(MPU9250_SMPLRT_DIV, 0x00); // 采样率 = 1kHz

    // 设置低通滤波器
    spiWrite(MPU9250_CONFIG, 0x03); // 低通滤波器截止频率 41Hz

    // 设置陀螺仪量程
    spiWrite(MPU9250_GYRO_CONFIG, 0x00); // ±250°/s

    // 设置加速度计量程
    spiWrite(MPU9250_ACCEL_CONFIG, 0x00); // ±2g

    // 启用I2C主模式（用于读取磁力计）
    spiWrite(MPU9250_INT_PIN_CFG, 0x02);

    Serial.println("MPU9250 initialized successfully!");
    return true;
}

// 读取传感器数据
MPU9250Data mpu9250_read_data(void) {
    MPU9250Data data = {0};

    // 读取加速度计和陀螺仪数据
    uint8_t raw_data[14];
    spiReadBurst(MPU9250_ACCEL_XOUT_H, raw_data, 14); // 加速度计(6字节) + 温度(2字节) + 陀螺仪(6字节)

    // 读取加速度计数据
    int16_t accel_x = (raw_data[0] << 8) | raw_data[1];
    int16_t accel_y = (raw_data[2] << 8) | raw_data[3];
    int16_t accel_z = (raw_data[4] << 8) | raw_data[5];

    // 跳过温度数据 (raw_data[6], raw_data[7])

    // 读取陀螺仪数据
    int16_t gyro_x = (raw_data[8] << 8) | raw_data[9];
    int16_t gyro_y = (raw_data[10] << 8) | raw_data[11];
    int16_t gyro_z = (raw_data[12] << 8) | raw_data[13];

    // 转换为物理单位
    float gyro_scale = 1.0 / 131.0 * M_PI / 180.0; // 陀螺仪量程 ±250°/s
    float accel_scale = 1.0 / 16384.0 * 9.81; // 加速度计量程 ±2g

    data.gyro_x = gyro_x * gyro_scale;
    data.gyro_y = gyro_y * gyro_scale;
    data.gyro_z = gyro_z * gyro_scale;
    data.accel_x = accel_x * accel_scale;
    data.accel_y = accel_y * accel_scale;
    data.accel_z = accel_z * accel_scale;

    // 磁力计数据暂时设为0，需要额外的I2C通信来读取
    data.mag_x = 0.0f;
    data.mag_y = 0.0f;
    data.mag_z = 0.0f;

    return data;
}

// 获取磁力计校准系数
std::array<float, 3> mpu9250_get_mag_calib(void) {
    // 暂时返回默认值
    return {1.0f, 1.0f, 1.0f};
}