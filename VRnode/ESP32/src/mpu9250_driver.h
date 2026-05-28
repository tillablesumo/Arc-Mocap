#pragma once

#include <cstdint>
#include <array>

// SPI配置
#define MPU9250_SPI_SCK 4
#define MPU9250_SPI_MISO 5
#define MPU9250_SPI_MOSI 6
#define MPU9250_SPI_CS 7

// MPU9250寄存器地址
#define MPU9250_WHO_AM_I      0x75
#define MPU9250_PWR_MGMT_1    0x6B
#define MPU9250_PWR_MGMT_2    0x6C
#define MPU9250_SMPLRT_DIV    0x19
#define MPU9250_CONFIG        0x1A
#define MPU9250_GYRO_CONFIG   0x1B
#define MPU9250_ACCEL_CONFIG  0x1C
#define MPU9250_INT_PIN_CFG   0x37
#define MPU9250_INT_ENABLE    0x38
#define MPU9250_ACCEL_XOUT_H  0x3B
#define MPU9250_GYRO_XOUT_H   0x43
#define MPU9250_EXT_SENS_DATA 0x49
#define MPU9250_I2C_SLV0_ADDR 0x24
#define MPU9250_I2C_SLV0_REG  0x25
#define MPU9250_I2C_SLV0_CTRL 0x26
#define MPU9250_I2C_SLV0_DO   0x63

// AK8963寄存器地址
#define AK8963_WHO_AM_I  0x00
#define AK8963_ST1       0x02
#define AK8963_HXL       0x03
#define AK8963_CNTL1     0x0A
#define AK8963_ASAX      0x10

// 传感器数据结构体
typedef struct {
    float gyro_x; // 陀螺仪X轴数据 (rad/s)
    float gyro_y; // 陀螺仪Y轴数据 (rad/s)
    float gyro_z; // 陀螺仪Z轴数据 (rad/s)
    float accel_x; // 加速度计X轴数据 (m/s²)
    float accel_y; // 加速度计Y轴数据 (m/s²)
    float accel_z; // 加速度计Z轴数据 (m/s²)
    float mag_x; // 磁力计X轴数据
    float mag_y; // 磁力计Y轴数据
    float mag_z; // 磁力计Z轴数据
} MPU9250Data;

// 初始化传感器
bool mpu9250_init(void);

// 读取传感器数据
MPU9250Data mpu9250_read_data(void);

// 获取磁力计校准系数
std::array<float, 3> mpu9250_get_mag_calib(void);