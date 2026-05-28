#pragma once

#include <cstdint>
#include <array>

// ICM20948 I2C地址
#define ICM20948_ADDR 0x68

// AK09916（磁力计）I2C地址
#define AK09916_ADDR 0x0C

// ICM20948寄存器地址
#define ICM20948_BANK_SEL      0x7F
#define ICM20948_WHO_AM_I      0x00
#define ICM20948_USER_CTRL     0x03
#define ICM20948_PWR_MGMT_1    0x06
#define ICM20948_PWR_MGMT_2    0x07
#define ICM20948_CONFIG        0x1A
#define ICM20948_GYRO_CONFIG   0x1B
#define ICM20948_ACCEL_CONFIG  0x1C
#define ICM20948_ACCEL_XOUT_H  0x2D
#define ICM20948_GYRO_XOUT_H   0x33

// AK09916寄存器地址
#define AK09916_WHO_AM_I  0x01
#define AK09916_ST1       0x10
#define AK09916_HXL       0x11
#define AK09916_CNTL2     0x31

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
} ICM20948Data;

// 初始化传感器
bool icm20948_init(void);

// 读取传感器数据
ICM20948Data icm20948_read_data(void);