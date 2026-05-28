#include "icm20948_driver.h"

#include "esp_system.h"
#include "driver/i2c.h"
#include "esp_log.h"

#include <cstring>
#include <cmath>

static const char* TAG = "ICM20948";

// I2C配置
#define I2C_MASTER_SCL_IO 2      // GPIO2作为SCL
#define I2C_MASTER_SDA_IO 1      // GPIO1作为SDA
#define I2C_MASTER_FREQ_HZ 400000 // 400kHz I2C时钟
#define I2C_MASTER_NUM I2C_NUM_0 // I2C端口0

// 校准系数
static float gyro_scale = 1.0f / 16.4f * M_PI / 180.0f; // 陀螺仪量程 ±2000°/s
static float accel_scale = 1.0f / 2048.0f * 9.81f; // 加速度计量程 ±16g

// I2C写入函数
static esp_err_t i2c_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// I2C读取函数
static esp_err_t i2c_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// 初始化I2C
bool i2c_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return false;
    }

    return true;
}

// 初始化ICM20948
bool icm20948_init(void) {
    // 初始化I2C
    if (!i2c_init()) {
        return false;
    }

    // 切换到Bank0
    int retry = 3;
    uint8_t who_am_i = 0;
    bool bank_switch_success = false;
    
    while (retry > 0 && !bank_switch_success) {
        // 写寄存器0x7F（BANK_SEL）值为0x00（切换到Bank0）
        if (i2c_write_byte(ICM20948_ADDR, ICM20948_BANK_SEL, 0x00) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to switch to Bank0");
            retry--;
            continue;
        }
        
        // 增加100μs延时，确保Bank切换完成
        ets_delay_us(100);
        
        // 读取WHO_AM_I寄存器
        if (i2c_read_bytes(ICM20948_ADDR, ICM20948_WHO_AM_I, &who_am_i, 1) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read ICM20948 WHO_AM_I");
            retry--;
            continue;
        }
        
        bank_switch_success = true;
    }
    
    if (!bank_switch_success) {
        ESP_LOGE(TAG, "Bank0 switching failed after 3 retries");
        return false;
    }

    if (who_am_i != 0xE1) {
        ESP_LOGE(TAG, "WHO_AM_I check failed! Read: 0x%02X, Expected: 0xE1", who_am_i);
        return false;
    }

    // 复位ICM20948
    if (i2c_write_byte(ICM20948_ADDR, ICM20948_PWR_MGMT_1, 0x80) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset ICM20948");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // 唤醒ICM20948
    if (i2c_write_byte(ICM20948_ADDR, ICM20948_PWR_MGMT_1, 0x01) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake ICM20948");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // 启用加速度计和陀螺仪
    if (i2c_write_byte(ICM20948_ADDR, ICM20948_PWR_MGMT_2, 0x00) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable sensors");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // 设置低通滤波器
    if (i2c_write_byte(ICM20948_ADDR, ICM20948_CONFIG, 0x03) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set config");
        return false;
    }

    // 设置陀螺仪量程 ±2000°/s
    if (i2c_write_byte(ICM20948_ADDR, ICM20948_GYRO_CONFIG, 0x18) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set gyro config");
        return false;
    }

    // 设置加速度计量程 ±16g
    if (i2c_write_byte(ICM20948_ADDR, ICM20948_ACCEL_CONFIG, 0x18) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set accel config");
        return false;
    }

    // 启用I2C主模式
    if (i2c_write_byte(ICM20948_ADDR, ICM20948_USER_CTRL, 0x20) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2C master mode");
        return false;
    }

    // 初始化AK09916磁力计
    // 检查AK09916 WHO_AM_I
    uint8_t ak09916_who_am_i;
    if (i2c_read_bytes(AK09916_ADDR, AK09916_WHO_AM_I, &ak09916_who_am_i, 1) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read AK09916 WHO_AM_I");
        return false;
    }

    if (ak09916_who_am_i != 0x09) {
        ESP_LOGE(TAG, "AK09916 not found, WHO_AM_I: 0x%02X", ak09916_who_am_i);
        return false;
    }

    // 设置磁力计模式为连续测量
    if (i2c_write_byte(AK09916_ADDR, AK09916_CNTL2, 0x08) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set AK09916 mode");
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "ICM20948 initialized successfully");
    return true;
}

// 读取ICM20948数据
ICM20948Data icm20948_read_data(void) {
    ICM20948Data data = {0};

    // 读取加速度计和陀螺仪数据
    uint8_t raw_data[12];
    if (i2c_read_bytes(ICM20948_ADDR, ICM20948_ACCEL_XOUT_H, raw_data, 12) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read accel and gyro data");
        return data;
    }

    // 解析加速度计数据
    int16_t accel_x = (raw_data[0] << 8) | raw_data[1];
    int16_t accel_y = (raw_data[2] << 8) | raw_data[3];
    int16_t accel_z = (raw_data[4] << 8) | raw_data[5];

    // 解析陀螺仪数据
    int16_t gyro_x = (raw_data[6] << 8) | raw_data[7];
    int16_t gyro_y = (raw_data[8] << 8) | raw_data[9];
    int16_t gyro_z = (raw_data[10] << 8) | raw_data[11];

    // 转换为物理单位
    data.accel_x = accel_x * accel_scale;
    data.accel_y = accel_y * accel_scale;
    data.accel_z = accel_z * accel_scale;
    data.gyro_x = gyro_x * gyro_scale;
    data.gyro_y = gyro_y * gyro_scale;
    data.gyro_z = gyro_z * gyro_scale;

    // 读取磁力计数据
    // 首先检查数据是否准备好
    uint8_t st1;
    if (i2c_read_bytes(AK09916_ADDR, AK09916_ST1, &st1, 1) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read AK09916 ST1");
        return data;
    }

    if (st1 & 0x01) { // 数据就绪
        uint8_t mag_data[6];
        if (i2c_read_bytes(AK09916_ADDR, AK09916_HXL, mag_data, 6) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read mag data");
            return data;
        }

        // 解析磁力计数据
        int16_t mag_x = (mag_data[1] << 8) | mag_data[0];
        int16_t mag_y = (mag_data[3] << 8) | mag_data[2];
        int16_t mag_z = (mag_data[5] << 8) | mag_data[4];

        data.mag_x = static_cast<float>(mag_x);
        data.mag_y = static_cast<float>(mag_y);
        data.mag_z = static_cast<float>(mag_z);

        // 读取ST2寄存器以清除数据就绪标志
        uint8_t st2;
        i2c_read_bytes(AK09916_ADDR, 0x18, &st2, 1);
    }

    return data;
}