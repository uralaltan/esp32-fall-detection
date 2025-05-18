#include "imu.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#define I2C_PORT      I2C_NUM_0
#define SDA_GPIO      21
#define SCL_GPIO      22
#define I2C_FREQ_HZ   400000

#define PWR_MGMT_1    0x6B
#define INT_PIN_CFG   0x37
#define ACCEL_CONFIG  0x1C
#define GYRO_CONFIG   0x1B

#define DEV_ADDR      MPU_ADDR

static const char *TAG = "IMU";

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;

static esp_err_t i2c_master_setup(void) {
    const i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .sda_io_num = SDA_GPIO,
        .scl_io_num = SCL_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DEV_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    return i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
}

static inline esp_err_t write_reg(const uint8_t reg, const uint8_t val) {
    const uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(dev_handle, buf, sizeof(buf), 1000);
}

static inline esp_err_t read_regs(const uint8_t reg, uint8_t *dst, const size_t len) {
    return i2c_master_transmit_receive(dev_handle, &reg, 1, dst, len, 1000);
}

esp_err_t imu_init(void) {
    ESP_LOGI(TAG, "Setting up I2C");
    ESP_ERROR_CHECK(i2c_master_setup());

    ESP_LOGI(TAG, "Waking MPU");
    ESP_ERROR_CHECK(write_reg(PWR_MGMT_1, 0x01));
    ESP_ERROR_CHECK(write_reg(INT_PIN_CFG, 0x02));
    ESP_ERROR_CHECK(write_reg(ACCEL_CONFIG,0x08));
    ESP_ERROR_CHECK(write_reg(GYRO_CONFIG, 0x08));

    uint8_t who = 0;
    ESP_ERROR_CHECK(read_regs(0x75, &who, 1));
    ESP_LOGI(TAG, "MPU WHOAMI = 0x%02X", who);

    return ESP_OK;
}

void imu_read_and_print(void) {
    uint8_t buf[14];
    ESP_ERROR_CHECK(read_regs(0x3B, buf, sizeof(buf)));

    const int16_t ax = (int16_t) ((buf[0] << 8) | buf[1]);
    const int16_t ay = (int16_t) ((buf[2] << 8) | buf[3]);
    const int16_t az = (int16_t) ((buf[4] << 8) | buf[5]);
    const int16_t gx = (int16_t) ((buf[8] << 8) | buf[9]);
    const int16_t gy = (int16_t) ((buf[10] << 8) | buf[11]);
    const int16_t gz = (int16_t) ((buf[12] << 8) | buf[13]);

    const float a_res = 4.0f / 32768.0f;
    const float g_res = 500.0f / 32768.0f;

    ESP_LOGI(TAG, "Acc: %.2f g, %.2f g, %.2f g\n", ax * a_res, ay * a_res, az * a_res);
    ESP_LOGI(TAG, "Gyr: %.2f °/s, %.2f °/s, %.2f °/s\n", gx * g_res, gy * g_res, gz * g_res);
}

void imu_read_raw(int16_t raw[6]) {
    uint8_t buf[14];
    ESP_ERROR_CHECK(read_regs(0x3B, buf, sizeof(buf)));

    raw[0] = (int16_t) ((buf[0] << 8) | buf[1]);
    raw[1] = (int16_t) ((buf[2] << 8) | buf[3]);
    raw[2] = (int16_t) ((buf[4] << 8) | buf[5]);
    raw[3] = (int16_t) ((buf[8] << 8) | buf[9]);
    raw[4] = (int16_t) ((buf[10] << 8) | buf[11]);
    raw[5] = (int16_t) ((buf[12] << 8) | buf[13]);
}
