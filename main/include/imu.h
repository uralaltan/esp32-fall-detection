#ifndef IMU_H
#define IMU_H

#include "esp_err.h"

#define MPU_ADDR    0x68

esp_err_t imu_init(void);

void imu_read_and_print(void);

void imu_read_raw(int16_t raw[6]);

#endif
