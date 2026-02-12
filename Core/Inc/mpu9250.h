#ifndef MPU9250_H
#define MPU9250_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef struct {
    float Ax, Ay, Az;          // g
    float Gx, Gy, Gz;          // deg/s

    float Ax_offset, Ay_offset, Az_offset; // g
    float Gx_offset, Gy_offset, Gz_offset; // deg/s

    uint8_t i2c_addr;          // 8-bit addr (0xD0 or 0xD2)
    uint8_t whoami;            // WHO_AM_I value read
    uint8_t accel_sanity_ok;   // 1 if accel magnitude ~1g (rough)
} MPU9250_t;

typedef enum {
    MPU_OK = 0,
    MPU_ERR_I2C = 1,
    MPU_ERR_WHOAMI = 2,
    MPU_ERR_CONFIG = 3,
    MPU_ERR_BAD_ARG = 4
} MPU_Status;

MPU_Status MPU9250_Init(I2C_HandleTypeDef *hi2c, MPU9250_t *mpu);
MPU_Status MPU9250_Read_Accel_Gyro(I2C_HandleTypeDef *hi2c, MPU9250_t *mpu);

MPU_Status MPU9250_Calibrate_Gyro(I2C_HandleTypeDef *hi2c, MPU9250_t *mpu, uint16_t samples);
MPU_Status MPU9250_Calibrate_Accel(I2C_HandleTypeDef *hi2c, MPU9250_t *mpu, uint16_t samples);

#endif // MPU9250_H
