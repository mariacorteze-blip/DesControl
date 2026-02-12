#include "mpu9250.h"
#include <math.h>
#include <string.h>

/* ---- Registers ---- */
#define REG_SMPLRT_DIV     0x19
#define REG_CONFIG         0x1A
#define REG_GYRO_CONFIG    0x1B
#define REG_ACCEL_CONFIG   0x1C
#define REG_ACCEL_XOUT_H   0x3B
#define REG_GYRO_XOUT_H    0x43
#define REG_PWR_MGMT_1     0x6B
#define REG_WHO_AM_I       0x75

/* ---- I2C addresses (8-bit for HAL) ---- */
#define MPU_ADDR_0x68      (0x68 << 1) // 0xD0
#define MPU_ADDR_0x69      (0x69 << 1) // 0xD2

/* ---- Moving average gyro ---- */
#define MOVING_AVG_SIZE 20
static float gx_buf[MOVING_AVG_SIZE], gy_buf[MOVING_AVG_SIZE], gz_buf[MOVING_AVG_SIZE];
static uint16_t buf_index = 0;
static uint8_t buf_full = 0;

static void gyro_mavg_reset(void)
{
    for (int i = 0; i < MOVING_AVG_SIZE; i++) {
        gx_buf[i] = gy_buf[i] = gz_buf[i] = 0.0f;
    }
    buf_index = 0;
    buf_full = 0;
}

static void gyro_mavg_push(float gx, float gy, float gz)
{
    gx_buf[buf_index] = gx;
    gy_buf[buf_index] = gy;
    gz_buf[buf_index] = gz;

    buf_index = (buf_index + 1) % MOVING_AVG_SIZE;
    if (buf_index == 0) buf_full = 1;
}

static void gyro_mavg_get(float *gx, float *gy, float *gz)
{
    float gx_sum = 0.0f, gy_sum = 0.0f, gz_sum = 0.0f;
    uint16_t count = buf_full ? MOVING_AVG_SIZE : buf_index;

    if (count == 0) return;

    for (uint16_t i = 0; i < count; i++) {
        gx_sum += gx_buf[i];
        gy_sum += gy_buf[i];
        gz_sum += gz_buf[i];
    }

    *gx = gx_sum / (float)count;
    *gy = gy_sum / (float)count;
    *gz = gz_sum / (float)count;
}

/* ---- Low-level helpers with retries ---- */
static HAL_StatusTypeDef i2c_write_retry(I2C_HandleTypeDef *hi2c, uint8_t dev, uint8_t reg, uint8_t val)
{
    const uint8_t max_retries = 3;
    for (uint8_t a = 0; a < max_retries; a++) {
        if (HAL_I2C_Mem_Write(hi2c, dev, reg, 1, &val, 1, 100) == HAL_OK) return HAL_OK;
        HAL_Delay(10);
    }
    return HAL_ERROR;
}

static HAL_StatusTypeDef i2c_read_retry(I2C_HandleTypeDef *hi2c, uint8_t dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    const uint8_t max_retries = 3;
    for (uint8_t a = 0; a < max_retries; a++) {
        if (HAL_I2C_Mem_Read(hi2c, dev, reg, 1, buf, len, 100) == HAL_OK) return HAL_OK;
        HAL_Delay(10);
    }
    return HAL_ERROR;
}

static uint8_t autodetect_addr(I2C_HandleTypeDef *hi2c, uint8_t *whoami_out, uint8_t *addr_out)
{
    uint8_t who = 0;

    // Try 0x68
    if (i2c_read_retry(hi2c, MPU_ADDR_0x68, REG_WHO_AM_I, &who, 1) == HAL_OK) {
        if (who == 0x71 || who == 0x70) {
            *whoami_out = who;
            *addr_out = MPU_ADDR_0x68;
            return 1;
        }
    }

    // Try 0x69
    if (i2c_read_retry(hi2c, MPU_ADDR_0x69, REG_WHO_AM_I, &who, 1) == HAL_OK) {
        if (who == 0x71 || who == 0x70) {
            *whoami_out = who;
            *addr_out = MPU_ADDR_0x69;
            return 1;
        }
    }

    return 0;
}

/* ---- Public API ---- */
MPU_Status MPU9250_Init(I2C_HandleTypeDef *hi2c, MPU9250_t *mpu)
{
    if (!hi2c || !mpu) return MPU_ERR_BAD_ARG;
    memset(mpu, 0, sizeof(*mpu));

    // Autodetect address + WHOAMI
    uint8_t who = 0, addr = 0;
    if (!autodetect_addr(hi2c, &who, &addr)) return MPU_ERR_WHOAMI;
    mpu->whoami = who;
    mpu->i2c_addr = addr;

    // Wake up + set clock source to PLL gyro X (0x01)
    if (i2c_write_retry(hi2c, mpu->i2c_addr, REG_PWR_MGMT_1, 0x01) != HAL_OK) return MPU_ERR_I2C;
    HAL_Delay(50);

    // CONFIG: DLPF (e.g. 0x03 -> ~44Hz gyro / 42Hz accel on many MPU variants)
    // You can tune this later: 0x00..0x06
    if (i2c_write_retry(hi2c, mpu->i2c_addr, REG_CONFIG, 0x03) != HAL_OK) return MPU_ERR_I2C;

    // SMPLRT_DIV: sample rate = gyro_rate/(1+div). If gyro_rate=1kHz and div=9 => 100Hz
    if (i2c_write_retry(hi2c, mpu->i2c_addr, REG_SMPLRT_DIV, 9) != HAL_OK) return MPU_ERR_I2C;

    // Gyro range ±250 dps (0x00)
    if (i2c_write_retry(hi2c, mpu->i2c_addr, REG_GYRO_CONFIG, 0x00) != HAL_OK) return MPU_ERR_I2C;

    // Accel range ±2g (0x00)
    if (i2c_write_retry(hi2c, mpu->i2c_addr, REG_ACCEL_CONFIG, 0x00) != HAL_OK) return MPU_ERR_I2C;

    // Reset moving average
    gyro_mavg_reset();

    return MPU_OK;
}

MPU_Status MPU9250_Read_Accel_Gyro(I2C_HandleTypeDef *hi2c, MPU9250_t *mpu)
{
    if (!hi2c || !mpu) return MPU_ERR_BAD_ARG;

    uint8_t data[14];
    if (i2c_read_retry(hi2c, mpu->i2c_addr, REG_ACCEL_XOUT_H, data, 14) != HAL_OK) return MPU_ERR_I2C;

    int16_t axr = (int16_t)((data[0] << 8) | data[1]);
    int16_t ayr = (int16_t)((data[2] << 8) | data[3]);
    int16_t azr = (int16_t)((data[4] << 8) | data[5]);

    int16_t gxr = (int16_t)((data[8]  << 8) | data[9]);
    int16_t gyr = (int16_t)((data[10] << 8) | data[11]);
    int16_t gzr = (int16_t)((data[12] << 8) | data[13]);

    // Scale factors for ±2g and ±250dps
    const float scale_accel = 1.0f / 16384.0f;
    const float scale_gyro  = 1.0f / 131.0f;

    float ax = (float)axr * scale_accel;
    float ay = (float)ayr * scale_accel;
    float az = (float)azr * scale_accel;

    float gx = (float)gxr * scale_gyro;
    float gy = (float)gyr * scale_gyro;
    float gz = (float)gzr * scale_gyro;

    // Apply offsets
    ax -= mpu->Ax_offset;
    ay -= mpu->Ay_offset;
    az -= mpu->Az_offset;

    gx -= mpu->Gx_offset;
    gy -= mpu->Gy_offset;
    gz -= mpu->Gz_offset;

    // Moving average (gyro only)
    gyro_mavg_push(gx, gy, gz);
    gyro_mavg_get(&gx, &gy, &gz);

    // Assign
    mpu->Ax = ax; mpu->Ay = ay; mpu->Az = az;
    mpu->Gx = gx; mpu->Gy = gy; mpu->Gz = gz;

    // accel sanity flag (NOT an error)
    float mag = sqrtf(ax*ax + ay*ay + az*az);
    mpu->accel_sanity_ok = (mag > 0.5f && mag < 1.5f) ? 1 : 0;

    return MPU_OK;
}

MPU_Status MPU9250_Calibrate_Gyro(I2C_HandleTypeDef *hi2c, MPU9250_t *mpu, uint16_t samples)
{
    if (!hi2c || !mpu || samples == 0) return MPU_ERR_BAD_ARG;

    float gx_sum = 0.0f, gy_sum = 0.0f, gz_sum = 0.0f;
    uint8_t data[6];

    for (uint16_t i = 0; i < samples; i++) {
        if (i2c_read_retry(hi2c, mpu->i2c_addr, REG_GYRO_XOUT_H, data, 6) != HAL_OK) return MPU_ERR_I2C;

        int16_t gxr = (int16_t)((data[0] << 8) | data[1]);
        int16_t gyr = (int16_t)((data[2] << 8) | data[3]);
        int16_t gzr = (int16_t)((data[4] << 8) | data[5]);

        gx_sum += (float)gxr / 131.0f;
        gy_sum += (float)gyr / 131.0f;
        gz_sum += (float)gzr / 131.0f;

        HAL_Delay(5);
    }

    mpu->Gx_offset = gx_sum / (float)samples;
    mpu->Gy_offset = gy_sum / (float)samples;
    mpu->Gz_offset = gz_sum / (float)samples;

    gyro_mavg_reset(); // important after changing offsets
    return MPU_OK;
}

MPU_Status MPU9250_Calibrate_Accel(I2C_HandleTypeDef *hi2c, MPU9250_t *mpu, uint16_t samples)
{
    if (!hi2c || !mpu || samples == 0) return MPU_ERR_BAD_ARG;

    float ax_sum = 0.0f, ay_sum = 0.0f, az_sum = 0.0f;
    uint8_t data[6];

    for (uint16_t i = 0; i < samples; i++) {
        if (i2c_read_retry(hi2c, mpu->i2c_addr, REG_ACCEL_XOUT_H, data, 6) != HAL_OK) return MPU_ERR_I2C;

        int16_t axr = (int16_t)((data[0] << 8) | data[1]);
        int16_t ayr = (int16_t)((data[2] << 8) | data[3]);
        int16_t azr = (int16_t)((data[4] << 8) | data[5]);

        ax_sum += (float)axr / 16384.0f;
        ay_sum += (float)ayr / 16384.0f;
        az_sum += (float)azr / 16384.0f;

        HAL_Delay(5);
    }

    mpu->Ax_offset = ax_sum / (float)samples;
    mpu->Ay_offset = ay_sum / (float)samples;

    // crude assumption: sensor is flat, Z should read +1g at rest
    mpu->Az_offset = (az_sum / (float)samples) - 1.0f;

    return MPU_OK;
}
