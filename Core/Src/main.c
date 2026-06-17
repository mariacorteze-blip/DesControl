/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : STM32 EXCLUSIVE MPC RAM STREAM EDITION - V11.4 (STABLE CORE)
 *
 * - RESTAURADO: Superloop y driver MPU9250_Read_Accel_Gyro idéntico a V9.5.
 * - RESTAURADO: Filtro complementario exacto de la versión estable.
 * - CONTROL: Solo MPC (Optimizador FGM) inyectado en el lazo de control.
 ******************************************************************************
 */
/* USER CODE END Header */

#include "main.h"
#include "mpu9250.h"
#include "nrf24.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

I2C_HandleTypeDef  hi2c1;
SPI_HandleTypeDef  hspi1;
TIM_HandleTypeDef  htim1;
TIM_HandleTypeDef  htim2;
UART_HandleTypeDef huart2;

static MPU9250_t mpu;
static NRF24_t   nrf;

static const uint8_t RF_ADDR[5] = {'D','R','O','N','E'};
static const uint8_t RF_CH      = 110;

static const uint32_t HB_PERIOD_MS = 20;     
static const uint32_t FAILSAFE_MS  = 3000;   
static const float    dt_control   = 0.005f; 

static uint8_t  imu_enable      = 1;
static uint8_t  armed           = 0;
static uint8_t  motors_running  = 0;   
static uint8_t  failsafe_active = 0;
static uint32_t last_cmd_ms     = 0;
static uint8_t  tx_seq          = 0;
static uint8_t  cmd_echo        = 0;
static uint8_t  ping_echo       = 0;
static uint8_t  current_ctrl_mode = 4; // 4 = MPC

static uint8_t  cal_busy           = 0;
static uint16_t cal_samples_target = 800;
static uint16_t cal_count          = 0;
static int32_t  gxs=0, gys=0, gzs=0;
static int32_t  axs=0, ays=0, azs=0;
static uint32_t cal_next_ms        = 0;
static const uint32_t CAL_SAMPLE_PERIOD_MS = 5;
static uint32_t next_hb_ms         = 0;

static uint8_t  arm_blink_count = 0;
static uint32_t next_blink_ms   = 0;

#define PKT_TELEM   0xA1
#define PKT_CMD     0xB1

#define ST_MPU_OK     (1U<<0)
#define ST_ACCEL_OK   (1U<<1)
#define ST_ARMED      (1U<<2)
#define ST_CAL_BUSY   (1U<<3)
#define ST_IMU_EN     (1U<<4)
#define ST_FAILSAFE   (1U<<5)
#define ST_MOTORS_RUN (1U<<6)

#define CMD_ARM           (1U<<0)
#define CMD_DISARM        (1U<<1)
#define CMD_ESTOP         (1U<<2)
#define CMD_SET_THR       (1U<<3)
#define CMD_IMU_EN        (1U<<4)
#define CMD_CAL_ALL       (1U<<5)
#define CMD_PING          (1U<<6)
#define CMD_START_MOTORS  (1U<<7)
#define CMD_STREAM_MATRIX (1U<<12) 
#define CMD_SET_U_MAX     (1U<<13) 

static volatile uint8_t imu_last_ok = 0;
static volatile uint8_t flag_200hz  = 0;

static float ax_f=0, ay_f=0, az_f=0;
static float gx_f=0, gy_f=0, gz_f=0;
static const float alpha_acc  = 0.05f; 
static const float alpha_gyro = 0.4f;  

static float roll_deg  = 0.0f;
static float pitch_deg = 0.0f;

static float target_roll  = 0.0f;
static float target_pitch = 0.0f;
static float int_e_roll  = 0.0f;
static float int_e_pitch = 0.0f;
static float int_limit = 65.0f; 

static float U_LIMIT = 300.0f;   
#define MIN_PWM       1000
#define MAX_PWM       1700
#define MAX_THR_USER  1600
static uint16_t base_throttle = 1120;

// ====================================================================
// MATRICES DEL MPC
// ====================================================================
#define MPC_N 10      
#define MPC_ITER 15   

volatile float L_step_roll = 0.038108f;
volatile float H_roll[MPC_N][MPC_N] = {
    {2.261298f, 1.259929f, 0.428456f, -0.119045f, -0.426543f, -0.551390f, -0.548231f, -0.468205f, -0.350328f, -0.219808f},
    {1.259929f, 2.261298f, 1.259929f, 0.428456f, -0.119045f, -0.426543f, -0.551390f, -0.548231f, -0.468205f, -0.350328f},
    {0.428456f, 1.259929f, 2.261298f, 1.259929f, 0.428456f, -0.119045f, -0.426543f, -0.551390f, -0.468205f, -0.350328f},
    {-0.119045f, 0.428456f, 1.259929f, 2.261298f, 1.259929f, 0.428456f, -0.119045f, -0.426543f, -0.551390f, -0.548231f},
    {-0.426543f, -0.119045f, 0.428456f, 1.259929f, 2.261298f, 1.259929f, 0.428456f, -0.119045f, -0.426543f, -0.551390f},
    {-0.551390f, -0.426543f, -0.119045f, 0.428456f, 1.259929f, 2.261298f, 1.259929f, 0.428456f, -0.119045f, -0.426543f},
    {-0.548231f, -0.551390f, -0.426543f, -0.119045f, 0.428456f, 1.259929f, 2.261298f, 1.259929f, 0.428456f, -0.119045f},
    {-0.468205f, -0.548231f, -0.551390f, -0.426543f, -0.119045f, 0.428456f, 1.259929f, 2.261298f, 1.259929f, 0.428456f},
    {-0.350328f, -0.468205f, -0.548231f, -0.551390f, -0.426543f, -0.119045f, 0.428456f, 1.259929f, 2.261298f, 1.259929f},
    {-0.219808f, -0.350328f, -0.468205f, -0.548231f, -0.551390f, -0.426543f, -0.119045f, 0.428456f, 1.259929f, 2.261298f},
};
volatile float F_roll[MPC_N][3] = {
    {-0.407421f, -0.010170f, 0.005000f}, {-0.804245f, -0.019488f, 0.010000f}, {-1.185217f, -0.027962f, 0.015000f},
    {-1.545367f, -0.035593f, 0.020000f}, {-1.879948f, -0.042385f, 0.025000f}, {-2.184496f, -0.048340f, 0.030000f},
    {-2.454865f, -0.053457f, 0.035000f}, {-2.687311f, -0.057739f, 0.040000f}, {-2.878484f, -0.061183f, 0.045000f},
    {-3.025424f, -0.063788f, 0.050000f},
};

volatile float L_step_pitch = 0.019478f;
volatile float H_pitch[MPC_N][MPC_N] = {
    {1.811802f, 1.185012f, 0.350616f, -0.198599f, -0.523049f, -0.686520f, -0.738096f, -0.716301f, -0.648177f, -0.553258f},
    {1.185012f, 1.811802f, 1.185012f, 0.350616f, -0.198599f, -0.523049f, -0.686520f, -0.738096f, -0.716301f, -0.648177f},
    {0.350616f, 1.185012f, 1.811802f, 1.185012f, 0.350616f, -0.198599f, -0.523049f, -0.686520f, -0.738096f, -0.716301f},
    {-0.198599f, 0.350616f, 1.185012f, 1.811802f, 1.185012f, 0.350616f, -0.198599f, -0.523049f, -0.686520f, -0.738096f},
    {-0.523049f, -0.198599f, 0.350616f, 1.185012f, 1.811802f, 1.185012f, 0.350616f, -0.198599f, -0.523049f, -0.686520f},
    {-0.686520f, -0.523049f, -0.198599f, 0.350616f, 1.185012f, 1.811802f, 1.185012f, 0.350616f, -0.198599f, -0.523049f},
    {-0.738096f, -0.686520f, -0.523049f, -0.198599f, 0.350616f, 1.185012f, 1.811802f, 1.185012f, 0.350616f, -0.198599f},
    {-0.716301f, -0.738096f, -0.686520f, -0.523049f, -0.198599f, 0.350616f, 1.185012f, 1.811802f, 1.185012f, 0.350616f},
    {-0.648177f, -0.716301f, -0.738096f, -0.686520f, -0.523049f, -0.198599f, 0.350616f, 1.185012f, 1.811802f, 1.185012f},
    {-0.553258f, -0.648177f, -0.716301f, -0.738096f, -0.686520f, -0.523049f, -0.198599f, 0.350616f, 1.185012f, 1.811802f},
};
volatile float F_pitch[MPC_N][3] = {
    {-0.767566f, -0.010255f, 0.010000f}, {-1.520448f, -0.019747f, 0.020000f}, {-2.245875f, -0.028475f, 0.030000f},
    {-2.931757f, -0.036441f, 0.040000f}, {-3.566861f, -0.043644f, 0.050000f}, {-4.141014f, -0.050085f, 0.060000f},
    {-4.645366f, -0.055767f, 0.070000f}, {-5.072535f, -0.060690f, 0.080000f}, {-5.416629f, -0.064858f, 0.090000f},
    {-5.673255f, -0.068273f, 0.100000f},
};

#pragma pack(push,1)
typedef struct {
    uint8_t  type;              
    uint8_t  cmd_seq;           
    uint16_t flags;             
    uint16_t thr_us;            
    int16_t  sp_roll_x10;       
    int16_t  sp_pitch_x10;      
    int16_t  k_roll_ang_x1000;  
    int16_t  k_roll_rate_x1000; 
    int16_t  k_roll_int_x1000;  
    int16_t  k_pitch_ang_x1000; 
    int16_t  k_pitch_rate_x1000;
    int16_t  k_pitch_int_x1000; 
    uint16_t aw_limit;          
    uint8_t  imu_enable;        
    uint8_t  ping_id;           
    uint8_t  ctrl_mode;         
} CmdPkt;                        
#pragma pack(pop)

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
void Error_Handler(void);

static inline uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi) {
    if (v < lo) return lo; if (v > hi) return hi; return v;
}
static inline int16_t clamp_i16(int32_t v) {
    if (v >  32767) return  32767; if (v < -32768) return -32768; return (int16_t)v;
}
static inline float clamp_f(float v, float lo, float hi) {
    if (v < lo) return lo; if (v > hi) return hi; return v;
}
static void uart_print(const char *s) {
    HAL_UART_Transmit(&huart2, (uint8_t*)s, (uint16_t)strlen(s), 200);
}

static void solve_mpc_fgm(volatile float H[MPC_N][MPC_N], volatile float F[MPC_N][3], float L_step, float x0[3], float u_max, float *u_out) {
    float z[MPC_N] = {0}; float y[MPC_N] = {0}; float z_new[MPC_N] = {0}; float grad[MPC_N];
    float Fx0[MPC_N] = {0};
    
    for(int i = 0; i < MPC_N; i++) Fx0[i] = F[i][0] * x0[0] + F[i][1] * x0[1] + F[i][2] * x0[2];
    for(int iter = 0; iter < MPC_ITER; iter++) {
        for(int i = 0; i < MPC_N; i++) {
            float Hy = 0;
            for(int j = 0; j < MPC_N; j++) Hy += H[i][j] * y[j];
            grad[i] = Hy + Fx0[i];
        }
        for(int i = 0; i < MPC_N; i++) {
            float step = y[i] - L_step * grad[i];
            if (step > u_max) step = u_max;
            else if (step < -u_max) step = -u_max;
            z_new[i] = step;
        }
        float beta = (float)iter / (iter + 3.0f);
        for(int i = 0; i < MPC_N; i++) {
            y[i] = z_new[i] + beta * (z_new[i] - z[i]); z[i] = z_new[i];
        }
    }
    *u_out = z[0];
}

static void write_matrix_element(uint8_t axis, uint16_t flat_idx, float value) {
    if (flat_idx > 130) return;
    if (axis == 0) { 
        if (flat_idx < 100)        ((float*)H_roll)[flat_idx] = value;
        else if (flat_idx < 130)   ((float*)F_roll)[flat_idx - 100] = value;
        else                       L_step_roll = value;
    } else { 
        if (flat_idx < 100)        ((float*)H_pitch)[flat_idx] = value;
        else if (flat_idx < 130)   ((float*)F_pitch)[flat_idx - 100] = value;
        else                       L_step_pitch = value;
    }
}

static void motors_start(void) {
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    __HAL_TIM_MOE_ENABLE(&htim1);
}

static void motors_write_all(uint16_t us) {
    us = clamp_u16(us, 1000, 2000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, us);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, us);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, us);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, us);
}

static void disarm_now(uint8_t set_failsafe_flag) {
    if (set_failsafe_flag) failsafe_active = 1;
    armed          = 0;
    motors_running = 0;
    int_e_roll     = 0.0f; 
    int_e_pitch    = 0.0f;
    motors_write_all(1000);
}

// ------------------------------------------------------------------------
// LAS LECTURAS ORIGINALES PARA CALIBRACIÓN (Intactas de V9.5)
// ------------------------------------------------------------------------
static bool mpu_read_raw_gyro(int16_t *gx, int16_t *gy, int16_t *gz) {
    uint8_t d[6];
    if (HAL_I2C_Mem_Read(&hi2c1, mpu.i2c_addr, 0x43, 1, d, 6, 100) != HAL_OK) return false;
    *gx = (int16_t)((d[0]<<8)|d[1]); *gy = (int16_t)((d[2]<<8)|d[3]); *gz = (int16_t)((d[4]<<8)|d[5]);
    return true;
}

static bool mpu_read_raw_accel(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t d[6];
    if (HAL_I2C_Mem_Read(&hi2c1, mpu.i2c_addr, 0x3B, 1, d, 6, 100) != HAL_OK) return false;
    *ax = (int16_t)((d[0]<<8)|d[1]); *ay = (int16_t)((d[2]<<8)|d[3]); *az = (int16_t)((d[4]<<8)|d[5]);
    return true;
}

static void cal_start(uint16_t samples) {
    cal_busy = 1; cal_samples_target = samples; cal_count = 0;
    gxs=gys=gzs=0; axs=ays=azs=0;
    cal_next_ms = HAL_GetTick();
    disarm_now(0);
}

static void cal_step(uint32_t now_ms) {
    if (!cal_busy) return;
    if ((now_ms - cal_next_ms) < CAL_SAMPLE_PERIOD_MS) return;
    cal_next_ms = now_ms;

    int16_t gx,gy,gz, ax,ay,az;
    if (!mpu_read_raw_gyro(&gx,&gy,&gz))   return;
    if (!mpu_read_raw_accel(&ax,&ay,&az))  return;

    gxs += gx; gys += gy; gzs += gz;
    axs += ax; ays += ay; azs += az;
    cal_count++;

    if (cal_count >= cal_samples_target) {
        mpu.Gx_offset = ((float)gxs / cal_count) / 131.0f; mpu.Gy_offset = ((float)gys / cal_count) / 131.0f; mpu.Gz_offset = ((float)gzs / cal_count) / 131.0f;
        mpu.Ax_offset = ((float)axs / cal_count) / 16384.0f; mpu.Ay_offset = ((float)ays / cal_count) / 16384.0f; mpu.Az_offset = (((float)azs / cal_count) / 16384.0f) - 1.0f;
        cal_busy = 0; uart_print("CAL DONE\r\n");
    }
}

// ------------------------------------------------------------------------
// LAZO DE CONTROL (Estructura Intacta de V9.5)
// ------------------------------------------------------------------------
static void control_update(void) {
    if (!imu_enable || cal_busy) return;

    ax_f += alpha_acc  * (mpu.Ay - ax_f); ay_f += alpha_acc  * (-mpu.Ax - ay_f); az_f += alpha_acc  * (mpu.Az - az_f);
    gx_f += alpha_gyro * (mpu.Gy - gx_f); gy_f += alpha_gyro * (-mpu.Gx - gy_f); gz_f += alpha_gyro * (mpu.Gz - gz_f);

    float roll_acc  = atan2f(ay_f, az_f) * 57.29578f;
    float pitch_acc = atan2f(-ax_f, sqrtf(ay_f*ay_f + az_f*az_f)) * 57.29578f;

    roll_deg  = 0.98f * (roll_deg  + gx_f * dt_control) + 0.02f * roll_acc;
    pitch_deg = 0.98f * (pitch_deg + gy_f * dt_control) + 0.02f * pitch_acc;

    if (!armed || !motors_running) {
        int_e_roll = 0.0f; int_e_pitch = 0.0f;
        motors_write_all(1000); return;
    }

    float e_roll  = roll_deg - target_roll;
    float e_pitch = pitch_deg - target_pitch;

    int_e_roll  += e_roll  * dt_control; int_e_pitch += e_pitch * dt_control;
    int_e_roll  = clamp_f(int_e_roll,  -int_limit, int_limit); int_e_pitch = clamp_f(int_e_pitch, -int_limit, int_limit);

    // MÁQUINA DE ESTADOS - AHORA SOLO EXISTE EL MPC
    float u_roll = 0.0f; float u_pitch = 0.0f;
    
    float x_r[3] = {e_roll, gx_f, int_e_roll};
    float x_p[3] = {e_pitch, gy_f, int_e_pitch};

    solve_mpc_fgm(H_roll, F_roll, L_step_roll, x_r, U_LIMIT, &u_roll);
    solve_mpc_fgm(H_pitch, F_pitch, L_step_pitch, x_p, U_LIMIT, &u_pitch);

    u_roll = u_roll; 
		u_pitch = -u_pitch;
    
    u_roll = clamp_f(u_roll, -U_LIMIT, U_LIMIT); u_pitch = clamp_f(u_pitch, -U_LIMIT, U_LIMIT);

    int32_t thr = (int32_t)base_throttle;
    int32_t m1 = thr + (int32_t)u_roll - (int32_t)u_pitch; int32_t m2 = thr - (int32_t)u_roll - (int32_t)u_pitch;
    int32_t m3 = thr + (int32_t)u_roll + (int32_t)u_pitch; int32_t m4 = thr - (int32_t)u_roll + (int32_t)u_pitch;

    uint16_t pwm_m1 = (uint16_t)clamp_u16((uint16_t)(m1 < 0 ? 0 : m1), MIN_PWM, MAX_PWM);
    uint16_t pwm_m2 = (uint16_t)clamp_u16((uint16_t)(m2 < 0 ? 0 : m2), MIN_PWM, MAX_PWM);
    uint16_t pwm_m3 = (uint16_t)clamp_u16((uint16_t)(m3 < 0 ? 0 : m3), MIN_PWM, MAX_PWM);
    uint16_t pwm_m4 = (uint16_t)clamp_u16((uint16_t)(m4 < 0 ? 0 : m4), MIN_PWM, MAX_PWM);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_m1); __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_m2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_m3); __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm_m4);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) { if (htim->Instance == TIM2) flag_200hz = 1; }

static void handle_cmd(const CmdPkt *c) {
    if (!c || c->type != PKT_CMD) return;

    last_cmd_ms       = HAL_GetTick();
    cmd_echo          = c->cmd_seq;
    failsafe_active   = 0;   
    current_ctrl_mode = c->ctrl_mode; 

    target_roll  = (float)c->sp_roll_x10  / 10.0f;
    target_pitch = (float)c->sp_pitch_x10 / 10.0f;

    if (c->flags & CMD_SET_U_MAX) { U_LIMIT = (float)c->aw_limit; uart_print("U_MAX UPDATED\r\n"); }
    if (c->flags & CMD_PING) ping_echo  = c->ping_id;
    if (c->flags & CMD_IMU_EN) imu_enable = c->imu_enable ? 1 : 0;
    if (c->flags & CMD_SET_THR) base_throttle = clamp_u16(c->thr_us, MIN_PWM, MAX_THR_USER);

    if (c->flags & CMD_ESTOP) { disarm_now(0); uart_print("ESTOP\r\n"); return; }
    if (c->flags & CMD_CAL_ALL) { cal_start(800); uart_print("CAL START\r\n"); return; }
    if (c->flags & CMD_DISARM) { disarm_now(0); uart_print("DISARM\r\n"); return; }

    if (c->flags & CMD_ARM) {
        if (!failsafe_active) {
            armed = 1; motors_running = 0; int_e_roll = 0.0f; int_e_pitch = 0.0f;
            motors_write_all(1000); uart_print("ARMED\r\n");
            
            arm_blink_count = 6; 
            next_blink_ms = HAL_GetTick();
        }
    }

    if ((c->flags & CMD_START_MOTORS) && armed) { motors_running = 1; uart_print("MOTORS RUNNING\r\n"); }
}

static inline void buf_u8 (uint8_t *b, uint32_t *o, uint8_t  v) { b[(*o)++] = v; }
static inline void buf_u16(uint8_t *b, uint32_t *o, uint16_t v) { b[(*o)++] = (uint8_t)(v & 0xFF); b[(*o)++] = (uint8_t)((v >> 8) & 0xFF); }
static inline void buf_u32(uint8_t *b, uint32_t *o, uint32_t v) {
    b[(*o)++] = (uint8_t)(v & 0xFF); b[(*o)++] = (uint8_t)((v >> 8) & 0xFF);
    b[(*o)++] = (uint8_t)((v >> 16) & 0xFF); b[(*o)++] = (uint8_t)((v >> 24) & 0xFF);
}

static void build_send_telem(uint32_t now_ms) {
    uint8_t buf[28]; uint32_t o = 0;
    uint16_t st = 0;
    if (imu_enable && !cal_busy && imu_last_ok) { st |= ST_MPU_OK; if (mpu.accel_sanity_ok) st |= ST_ACCEL_OK; }
    if (armed) st |= ST_ARMED; if (motors_running) st |= ST_MOTORS_RUN;
    if (cal_busy) st |= ST_CAL_BUSY; if (imu_enable) st |= ST_IMU_EN; if (failsafe_active) st |= ST_FAILSAFE;

    buf_u8 (buf, &o, PKT_TELEM); buf_u8 (buf, &o, tx_seq++); buf_u32(buf, &o, now_ms);                                        
    buf_u16(buf, &o, (uint16_t)clamp_i16((int32_t)(roll_deg  * 10.0f))); buf_u16(buf, &o, (uint16_t)clamp_i16((int32_t)(pitch_deg * 10.0f))); 
    buf_u16(buf, &o, (uint16_t)clamp_i16((int32_t)(gx_f * 10.0f))); buf_u16(buf, &o, (uint16_t)clamp_i16((int32_t)(gy_f * 10.0f)));      
    buf_u16(buf, &o, (uint16_t)clamp_i16((int32_t)(gz_f * 10.0f))); buf_u16(buf, &o, st);                                              
    buf_u16(buf, &o, (uint16_t)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1)); buf_u16(buf, &o, (uint16_t)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2)); 
    buf_u16(buf, &o, (uint16_t)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3)); buf_u16(buf, &o, (uint16_t)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_4)); 
    buf_u8 (buf, &o, cmd_echo); buf_u8 (buf, &o, ping_echo);                                        

    uint8_t ack_buf[32]; uint8_t acklen = 0;
    (void)NRF24_WriteAndReadAck(&hspi1, &nrf, buf, 28, ack_buf, 32, &acklen, 15);

    if (acklen == 27 && ack_buf[0] == PKT_CMD) {
        uint16_t flags = (uint16_t)(ack_buf[2] | (ack_buf[3] << 8));
        
        if (flags & CMD_STREAM_MATRIX) {
            uint16_t thr_us = (uint16_t)(ack_buf[4] | (ack_buf[5] << 8));
            uint8_t axis = thr_us & 0xFF;
            uint8_t chunk_seq = (thr_us >> 8) & 0xFF;
            
            float chunk_data[4];
            memcpy(chunk_data, &ack_buf[6], 16); 
            
            uint16_t start_idx = chunk_seq * 4;
            for(int i = 0; i < 4; i++) {
                write_matrix_element(axis, start_idx + i, chunk_data[i]);
            }
            last_cmd_ms = HAL_GetTick(); cmd_echo = ack_buf[1]; failsafe_active = 0;
        } else {
            CmdPkt c;
            c.type = ack_buf[0]; c.cmd_seq = ack_buf[1]; c.flags = flags;
            c.thr_us = (uint16_t)(ack_buf[4] | (ack_buf[5] << 8));
            c.sp_roll_x10 = (int16_t)(ack_buf[6] | (ack_buf[7] << 8)); c.sp_pitch_x10 = (int16_t)(ack_buf[8] | (ack_buf[9] << 8));
            c.aw_limit = (uint16_t)(ack_buf[22] | (ack_buf[23] << 8)); c.imu_enable = ack_buf[24]; c.ping_id = ack_buf[25]; c.ctrl_mode = ack_buf[26];
            handle_cmd(&c);
        }
    }
}

int main(void) {
    HAL_Init(); SystemClock_Config();
    MX_GPIO_Init(); MX_I2C1_Init(); MX_SPI1_Init(); MX_TIM1_Init(); MX_TIM2_Init(); MX_USART2_UART_Init();
    HAL_Delay(200); uart_print("BOOT V11.4 - CORE RESTORED\r\n");
    motors_start(); disarm_now(0); last_cmd_ms = HAL_GetTick();
    
    if (MPU9250_Init(&hi2c1, &mpu) != MPU_OK) { uart_print("MPU FAIL\r\n"); while (1) HAL_Delay(1000); }
    nrf.ce_port = GPIOB; nrf.ce_pin = GPIO_PIN_0; nrf.csn_port = GPIOB; nrf.csn_pin = GPIO_PIN_1;
    if (NRF24_Init(&hspi1, &nrf) != NRF_OK) { uart_print("NRF FAIL\r\n"); while (1) HAL_Delay(1000); }
    (void)NRF24_Configure_PTX(&hspi1, &nrf, RF_ADDR, RF_CH, NRF_DATARATE_250K, NRF_PA_LOW);
    
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0); HAL_NVIC_EnableIRQ(TIM2_IRQn); HAL_TIM_Base_Start_IT(&htim2);
    next_hb_ms = HAL_GetTick(); uart_print("READY\r\n");

    while (1) {
        uint32_t now = HAL_GetTick();
        
        // ====================================================================
        // RETORNO A LA LECTURA EXACTA DE V9.5 (Librería Oficial)
        // ====================================================================
        if (flag_200hz) {
            flag_200hz = 0;
            if (imu_enable && !cal_busy) {
                imu_last_ok = (MPU9250_Read_Accel_Gyro(&hi2c1, &mpu) == MPU_OK) ? 1 : 0;
                if (imu_last_ok) control_update();
            }
        }
        
        cal_step(now);

        if (arm_blink_count > 0 && now >= next_blink_ms) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            next_blink_ms = now + 100;
            arm_blink_count--;
            
            if (arm_blink_count == 0) {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); 
            }
        }

        if (cmd_echo != 0 && (now - last_cmd_ms) > FAILSAFE_MS) { if (!failsafe_active) disarm_now(1); }
        if ((int32_t)(now - next_hb_ms) >= 0) { next_hb_ms += HB_PERIOD_MS; build_send_telem(now); }
    }
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0}; RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    __HAL_RCC_PWR_CLK_ENABLE(); __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI; RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI; RCC_OscInitStruct.PLL.PLLM = 8; RCC_OscInitStruct.PLL.PLLN = 84;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2; RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}
static void MX_I2C1_Init(void) {
    hi2c1.Instance = I2C1; hi2c1.Init.ClockSpeed = 400000; hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0; hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT; hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0; hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE; hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}
static void MX_SPI1_Init(void) {
    hspi1.Instance = SPI1; hspi1.Init.Mode = SPI_MODE_MASTER; hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT; hspi1.Init.CLKPolarity = SPI_POLARITY_LOW; hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT; hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64; hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE; hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE; hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}
static void MX_TIM1_Init(void) {
    TIM_MasterConfigTypeDef sMasterConfig = {0}; TIM_OC_InitTypeDef sConfigOC = {0}; TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
    htim1.Instance = TIM1; htim1.Init.Prescaler = 83; htim1.Init.CounterMode = TIM_COUNTERMODE_UP; htim1.Init.Period = 3999;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; htim1.Init.RepetitionCounter = 0; htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET; sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) Error_Handler();
    sConfigOC.OCMode = TIM_OCMODE_PWM1; sConfigOC.Pulse = 1000; sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH; sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET; sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) Error_Handler();
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE; sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF; sBreakDeadTimeConfig.DeadTime = 0; sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH; sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) Error_Handler();
    HAL_TIM_MspPostInit(&htim1);
}
static void MX_TIM2_Init(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0}; TIM_MasterConfigTypeDef sMasterConfig = {0};
    __HAL_RCC_TIM2_CLK_ENABLE();
    htim2.Instance = TIM2; htim2.Init.Prescaler = 83; htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 4999; htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) Error_Handler();
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET; sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();
}
static void MX_USART2_UART_Init(void) {
    huart2.Instance = USART2; huart2.Init.BaudRate = 115200; huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1; huart2.Init.Parity = UART_PARITY_NONE; huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE; huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOA_CLK_ENABLE(); 
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE(); 

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); 
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET); 
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = CE_Pin | CSN_Pin; 
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL; 
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
void Error_Handler(void) { __disable_irq(); while (1) {} }
