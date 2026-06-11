/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : STM32 LQI/LQG/FUZZY Multiplexer - V9.4.1 (Con LED de Armado)
 *
 * - ERROR DE SIGNO CORREGIDO: Fuzzy usa retroalimentación negativa (-).
 * - DESBORDAMIENTO CORREGIDO: Límites Fuzzy desempaquetados a /10.0f.
 * - INDICADOR VISUAL: LED en PC13 parpadea 3 veces al armar (No bloqueante).
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
static uint8_t  current_ctrl_mode = 0; 

static uint8_t  cal_busy           = 0;
static uint16_t cal_samples_target = 800;
static uint16_t cal_count          = 0;
static int32_t  gxs=0, gys=0, gzs=0;
static int32_t  axs=0, ays=0, azs=0;
static uint32_t cal_next_ms        = 0;
static const uint32_t CAL_SAMPLE_PERIOD_MS = 5;
static uint32_t next_hb_ms         = 0;

// Variables para el parpadeo del LED
static uint8_t  arm_blink_count = 0;
static uint32_t next_blink_ms   = 0;

#define PKT_TELEM   0xA1
#define PKT_CMD     0xB1
#define PKT_KALMAN  0xB2

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
#define CMD_UPDATE_K      (1U<<8)   
#define CMD_SET_AW        (1U<<9)   
#define CMD_UPDATE_FUZZY  (1U<<10) 

static volatile uint8_t imu_last_ok = 0;
static volatile uint8_t flag_200hz  = 0;

static float ax_f=0, ay_f=0, az_f=0;
static float gx_f=0, gy_f=0, gz_f=0;

static const float alpha_acc  = 0.05f; 
static const float alpha_gyro = 0.4f;  

static float roll_deg  = 0.0f;
static float pitch_deg = 0.0f;

static float L_roll[2][2]  = {{0.1f, 0.0f}, {0.0f, 0.1f}};
static float L_pitch[2][2] = {{0.1f, 0.0f}, {0.0f, 0.1f}};
static float x_hat_roll[2]  = {0.0f, 0.0f}; 
static float x_hat_pitch[2] = {0.0f, 0.0f};

static const float A_r[2][2] = {{0.9976f, 0.02669f}, {-0.03649f, 0.9870f}};
static const float B_r[2]    = {-0.0007416f, 0.0003183f};
static const float A_p[2][2] = {{0.9939f, 0.01319f}, {-0.03461f, 0.9828f}};
static const float B_p[2]    = {-0.0007354f, 0.007451f};

static float last_u_roll = 0.0f;
static float last_u_pitch = 0.0f;

static float target_roll  = 0.0f;
static float target_pitch = 0.0f;
static float int_e_roll  = 0.0f;
static float int_e_pitch = 0.0f;
static float int_limit = 65.0f; 

static float K_roll_ang   = 9.058f;
static float K_roll_rate  = 0.683f;
static float K_roll_int   = 1.287f;     
static float K_pitch_ang  = 14.639f;
static float K_pitch_rate = 12.095f;
static float K_pitch_int  = 1.752f;     

static float fz_e_max   = 30.0f;  
static float fz_r_max   = 200.0f; 
static float fz_out_max = 300.0f; 

static const float U_LIMIT = 300.0f;   

#define MIN_PWM       1000
#define MAX_PWM       1700
#define MAX_THR_USER  1600

static uint16_t base_throttle = 1120;

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
typedef struct {
    uint8_t  type;              
    uint8_t  tx_seq;            
    uint32_t t_ms;              
    int16_t  roll_x10;          
    int16_t  pitch_x10;         
    int16_t  gx_dps10;          
    int16_t  gy_dps10;          
    int16_t  gz_dps10;          
    uint16_t status;            
    uint16_t pwm_m1;            
    uint16_t pwm_m2;            
    uint16_t pwm_m3;            
    uint16_t pwm_m4;            
    uint8_t  cmd_echo;          
    uint8_t  ping_echo;         
} TelemPkt;                      
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
static inline float min_f(float a, float b) { return (a < b) ? a : b; }

static float fuzzify_triangle(float x, float a, float b, float c) {
    if (x <= a || x >= c) return 0.0f;
    if (x == b) return 1.0f;
    if (x < b) return (x - a) / (b - a);
    return (c - x) / (c - b);
}

static float fuzzify_grade_left(float x, float a, float b) {
    if (x <= a) return 1.0f;
    if (x >= b) return 0.0f;
    return (b - x) / (b - a);
}

static float fuzzify_grade_right(float x, float a, float b) {
    if (x <= a) return 0.0f;
    if (x >= b) return 1.0f;
    return (x - a) / (b - a);
}

float compute_fuzzy(float error, float rate) {
    float e_mu[5]; 
    float r_mu[5]; 

    float e_mid = fz_e_max / 2.0f;
    float r_mid = fz_r_max / 2.0f;

    e_mu[0] = fuzzify_grade_left(error, -fz_e_max, -e_mid);        
    e_mu[1] = fuzzify_triangle(error, -fz_e_max, -e_mid, 0.0f);    
    e_mu[2] = fuzzify_triangle(error, -e_mid, 0.0f, e_mid);        
    e_mu[3] = fuzzify_triangle(error, 0.0f, e_mid, fz_e_max);      
    e_mu[4] = fuzzify_grade_right(error, e_mid, fz_e_max);         

    r_mu[0] = fuzzify_grade_left(rate, -fz_r_max, -r_mid);       
    r_mu[1] = fuzzify_triangle(rate, -fz_r_max, -r_mid, 0.0f);   
    r_mu[2] = fuzzify_triangle(rate, -r_mid, 0.0f, r_mid);    
    r_mu[3] = fuzzify_triangle(rate, 0.0f, r_mid, fz_r_max);     
    r_mu[4] = fuzzify_grade_right(rate, r_mid, fz_r_max);        

    const int FAM[5][5] = {
        {0, 0, 1, 1, 2}, 
        {0, 1, 1, 2, 3}, 
        {1, 1, 2, 3, 3}, 
        {1, 2, 3, 3, 4}, 
        {2, 3, 3, 4, 4}  
    };

    const float OUT_CENTERS[5] = {-fz_out_max, -fz_out_max/2.0f, 0.0f, fz_out_max/2.0f, fz_out_max};

    float num = 0.0f;
    float den = 0.0f;

    for(int i=0; i<5; i++) {       
        for(int j=0; j<5; j++) {   
            float weight = min_f(e_mu[i], r_mu[j]);
            if (weight > 0.0f) {
                int out_idx = FAM[i][j];
                num += weight * OUT_CENTERS[out_idx];
                den += weight;
            }
        }
    }

    if (den == 0.0f) return 0.0f;
    float final_out = num / den;

    return clamp_f(final_out, -U_LIMIT, U_LIMIT);
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
    last_u_roll    = 0.0f;
    last_u_pitch   = 0.0f;
    motors_write_all(1000);
}

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

static void control_update(void) {
    if (!imu_enable || cal_busy) return;

    ax_f += alpha_acc  * (mpu.Ay - ax_f); ay_f += alpha_acc  * (-mpu.Ax - ay_f); az_f += alpha_acc  * (mpu.Az - az_f);
    gx_f += alpha_gyro * (mpu.Gy - gx_f); gy_f += alpha_gyro * (-mpu.Gx - gy_f); gz_f += alpha_gyro * (mpu.Gz - gz_f);

    float roll_acc  = atan2f(ay_f, az_f) * 57.29578f;
    float pitch_acc = atan2f(-ax_f, sqrtf(ay_f*ay_f + az_f*az_f)) * 57.29578f;

    float final_roll_est = 0.0f, final_pitch_est = 0.0f;
    float final_gx = gx_f, final_gy = gy_f;

    if (current_ctrl_mode == 0 || current_ctrl_mode == 2) {
        roll_deg  = 0.98f * (roll_deg  + gx_f * dt_control) + 0.02f * roll_acc;
        pitch_deg = 0.98f * (pitch_deg + gy_f * dt_control) + 0.02f * pitch_acc;
        final_roll_est = roll_deg; final_pitch_est = pitch_deg;
        x_hat_roll[0] = roll_deg; x_hat_roll[1] = gx_f;
        x_hat_pitch[0] = pitch_deg; x_hat_pitch[1] = gy_f;
    }
    else if (current_ctrl_mode == 1) {
        float x_pred_roll[2];
        x_pred_roll[0] = A_r[0][0]*x_hat_roll[0] + A_r[0][1]*x_hat_roll[1] + B_r[0]*last_u_roll;
        x_pred_roll[1] = A_r[1][0]*x_hat_roll[0] + A_r[1][1]*x_hat_roll[1] + B_r[1]*last_u_roll;

        float x_pred_pitch[2];
        x_pred_pitch[0] = A_p[0][0]*x_hat_pitch[0] + A_p[0][1]*x_hat_pitch[1] + B_p[0]*last_u_pitch;
        x_pred_pitch[1] = A_p[1][0]*x_hat_pitch[0] + A_p[1][1]*x_hat_pitch[1] + B_p[1]*last_u_pitch;

        float err_y_roll[2]; err_y_roll[0] = roll_acc - x_pred_roll[0]; err_y_roll[1] = gx_f - x_pred_roll[1];
        float err_y_pitch[2]; err_y_pitch[0] = pitch_acc - x_pred_pitch[0]; err_y_pitch[1] = gy_f - x_pred_pitch[1];

        x_hat_roll[0] = x_pred_roll[0] + L_roll[0][0]*err_y_roll[0] + L_roll[0][1]*err_y_roll[1];
        x_hat_roll[1] = x_pred_roll[1] + L_roll[1][0]*err_y_roll[0] + L_roll[1][1]*err_y_roll[1];

        x_hat_pitch[0] = x_pred_pitch[0] + L_pitch[0][0]*err_y_pitch[0] + L_pitch[0][1]*err_y_pitch[1];
        x_hat_pitch[1] = x_pred_pitch[1] + L_pitch[1][0]*err_y_pitch[0] + L_pitch[1][1]*err_y_pitch[1];

        roll_deg = x_hat_roll[0]; pitch_deg = x_hat_pitch[0];
        final_roll_est = x_hat_roll[0]; final_pitch_est = x_hat_pitch[0];
        final_gx = x_hat_roll[1]; final_gy = x_hat_pitch[1];
    }

    if (!armed || !motors_running) {
        int_e_roll = 0.0f; int_e_pitch = 0.0f;
        last_u_roll = 0.0f; last_u_pitch = 0.0f;
        motors_write_all(1000); return;
    }

    float u_roll = 0.0f, u_pitch = 0.0f;
    float e_roll = final_roll_est - target_roll;
    float e_pitch = final_pitch_est - target_pitch;

    if (current_ctrl_mode == 0 || current_ctrl_mode == 1) {
        int_e_roll  += e_roll  * dt_control; int_e_pitch += e_pitch * dt_control;
        int_e_roll  = clamp_f(int_e_roll,  -int_limit, int_limit); int_e_pitch = clamp_f(int_e_pitch, -int_limit, int_limit);

        u_roll  = -(K_roll_ang * e_roll  + K_roll_rate * final_gx + K_roll_int * int_e_roll);
        u_pitch = -(K_pitch_ang * e_pitch + K_pitch_rate * final_gy + K_pitch_int * int_e_pitch);
    }
    else if (current_ctrl_mode == 2) {
        u_roll  = -compute_fuzzy(e_roll, final_gx);
        u_pitch = -compute_fuzzy(e_pitch, final_gy);
    }
    
    last_u_roll = u_roll; last_u_pitch = u_pitch;

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

    if (c->flags & CMD_UPDATE_FUZZY) {
        fz_e_max   = (float)c->k_roll_ang_x1000   / 10.0f;
        fz_r_max   = (float)c->k_roll_rate_x1000  / 10.0f;
        fz_out_max = (float)c->k_roll_int_x1000   / 10.0f;
        uart_print("FUZZY LIMITS UPDATED\r\n");
    }
    else if (c->flags & CMD_UPDATE_K) {
        K_roll_ang   = (float)c->k_roll_ang_x1000   / 1000.0f;
        K_roll_rate  = (float)c->k_roll_rate_x1000  / 1000.0f;
        K_roll_int   = (float)c->k_roll_int_x1000   / 1000.0f;
        K_pitch_ang  = (float)c->k_pitch_ang_x1000  / 1000.0f;
        K_pitch_rate = (float)c->k_pitch_rate_x1000 / 1000.0f;
        K_pitch_int  = (float)c->k_pitch_int_x1000  / 1000.0f;
        uart_print("K UPDATED\r\n");
    }

    if (c->flags & CMD_SET_AW) { int_limit = (float)c->aw_limit; uart_print("AW UPDATED\r\n"); }
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
            
            // Iniciar ráfaga de parpadeo del LED
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
        CmdPkt c;
        c.type = ack_buf[0]; c.cmd_seq = ack_buf[1];
        c.flags = (uint16_t)(ack_buf[2] | (ack_buf[3] << 8)); c.thr_us = (uint16_t)(ack_buf[4] | (ack_buf[5] << 8));
        c.sp_roll_x10 = (int16_t)(ack_buf[6] | (ack_buf[7] << 8)); c.sp_pitch_x10 = (int16_t)(ack_buf[8] | (ack_buf[9] << 8));
        c.k_roll_ang_x1000 = (int16_t)(ack_buf[10] | (ack_buf[11] << 8)); c.k_roll_rate_x1000 = (int16_t)(ack_buf[12] | (ack_buf[13] << 8));
        c.k_roll_int_x1000 = (int16_t)(ack_buf[14] | (ack_buf[15] << 8)); c.k_pitch_ang_x1000 = (int16_t)(ack_buf[16] | (ack_buf[17] << 8));
        c.k_pitch_rate_x1000 = (int16_t)(ack_buf[18] | (ack_buf[19] << 8)); c.k_pitch_int_x1000 = (int16_t)(ack_buf[20] | (ack_buf[21] << 8)); 
        c.aw_limit = (uint16_t)(ack_buf[22] | (ack_buf[23] << 8)); c.imu_enable = ack_buf[24]; c.ping_id = ack_buf[25]; c.ctrl_mode = ack_buf[26];
        handle_cmd(&c);
    }
    else if (acklen == 18 && ack_buf[0] == PKT_KALMAN) {
        L_roll[0][0] = (float)((int16_t)(ack_buf[2] | (ack_buf[3]<<8))) / 1000.0f; L_roll[0][1] = (float)((int16_t)(ack_buf[4] | (ack_buf[5]<<8))) / 1000.0f;
        L_roll[1][0] = (float)((int16_t)(ack_buf[6] | (ack_buf[7]<<8))) / 1000.0f; L_roll[1][1] = (float)((int16_t)(ack_buf[8] | (ack_buf[9]<<8))) / 1000.0f;
        L_pitch[0][0] = (float)((int16_t)(ack_buf[10] | (ack_buf[11]<<8))) / 1000.0f; L_pitch[0][1] = (float)((int16_t)(ack_buf[12] | (ack_buf[13]<<8))) / 1000.0f;
        L_pitch[1][0] = (float)((int16_t)(ack_buf[14] | (ack_buf[15]<<8))) / 1000.0f; L_pitch[1][1] = (float)((int16_t)(ack_buf[16] | (ack_buf[17]<<8))) / 1000.0f;
    }
}

int main(void) {
    HAL_Init(); SystemClock_Config();
    MX_GPIO_Init(); MX_I2C1_Init(); MX_SPI1_Init(); MX_TIM1_Init(); MX_TIM2_Init(); MX_USART2_UART_Init();
    HAL_Delay(200); uart_print("BOOT V9.4.1 - CON LED ARM\r\n");
    motors_start(); disarm_now(0); last_cmd_ms = HAL_GetTick();
    if (MPU9250_Init(&hi2c1, &mpu) != MPU_OK) { uart_print("MPU FAIL\r\n"); while (1) HAL_Delay(1000); }
    nrf.ce_port = GPIOB; nrf.ce_pin = GPIO_PIN_0; nrf.csn_port = GPIOB; nrf.csn_pin = GPIO_PIN_1;
    if (NRF24_Init(&hspi1, &nrf) != NRF_OK) { uart_print("NRF FAIL\r\n"); while (1) HAL_Delay(1000); }
    (void)NRF24_Configure_PTX(&hspi1, &nrf, RF_ADDR, RF_CH, NRF_DATARATE_250K, NRF_PA_LOW);
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0); HAL_NVIC_EnableIRQ(TIM2_IRQn); HAL_TIM_Base_Start_IT(&htim2);
    next_hb_ms = HAL_GetTick(); uart_print("READY\r\n");

    while (1) {
        uint32_t now = HAL_GetTick();
        if (flag_200hz) {
            flag_200hz = 0;
            if (imu_enable && !cal_busy) {
                imu_last_ok = (MPU9250_Read_Accel_Gyro(&hi2c1, &mpu) == MPU_OK) ? 1 : 0;
                if (imu_last_ok) control_update();
            }
        }
        cal_step(now);

        // --- GESTOR DE PARPADEO DEL LED PC13 ---
        if (arm_blink_count > 0 && now >= next_blink_ms) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            next_blink_ms = now + 100;
            arm_blink_count--;
            
            if (arm_blink_count == 0) {
                // Forzar apagado al terminar la ráfaga (PC13 suele ser activo bajo, SET = OFF)
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
    
    // Habilitar los relojes para los puertos A, B y C (PC13 para el LED)
    __HAL_RCC_GPIOA_CLK_ENABLE(); 
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE(); 

    // Inicialización del LED en PC13 (Apagado por defecto - Activo bajo)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); 
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Inicialización de NRF (Mantenido intacto)
    HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET); 
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = CE_Pin | CSN_Pin; 
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL; 
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
void Error_Handler(void) { __disable_irq(); while (1) {} }
