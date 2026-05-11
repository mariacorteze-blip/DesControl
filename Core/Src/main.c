/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Control LQI (LQR + Integral) + Timer Flag + LPF Seguro
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

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim2;

static MPU9250_t mpu;
static NRF24_t nrf;

static const uint8_t RF_ADDR[5] = {'D','R','O','N','E'};
static const uint8_t RF_CH = 110;

static const uint32_t HB_PERIOD_MS  = 50;  

static uint8_t  imu_enable = 1;
static uint8_t  armed = 0;
static uint16_t thr_us = 1000;
static uint32_t last_cmd_ms = 0;
static const uint32_t FAILSAFE_MS = 15000;  

static uint8_t tx_seq = 0;
static uint8_t cmd_echo = 0;
static uint8_t ping_echo = 0;

static uint8_t  cal_busy = 0;
static uint16_t cal_samples_target = 800;
static uint16_t cal_count = 0;
static int32_t gxs=0,gys=0,gzs=0;
static int32_t axs=0,ays=0,azs=0;
static uint32_t cal_next_ms = 0;
static const uint32_t CAL_SAMPLE_PERIOD_MS = 5;

static uint32_t next_hb_ms  = 0;

#define PKT_TELEM   0xA1
#define PKT_CMD     0xB1

#define ST_MPU_OK     (1U<<0)
#define ST_ACCEL_OK   (1U<<1)
#define ST_ARMED      (1U<<2)
#define ST_CAL_BUSY   (1U<<3)
#define ST_IMU_EN     (1U<<4)
#define ST_FAILSAFE   (1U<<5)

#define CMD_ARM        (1U<<0)
#define CMD_DISARM     (1U<<1)
#define CMD_ESTOP      (1U<<2)
#define CMD_SET_THR    (1U<<3)
#define CMD_SET_MOTOR  (1U<<4)
#define CMD_IMU_EN     (1U<<5)
#define CMD_CAL_ALL    (1U<<6)
#define CMD_PING       (1U<<7)
#define CMD_SET_PID    (1U<<8)

#define M1_BIT (1U<<0)
#define M2_BIT (1U<<1)
#define M3_BIT (1U<<2)
#define M4_BIT (1U<<3)

/* ===== VARIABLES DEL CONTROL LQI ===== */
static float roll_angle = 0.0f;     
static float roll_sp = 0.0f;        

static float q_ang = 25.0f;      
static float q_rate = 2.0f;      
static float r_ctrl = 1.0f;      
static float b_param = 1.0f;     

static float K1 = 0.0f;          
static float K2 = 0.0f;          
static int32_t trim_m1_m4 = 0; 

// --- NUEVAS VARIABLES PARA EL TÉRMINO INTEGRAL ---
static float Ki_ang = 0.5f;      // Ganancia Integral
static float Iterm_ang = 0.0f;   // Acumulador de error
static const float I_LIM = 50.0f; // Anti-windup límite 

static const float Ts = 0.005f;     
static const float U_LIM = 250.0f;   

static uint32_t manual_override_until_ms = 0;
static uint8_t  manual_mask = 0;
static uint16_t manual_us   = 1000;

static volatile uint8_t imu_last_ok = 0;
static volatile uint8_t flag_200hz = 0; 

/* ===== VARIABLES DEL FILTRO PASO BAJO ===== */
static float ax_f = 0.0f, ay_f = 0.0f, az_f = 0.0f;
static float gx_f = 0.0f, gy_f = 0.0f, gz_f = 0.0f;

static float alpha_acc  = 0.05f;  
static float alpha_gyro = 0.15f;  

#pragma pack(push,1)
typedef struct {
  uint8_t  type;        
  uint8_t  cmd_seq;     
  uint16_t flags;       
  uint16_t thr_us;      
  uint8_t  motor_mask;  
  uint16_t motor_us;    
  uint8_t  imu_enable;  
  uint8_t  ping_id;     
  uint8_t  pad;         
} CmdPkt;

typedef struct {
  uint8_t  type;        
  uint8_t  tx_seq;      
  uint32_t t_ms;
  int16_t  ax_mg, ay_mg, az_mg;
  int16_t  gx_dps10, gy_dps10, gz_dps10;
  uint16_t status;
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

static void uart_print(const char *s) {
  HAL_UART_Transmit(&huart2, (uint8_t*)s, (uint16_t)strlen(s), 200);
}

static inline uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline int16_t clamp_i16(int32_t v) {
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return (int16_t)v;
}

static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
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
  if (!armed) us = 1000;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, us);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, us);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, us);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, us);
}

static void motors_write_mask(uint8_t mask, uint16_t us) {
  us = clamp_u16(us, 1000, 2000);
  if (!armed) us = 1000;
  if (mask & M1_BIT) __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, us);
  if (mask & M2_BIT) __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, us);
  if (mask & M3_BIT) __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, us);
  if (mask & M4_BIT) __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, us);
}

static void motors_write_roll_mix(uint16_t thr, float u_roll_us) {
  if (!armed) {
    motors_write_all(1000);
    return;
  }
  uint16_t t = clamp_u16(thr, 1000, 2000);
  float u = clampf(u_roll_us, -U_LIM, +U_LIM);

  int32_t m1 = (int32_t)t - (int32_t)u + trim_m1_m4;
  int32_t m4 = (int32_t)t - (int32_t)u + trim_m1_m4;
  int32_t m2 = (int32_t)t + (int32_t)u;
  int32_t m3 = (int32_t)t + (int32_t)u;

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, clamp_u16((uint16_t)m1, 1000, 2000));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, clamp_u16((uint16_t)m2, 1000, 2000));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, clamp_u16((uint16_t)m3, 1000, 2000));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, clamp_u16((uint16_t)m4, 1000, 2000));
}

static void disarm_now(uint8_t set_failsafe_flag) {
  (void)set_failsafe_flag;
  armed = 0;
  thr_us = 1000;
  motors_write_all(1000);
  roll_angle = 0.0f; 
  Iterm_ang = 0.0f; // Resetear el acumulador integral por seguridad
}

static bool mpu_read_raw_gyro(int16_t *gx, int16_t *gy, int16_t *gz) {
  uint8_t d[6];
  if (HAL_I2C_Mem_Read(&hi2c1, mpu.i2c_addr, 0x43, 1, d, 6, 100) != HAL_OK) return false;
  *gx = (int16_t)((d[0]<<8) | d[1]);
  *gy = (int16_t)((d[2]<<8) | d[3]);
  *gz = (int16_t)((d[4]<<8) | d[5]);
  return true;
}

static bool mpu_read_raw_accel(int16_t *ax, int16_t *ay, int16_t *az) {
  uint8_t d[6];
  if (HAL_I2C_Mem_Read(&hi2c1, mpu.i2c_addr, 0x3B, 1, d, 6, 100) != HAL_OK) return false;
  *ax = (int16_t)((d[0]<<8) | d[1]);
  *ay = (int16_t)((d[2]<<8) | d[3]);
  *az = (int16_t)((d[4]<<8) | d[5]);
  return true;
}

static void cal_start(uint16_t samples) {
  cal_busy = 1;
  cal_samples_target = samples;
  cal_count = 0;
  gxs=gys=gzs=0;
  axs=ays=azs=0;
  cal_next_ms = HAL_GetTick();
  disarm_now(0);
}

static void cal_step(uint32_t now_ms) {
  if (!cal_busy) return;
  if ((int32_t)(now_ms - cal_next_ms) < 0) return;
  cal_next_ms += CAL_SAMPLE_PERIOD_MS;

  int16_t gx,gy,gz, ax,ay,az;
  if (!mpu_read_raw_gyro(&gx,&gy,&gz)) return;
  if (!mpu_read_raw_accel(&ax,&ay,&az)) return;

  gxs += gx; gys += gy; gzs += gz;
  axs += ax; ays += ay; azs += az;
  cal_count++;

  if (cal_count >= cal_samples_target) {
    mpu.Gx_offset = ((float)gxs / (float)cal_count) / 131.0f;
    mpu.Gy_offset = ((float)gys / (float)cal_count) / 131.0f;
    mpu.Gz_offset = ((float)gzs / (float)cal_count) / 131.0f;

    mpu.Ax_offset = ((float)axs / (float)cal_count) / 16384.0f;
    mpu.Ay_offset = ((float)ays / (float)cal_count) / 16384.0f;
    mpu.Az_offset = (((float)azs / (float)cal_count) / 16384.0f) - 1.0f;

    cal_busy = 0;
  }
}

static void lqr_recalculate_gains(void) {
  float r_val = r_ctrl;
  float b_val = b_param;

  if (r_val < 0.001f) r_val = 0.001f;
  if (b_val < 0.001f) b_val = 0.001f;

  K1 = sqrtf(q_ang / r_val);
  float inside = (2.0f / b_val) * K1 + (q_rate / r_val);
  if (inside < 0.0f) inside = 0.0f;
  K2 = sqrtf(inside);
}

/* ====================================================================
 * CÁLCULO DE CONTROL Y FILTROS
 * ==================================================================== */
static void lqr_roll_update_and_apply(void) {

  if (!imu_enable || cal_busy) return;

  ax_f = ax_f + alpha_acc  * (mpu.Ax - ax_f);
  ay_f = ay_f + alpha_acc  * (mpu.Ay - ay_f);
  az_f = az_f + alpha_acc  * (mpu.Az - az_f);

  gx_f = gx_f + alpha_gyro * (mpu.Gx - gx_f);
  gy_f = gy_f + alpha_gyro * (mpu.Gy - gy_f);
  gz_f = gz_f + alpha_gyro * (mpu.Gz - gz_f);

  float acc_roll = atan2f(ay_f, az_f) * (180.0f / 3.14159265f);
  roll_angle = 0.98f * (roll_angle + (gx_f * Ts)) + 0.02f * acc_roll;

  if (!armed) return;

  if ((int32_t)(HAL_GetTick() - manual_override_until_ms) < 0) {
    motors_write_mask(manual_mask, manual_us);
    return;
  }


  float err_angle = roll_sp - roll_angle;
  
  Iterm_ang += err_angle * Ts;
  Iterm_ang = clampf(Iterm_ang, -I_LIM, I_LIM); 

  float u = (K1 * err_angle) + (Ki_ang * Iterm_ang) - (K2 * gx_f); 
  
  u = clampf(u, -U_LIM, +U_LIM);

  motors_write_roll_mix(thr_us, u);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) {
    flag_200hz = 1; 
  }
}

static void handle_cmd(const CmdPkt *c) {
  if (!c || c->type != PKT_CMD) return;

  last_cmd_ms = HAL_GetTick();
  cmd_echo = c->cmd_seq;

  if (c->flags & CMD_PING) ping_echo = c->ping_id;
  if (c->flags & CMD_IMU_EN) imu_enable = (c->imu_enable ? 1 : 0);
  if (c->flags & CMD_ESTOP) { disarm_now(0); return; }
  if (c->flags & CMD_CAL_ALL) { cal_start(800); return; }
  if (c->flags & CMD_DISARM) disarm_now(0);
  
  if (c->flags & CMD_ARM) {
    armed = 1;
    motors_write_all(thr_us);
    roll_angle = 0.0f; 
    Iterm_ang = 0.0f;
    
    ax_f = mpu.Ax; ay_f = mpu.Ay; az_f = mpu.Az;
    gx_f = mpu.Gx; gy_f = mpu.Gy; gz_f = mpu.Gz;
  }
  
  if (c->flags & CMD_SET_THR) {
    thr_us = clamp_u16(c->thr_us, 1000, 2000);
    motors_write_all(thr_us);
  }
  if (c->flags & CMD_SET_MOTOR) {
    manual_mask = c->motor_mask;
    manual_us   = clamp_u16(c->motor_us, 1000, 2000);
    manual_override_until_ms = HAL_GetTick() + 300; 
    motors_write_mask(manual_mask, manual_us);
  }

  if (c->flags & CMD_SET_PID) {
    float v = ((float)((int16_t)c->motor_us)) / 256.0f;
    uint8_t sel = c->motor_mask;

    if (sel == 0) q_ang = v;
    else if (sel == 1) q_rate = v;
    else if (sel == 2) r_ctrl = v;
    else if (sel == 3) roll_sp = v;      
    else if (sel == 4) b_param = v;     
    else if (sel == 5) trim_m1_m4 = (int32_t)v; 
    else if (sel == 6) { Ki_ang = v; Iterm_ang = 0.0f; }

    if (sel == 0 || sel == 1 || sel == 2 || sel == 4) {
      lqr_recalculate_gains();
    }

    char msg[100];
    snprintf(msg, sizeof(msg), "LQR set: sel=%u v=%.3f | Gains: K1=%.3f K2=%.3f Ki=%.3f\r\n", sel, v, K1, K2, Ki_ang);
    uart_print(msg);
  }
}

static void build_send_telem(uint32_t now_ms) {
  TelemPkt p;
  memset(&p, 0, sizeof(p));
  p.type = PKT_TELEM;
  p.tx_seq = tx_seq++;
  p.t_ms = now_ms;

  uint16_t st = 0;

  if (imu_enable && !cal_busy && imu_last_ok) {
    p.ax_mg = clamp_i16((int32_t)(ax_f * 1000.0f));
    p.ay_mg = clamp_i16((int32_t)(ay_f * 1000.0f));
    p.az_mg = clamp_i16((int32_t)(az_f * 1000.0f));

    p.gx_dps10 = clamp_i16((int32_t)(gx_f * 10.0f));
    p.gy_dps10 = clamp_i16((int32_t)(gy_f * 10.0f));
    p.gz_dps10 = clamp_i16((int32_t)(gz_f * 10.0f));

    st |= ST_MPU_OK;
    if (mpu.accel_sanity_ok) st |= ST_ACCEL_OK;
  }

  if (armed) st |= ST_ARMED;
  if (cal_busy) st |= ST_CAL_BUSY;
  if (imu_enable) st |= ST_IMU_EN;
  if ((HAL_GetTick() - last_cmd_ms) > FAILSAFE_MS) st |= ST_FAILSAFE;

  p.status = st;
  p.cmd_echo = cmd_echo;
  p.ping_echo = ping_echo;

  CmdPkt ack;
  uint8_t acklen = 0;
  (void)NRF24_WriteAndReadAck(&hspi1, &nrf,
                             &p, (uint8_t)sizeof(p),
                             &ack, (uint8_t)sizeof(ack),
                             &acklen, 15);

  if (acklen == sizeof(CmdPkt) && ack.type == PKT_CMD) {
    handle_cmd(&ack);
  }
}

int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();

  HAL_Delay(200);
  uart_print("BOOT\r\n");

  motors_start();
  disarm_now(0);
  last_cmd_ms = HAL_GetTick();

  if (MPU9250_Init(&hi2c1, &mpu) != MPU_OK) {
    uart_print("MPU FAIL\r\n");
    while (1) { HAL_Delay(1000); }
  }
  uart_print("MPU OK\r\n");

  nrf.ce_port  = GPIOB;
  nrf.ce_pin   = GPIO_PIN_0;
  nrf.csn_port = GPIOB;
  nrf.csn_pin  = GPIO_PIN_1;

  if (NRF24_Init(&hspi1, &nrf) != NRF_OK) {
    uart_print("NRF FAIL\r\n");
    while (1) { HAL_Delay(1000); }
  }
  (void)NRF24_Configure_PTX(&hspi1, &nrf, RF_ADDR, RF_CH, NRF_DATARATE_250K, NRF_PA_LOW);
  uart_print("NRF PTX OK\r\n");

  lqr_recalculate_gains(); 

  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0); 
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  
  // ENCENDER RELOJ DEL TIMER Y ARRANCAR
  __HAL_RCC_TIM2_CLK_ENABLE();
  HAL_TIM_Base_Start_IT(&htim2); 

  next_hb_ms  = HAL_GetTick();

  /* BUCLE PRINCIPAL */
  while (1) {
    uint32_t now = HAL_GetTick();
    
    // 1. CONTROL A 200HZ EXACTOS
    if (flag_200hz) {
      flag_200hz = 0; // Bajamos la bandera
      
      if (imu_enable && !cal_busy) {
        imu_last_ok = (MPU9250_Read_Accel_Gyro(&hi2c1, &mpu) == MPU_OK) ? 1 : 0;
        if (imu_last_ok) {
          lqr_roll_update_and_apply();
        }
      }
    }

    // 2. Calibración
    cal_step(now);

    // 3. Failsafe
    if ((now - last_cmd_ms) > FAILSAFE_MS) {
      disarm_now(1);
    }

    // 4. Telemetría
    if ((int32_t)(now - next_hb_ms) >= 0) {
      next_hb_ms += HB_PERIOD_MS;
      build_send_telem(now);
    }
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) { Error_Handler(); }
}

static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) { Error_Handler(); }
}

static void MX_TIM1_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) { Error_Handler(); }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) { Error_Handler(); }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) Error_Handler();

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) Error_Handler();

  HAL_TIM_MspPostInit(&htim1);
}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
	
	__HAL_RCC_TIM2_CLK_ENABLE();

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83; 
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4999; 
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) { Error_Handler(); }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = CE_Pin|CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
