/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Horus Link - MPU9250 + Motors + NRF24 (ACK commands) robust
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include "mpu9250.h"
#include "nrf24.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static MPU9250_t mpu;
static NRF24_t nrf;

/* RF MUST match Pi */
static const uint8_t RF_ADDR[5] = {'D','R','O','N','E'};
static const uint8_t RF_CH = 110;

/* Rates */
static const uint32_t IMU_HZ = 200;
static const uint32_t IMU_PERIOD_MS = 5;      // 200 Hz
static const uint32_t HB_PERIOD_MS  = 50;     // 20 Hz heartbeat when IMU off/busy

/* Link / state */
static uint8_t  imu_enable = 1;
static uint8_t  armed = 0;
static uint16_t thr_us = 1000;
static uint32_t last_cmd_ms = 0;

// MODIFICACIÓN 1: Failsafe ampliado a 60 segundos para evitar cortes durante las pruebas
static const uint32_t FAILSAFE_MS = 15000 ;      

/* Sequence */
static uint8_t tx_seq = 0;
static uint8_t cmd_echo = 0;
static uint8_t ping_echo = 0;

/* Calibration state machine (non-blocking) */
static uint8_t cal_busy = 0;
static uint16_t cal_samples_target = 800;
static uint16_t cal_count = 0;
static int32_t gxs=0,gys=0,gzs=0;
static int32_t axs=0,ays=0,azs=0;
static uint32_t cal_next_ms = 0;
static const uint32_t CAL_SAMPLE_PERIOD_MS = 5;  // same as your old delay

/* Schedulers */
static uint32_t next_imu_ms = 0;
static uint32_t next_hb_ms  = 0;

/* Packet types */
#define PKT_TELEM   0xA1
#define PKT_CMD     0xB1

/* Status bits */
#define ST_MPU_OK     (1U<<0)
#define ST_ACCEL_OK   (1U<<1)
#define ST_ARMED      (1U<<2)
#define ST_CAL_BUSY   (1U<<3)
#define ST_IMU_EN     (1U<<4)
#define ST_FAILSAFE   (1U<<5)

/* Command flags */
#define CMD_ARM        (1U<<0)
#define CMD_DISARM     (1U<<1)
#define CMD_ESTOP      (1U<<2)
#define CMD_SET_THR    (1U<<3)
#define CMD_SET_MOTOR  (1U<<4)
#define CMD_IMU_EN     (1U<<5)
#define CMD_CAL_ALL    (1U<<6)
#define CMD_PING       (1U<<7)

/* Motor mask bits */
#define M1_BIT (1U<<0)
#define M2_BIT (1U<<1)
#define M3_BIT (1U<<2)
#define M4_BIT (1U<<3)

#pragma pack(push,1)
/* Pi -> STM (ACK payload) : 12 bytes */
typedef struct {
  uint8_t  type;        // 0xB1
  uint8_t  cmd_seq;     // increments each command
  uint16_t flags;       // CMD_*
  uint16_t thr_us;      // 1000..2000
  uint8_t  motor_mask;  // M1..M4 bits
  uint16_t motor_us;    // 1000..2000
  uint8_t  imu_enable;  // 0/1
  uint8_t  ping_id;     // increments when user 'ping'
} CmdPkt;

/* STM -> Pi : 24 bytes (<=32) */
typedef struct {
  uint8_t  type;        // 0xA1
  uint8_t  tx_seq;      // increments always
  uint32_t t_ms;
  int16_t  ax_mg, ay_mg, az_mg;
  int16_t  gx_dps10, gy_dps10, gz_dps10;
  uint16_t status;
  uint8_t  cmd_echo;    // last cmd_seq applied/accepted
  uint8_t  ping_echo;   // last ping_id seen
} TelemPkt;
#pragma pack(pop)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
static void uart_print(const char *s);

static inline uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi);
static inline int16_t clamp_i16(int32_t v);

static void motors_start(void);
static void motors_write_all(uint16_t us);
static void motors_write_mask(uint8_t mask, uint16_t us);
static void disarm_now(uint8_t set_failsafe_flag);

static bool mpu_read_raw_gyro(int16_t *gx, int16_t *gy, int16_t *gz);
static bool mpu_read_raw_accel(int16_t *ax, int16_t *ay, int16_t *az);
static void cal_start(uint16_t samples);
static void cal_step(uint32_t now_ms);

static void build_send_telem(uint32_t now_ms);
static void handle_cmd(const CmdPkt *c);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static void uart_print(const char *s)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)s, (uint16_t)strlen(s), 200);
}

static inline uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline int16_t clamp_i16(int32_t v)
{
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return (int16_t)v;
}

static void motors_start(void)
{
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  __HAL_TIM_MOE_ENABLE(&htim1);
}

static void motors_write_all(uint16_t us)
{
  us = clamp_u16(us, 1000, 2000);
  if (!armed) us = 1000;

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, us);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, us);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, us);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, us);
}

static void motors_write_mask(uint8_t mask, uint16_t us)
{
  us = clamp_u16(us, 1000, 2000);
  if (!armed) us = 1000;

  if (mask & M1_BIT) __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, us);
  if (mask & M2_BIT) __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, us);
  if (mask & M3_BIT) __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, us);
  if (mask & M4_BIT) __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, us);
}

static void disarm_now(uint8_t set_failsafe_flag)
{
  (void)set_failsafe_flag;
  armed = 0;
  thr_us = 1000;
  motors_write_all(1000);
}

/* --- raw reads for non-blocking calibration --- */
static bool mpu_read_raw_gyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
  uint8_t d[6];
  if (HAL_I2C_Mem_Read(&hi2c1, mpu.i2c_addr, 0x43, 1, d, 6, 100) != HAL_OK) return false;
  *gx = (int16_t)((d[0]<<8) | d[1]);
  *gy = (int16_t)((d[2]<<8) | d[3]);
  *gz = (int16_t)((d[4]<<8) | d[5]);
  return true;
}

static bool mpu_read_raw_accel(int16_t *ax, int16_t *ay, int16_t *az)
{
  uint8_t d[6];
  if (HAL_I2C_Mem_Read(&hi2c1, mpu.i2c_addr, 0x3B, 1, d, 6, 100) != HAL_OK) return false;
  *ax = (int16_t)((d[0]<<8) | d[1]);
  *ay = (int16_t)((d[2]<<8) | d[3]);
  *az = (int16_t)((d[4]<<8) | d[5]);
  return true;
}

static void cal_start(uint16_t samples)
{
  cal_busy = 1;
  cal_samples_target = samples;
  cal_count = 0;
  gxs=gys=gzs=0;
  axs=ays=azs=0;
  cal_next_ms = HAL_GetTick();
  // Por seguridad: desarmar y dejar 1000us
  disarm_now(0);
}

static void cal_step(uint32_t now_ms)
{
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
    // Gyro offsets (deg/s) using 131 LSB/dps
    float gxo = ((float)gxs / (float)cal_count) / 131.0f;
    float gyo = ((float)gys / (float)cal_count) / 131.0f;
    float gzo = ((float)gzs / (float)cal_count) / 131.0f;

    // Accel offsets (g) using 16384 LSB/g
    float axo = ((float)axs / (float)cal_count) / 16384.0f;
    float ayo = ((float)ays / (float)cal_count) / 16384.0f;
    float azo = ((float)azs / (float)cal_count) / 16384.0f;

    mpu.Gx_offset = gxo;
    mpu.Gy_offset = gyo;
    mpu.Gz_offset = gzo;

    mpu.Ax_offset = axo;
    mpu.Ay_offset = ayo;
    // Z debe ser +1g en reposo
    mpu.Az_offset = azo - 1.0f;

    cal_busy = 0;
  }
}

static void handle_cmd(const CmdPkt *c)
{
  // Validación básica
  if (!c || c->type != PKT_CMD) return;

  last_cmd_ms = HAL_GetTick();
  cmd_echo = c->cmd_seq;

  // Ping
  if (c->flags & CMD_PING) {
    ping_echo = c->ping_id;
  }

  // IMU enable
  if (c->flags & CMD_IMU_EN) {
    imu_enable = (c->imu_enable ? 1 : 0);
  }

  // ESTOP
  if (c->flags & CMD_ESTOP) {
    disarm_now(0);
    return;
  }

  // CAL (gyro+acc juntos)
  if (c->flags & CMD_CAL_ALL) {
    cal_start(800); // fijo por ahora
    return;
  }

  // MODIFICACIÓN 2: Quitamos el `if (!cal_busy)` de todos los comandos
  // para forzar que el dron te obedezca siempre, ignorando si se quedó calibrando.

  // ARM/DISARM
  if (c->flags & CMD_DISARM) {
    disarm_now(0);
  }
  if (c->flags & CMD_ARM) {
    armed = 1;
    cal_busy = 0; // Apagamos a la fuerza la calibración si estaba encendida
    motors_write_all(thr_us);
  }

  // Throttle global
  if (c->flags & CMD_SET_THR) {
    thr_us = clamp_u16(c->thr_us, 1000, 2000);
    motors_write_all(thr_us);
  }

  // Motor individual (mask)
  if (c->flags & CMD_SET_MOTOR) {
    motors_write_mask(c->motor_mask, c->motor_us);
  }
}

static void build_send_telem(uint32_t now_ms)
{
  TelemPkt p;
  memset(&p, 0, sizeof(p));
  p.type = PKT_TELEM;
  p.tx_seq = tx_seq++;
  p.t_ms = now_ms;

  uint16_t st = 0;

  if (imu_enable && !cal_busy) {
    MPU_Status r = MPU9250_Read_Accel_Gyro(&hi2c1, &mpu);
    if (r == MPU_OK) {
      p.ax_mg = clamp_i16((int32_t)(mpu.Ax * 1000.0f));
      p.ay_mg = clamp_i16((int32_t)(mpu.Ay * 1000.0f));
      p.az_mg = clamp_i16((int32_t)(mpu.Az * 1000.0f));

      p.gx_dps10 = clamp_i16((int32_t)(mpu.Gx * 10.0f));
      p.gy_dps10 = clamp_i16((int32_t)(mpu.Gy * 10.0f));
      p.gz_dps10 = clamp_i16((int32_t)(mpu.Gz * 10.0f));

      st |= ST_MPU_OK;
      if (mpu.accel_sanity_ok) st |= ST_ACCEL_OK;
    }
  }

  if (armed) st |= ST_ARMED;
  if (cal_busy) st |= ST_CAL_BUSY;
  if (imu_enable) st |= ST_IMU_EN;

  // failsafe flag (solo indicador)
  if ((HAL_GetTick() - last_cmd_ms) > FAILSAFE_MS) st |= ST_FAILSAFE;

  p.status = st;
  p.cmd_echo = cmd_echo;
  p.ping_echo = ping_echo;

  // TX + read ACK (commands)
  CmdPkt ack;
  uint8_t acklen = 0;
  NRF_Status s = NRF24_WriteAndReadAck(&hspi1, &nrf, &p, (uint8_t)sizeof(p),
                                      &ack, (uint8_t)sizeof(ack), &acklen, 15);
  (void)s;

  if (acklen == sizeof(CmdPkt) && ack.type == PKT_CMD) {
    handle_cmd(&ack);
  }
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_Delay(200);
  uart_print("BOOT\r\n");

  motors_start();
  disarm_now(0);
  
  // MODIFICACIÓN 3: PRUEBA FORZADA HARDWARE
  // Descomenta estas líneas si la Raspberry Pi dice que armó pero no giran. 
  // Esto obligará al motor 1 a girar apenas le conectes la batería.
  // HAL_Delay(3000); // Esperar a que el ESC reconozca los 1000us iniciales de disarm_now()
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1150); // Girar Motor 1 a baja velocidad
  // -----------------------------------------------------------

  last_cmd_ms = HAL_GetTick();

  // ---- MPU init ----
  MPU_Status st = MPU9250_Init(&hi2c1, &mpu);
  if (st != MPU_OK) {
    uart_print("MPU FAIL\r\n");
    while (1) { HAL_Delay(1000); }
  }
  uart_print("MPU OK\r\n");

  // ---- NRF init ----
  nrf.ce_port  = GPIOB;
  nrf.ce_pin   = GPIO_PIN_0; // CE = PB0
  nrf.csn_port = GPIOB;
  nrf.csn_pin  = GPIO_PIN_1; // CSN = PB1

  NRF_Status ns = NRF24_Init(&hspi1, &nrf);
  if (ns != NRF_OK) {
    uart_print("NRF FAIL\r\n");
    while (1) { HAL_Delay(1000); }
  }
  (void)NRF24_Configure_PTX(&hspi1, &nrf, RF_ADDR, RF_CH, NRF_DATARATE_250K, NRF_PA_LOW);
  uart_print("NRF PTX OK\r\n");

  next_imu_ms = HAL_GetTick();
  next_hb_ms  = HAL_GetTick();
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    // non-blocking calibration steps
    cal_step(now);

    // FAILSAFE: if no cmd refresh -> disarm
    if ((now - last_cmd_ms) > FAILSAFE_MS) {
      disarm_now(1);
    }

    // Scheduler:
    // - if IMU enabled and not calibrating -> 200 Hz telemetry
    // - else -> heartbeat 20 Hz (still does TX+ACK to keep command link alive)
    if (imu_enable && !cal_busy) {
      if ((int32_t)(now - next_imu_ms) >= 0) {
        next_imu_ms += IMU_PERIOD_MS;
        build_send_telem(now);
      }
    } else {
      if ((int32_t)(now - next_hb_ms) >= 0) {
        next_hb_ms += HB_PERIOD_MS;
        build_send_telem(now);
      }
    }
    /* USER CODE END 3 */
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

  // *** IMPORTANT: slow down SPI for nRF modules ***
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64; //

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

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
