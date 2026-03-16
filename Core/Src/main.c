/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Horus: MPU9250 -> nRF24 -> Raspberry Pi (IMU 200 Hz + CMD ACK)
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
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static MPU9250_t mpu;
static NRF24_t nrf;
static char txbuf[128];

/* RF MUST match Pi */
static const uint8_t RF_ADDR[5] = {'D','R','O','N','E'};
static const uint8_t RF_CH = 110;

/* Timing */
static const uint32_t IMU_HZ = 200;
static const uint32_t IMU_PERIOD_MS = 5;
static uint32_t next_ms = 0;
static uint8_t seq8 = 0;

/* Packet types */
#define PKT_IMU  0xA1
#define PKT_CMD  0xB1

/* IMU status bits */
#define ST_MPU_OK     (1U<<0)
#define ST_ACCEL_OK   (1U<<1)
#define ST_ARMED      (1U<<2)
#define ST_LASTCMD_OK (1U<<3)

/* CMD flags (Pi -> STM via ACK payload) */
#define CMD_CAL_GYRO   (1U<<0)
#define CMD_CAL_ACCEL  (1U<<1)
#define CMD_ARM        (1U<<2)
#define CMD_DISARM     (1U<<3)
#define CMD_SET_THR    (1U<<4)
#define CMD_SET_MOTOR  (1U<<5)
#define CMD_ESTOP      (1U<<6)

/* Motor mask bits */
#define M1_BIT (1U<<0)
#define M2_BIT (1U<<1)
#define M3_BIT (1U<<2)
#define M4_BIT (1U<<3)

/* Control state */
static volatile uint8_t armed = 0;     // por seguridad: arranca desarmado
static uint16_t m_us[4] = {1000,1000,1000,1000};
static uint16_t thr_us = 1000;

static volatile uint8_t imu_enable = 1;    // la Pi puede apagar transmisión
static volatile uint8_t last_cmd_seq = 0;
static volatile uint8_t last_cmd_seen = 0;

#pragma pack(push,1)
typedef struct {
  uint8_t  type;      // 0xA1
  uint8_t  seq;       // 0..255
  uint32_t t_ms;      // HAL_GetTick()
  int16_t  ax_mg, ay_mg, az_mg;          // accel mg
  int16_t  gx_dps10, gy_dps10, gz_dps10; // gyro 0.1 dps
  uint16_t status;    // bits
} ImuPkt;

/* 12 bytes CMD (de tu Python) */
typedef struct {
  uint8_t  type;       // 0xB1
  uint8_t  cmd_seq;
  uint16_t flags;
  uint16_t thr_us;
  uint8_t  motor_mask;
  uint16_t motor_us;
  uint8_t  imu_enable;
  uint8_t  r0;
  uint8_t  r1;
} CmdPkt;
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
static inline int16_t clamp_i16(int32_t v);
static inline uint16_t clamp_u16(uint16_t v);

static void pwm_apply(void);
static void set_all(uint16_t us);
static void set_mask(uint8_t mask, uint16_t us);

static void handle_cmd(const CmdPkt *c, uint8_t len);
static void send_imu_and_poll_cmd(uint32_t now_ms);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static void uart_print(const char *s)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)s, (uint16_t)strlen(s), 200);
}

static inline int16_t clamp_i16(int32_t v)
{
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return (int16_t)v;
}

static inline uint16_t clamp_u16(uint16_t v)
{
  if (v < 1000) return 1000;
  if (v > 2000) return 2000;
  return v;
}

/* --- PWM helpers --- */
static void pwm_apply(void)
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, armed ? m_us[0] : 1000);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, armed ? m_us[1] : 1000);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, armed ? m_us[2] : 1000);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, armed ? m_us[3] : 1000);
}

static void set_all(uint16_t us)
{
  us = clamp_u16(us);
  thr_us = us;
  m_us[0] = m_us[1] = m_us[2] = m_us[3] = us;
  pwm_apply();
}

static void set_mask(uint8_t mask, uint16_t us)
{
  us = clamp_u16(us);
  if(mask & M1_BIT) m_us[0] = us;
  if(mask & M2_BIT) m_us[1] = us;
  if(mask & M3_BIT) m_us[2] = us;
  if(mask & M4_BIT) m_us[3] = us;
  pwm_apply();
}

/* --- CMD handler --- */
static void handle_cmd(const CmdPkt *c, uint8_t len)
{
  if (!c || len < sizeof(CmdPkt)) return;
  if (c->type != PKT_CMD) return;

  last_cmd_seq = c->cmd_seq;
  last_cmd_seen = 1;

  /* imu enable controls TX of IMU packets */
  imu_enable = (c->imu_enable != 0);

  if (c->flags & CMD_ESTOP) {
    armed = 0;
    set_all(1000);
    return;
  }

  if (c->flags & CMD_DISARM) {
    armed = 0;
    set_all(1000);
  }

  if (c->flags & CMD_ARM) {
    armed = 1;
    pwm_apply();
  }

  if (c->flags & CMD_SET_THR) {
    set_all(c->thr_us);
  }

  if (c->flags & CMD_SET_MOTOR) {
    set_mask(c->motor_mask, c->motor_us);
  }

  if (c->flags & CMD_CAL_GYRO) {
    uart_print("CAL_GYRO...\r\n");
    (void)MPU9250_Calibrate_Gyro(&hi2c1, &mpu, 800);
    uart_print("CAL_GYRO DONE\r\n");
  }

  if (c->flags & CMD_CAL_ACCEL) {
    uart_print("CAL_ACCEL...\r\n");
    (void)MPU9250_Calibrate_Accel(&hi2c1, &mpu, 800);
    uart_print("CAL_ACCEL DONE\r\n");
  }
}

/* --- IMU TX + ACK read --- */
static void send_imu_and_poll_cmd(uint32_t now_ms)
{
  ImuPkt p;
  memset(&p, 0, sizeof(p));
  p.type = PKT_IMU;
  p.seq  = seq8++;
  p.t_ms = now_ms;

  uint16_t st_bits = 0;

  MPU_Status r = MPU9250_Read_Accel_Gyro(&hi2c1, &mpu);
  if (r == MPU_OK) {
    p.ax_mg = clamp_i16((int32_t)(mpu.Ax * 1000.0f));
    p.ay_mg = clamp_i16((int32_t)(mpu.Ay * 1000.0f));
    p.az_mg = clamp_i16((int32_t)(mpu.Az * 1000.0f));

    p.gx_dps10 = clamp_i16((int32_t)(mpu.Gx * 10.0f));
    p.gy_dps10 = clamp_i16((int32_t)(mpu.Gy * 10.0f));
    p.gz_dps10 = clamp_i16((int32_t)(mpu.Gz * 10.0f));

    st_bits |= ST_MPU_OK;
    if (mpu.accel_sanity_ok) st_bits |= ST_ACCEL_OK;
  }

  if (armed) st_bits |= ST_ARMED;
  if (last_cmd_seen) st_bits |= ST_LASTCMD_OK;

  p.status = st_bits;

  /* If imu_enable=0, we still transmit a small packet sometimes?
     For now: if disabled, still send at 10Hz just to keep link alive.
  */
  static uint32_t keepalive_next = 0;
  uint8_t do_send = 1;
  if (!imu_enable) {
    if ((int32_t)(now_ms - keepalive_next) >= 0) {
      keepalive_next = now_ms + 100;
      do_send = 1;
    } else {
      do_send = 0;
    }
  }

  CmdPkt cmd;
  uint8_t acklen = 0;

  if (do_send) {
    (void)NRF24_WriteAndReadAck(&hspi1, &nrf,
                               &p, (uint8_t)sizeof(p),
                               &cmd, (uint8_t)sizeof(cmd),
                               &acklen,
                               15);
  } else {
    // even if we don't send IMU, we can't receive ACK; that's OK.
    return;
  }

  if (acklen >= sizeof(CmdPkt)) {
    handle_cmd(&cmd, acklen);
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

  /* PWM start */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  __HAL_TIM_MOE_ENABLE(&htim1);

  /* Start safe */
  armed = 0;
  set_all(1000);

  /* MPU init */
  MPU_Status st = MPU9250_Init(&hi2c1, &mpu);
  snprintf(txbuf, sizeof(txbuf), "MPU init st=%u who=0x%02X addr=0x%02X\r\n",
           (unsigned)st, (unsigned)mpu.whoami, (unsigned)(mpu.i2c_addr >> 1));
  uart_print(txbuf);
  if (st != MPU_OK) { uart_print("MPU FAIL\r\n"); while (1) HAL_Delay(1000); }

  uart_print("Gyro calib... keep still\r\n");
  st = MPU9250_Calibrate_Gyro(&hi2c1, &mpu, 800);
  snprintf(txbuf, sizeof(txbuf), "Gyro calib st=%u\r\n", (unsigned)st);
  uart_print(txbuf);

  /* Set IMU sample divider to ~200Hz if needed */
  uint8_t div = 4;
  (void)HAL_I2C_Mem_Write(&hi2c1, mpu.i2c_addr, 0x19, 1, &div, 1, 100);

  /* NRF init */
  nrf.ce_port  = GPIOB;
  nrf.ce_pin   = GPIO_PIN_0; // CE = PB0
  nrf.csn_port = GPIOB;
  nrf.csn_pin  = GPIO_PIN_1; // CSN = PB1

  NRF_Status ns = NRF24_Init(&hspi1, &nrf);
  snprintf(txbuf, sizeof(txbuf), "NRF init st=%d\r\n", (int)ns);
  uart_print(txbuf);
  if (ns != NRF_OK) { uart_print("NRF FAIL\r\n"); while (1) HAL_Delay(1000); }

  (void)NRF24_Configure_PTX(&hspi1, &nrf, RF_ADDR, RF_CH, NRF_DATARATE_250K, NRF_PA_LOW);
  uart_print("NRF PTX OK\r\n");

  next_ms = HAL_GetTick();
  uart_print("IMU 200Hz -> NRF (ACK CMD enabled)\r\n");
  uart_print("Start DISARMED. Use Pi: arm / t 1100 ...\r\n");
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - next_ms) >= 0) {
      next_ms += IMU_PERIOD_MS;
      send_imu_and_poll_cmd(now);
    }
    /* USER CODE END 3 */
  }
}

/* ===== CubeMX init functions ===== */

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
