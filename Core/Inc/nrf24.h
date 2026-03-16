#pragma once
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
  NRF_OK = 0,
  NRF_ERR_I2C = 1,
  NRF_ERR_WHOAMI = 2,
  NRF_ERR_CONFIG = 3,
  NRF_ERR_BAD_ARG = 4,
  NRF_ERR_NOT_FOUND = 5,
  NRF_ERR_TIMEOUT = 6
} NRF_Status;

typedef enum {
  NRF_DATARATE_250K = 0,
  NRF_DATARATE_1M   = 1,
  NRF_DATARATE_2M   = 2
} NRF_Datarate;

typedef enum {
  NRF_PA_MIN  = 0,
  NRF_PA_LOW  = 1,
  NRF_PA_HIGH = 2,
  NRF_PA_MAX  = 3
} NRF_PaLevel;

typedef struct {
  GPIO_TypeDef *ce_port;
  uint16_t      ce_pin;
  GPIO_TypeDef *csn_port;
  uint16_t      csn_pin;

  uint8_t addr[5];
  uint8_t channel;
} NRF24_t;

/* Existing API */
NRF_Status NRF24_Init(SPI_HandleTypeDef *hspi, NRF24_t *nrf);
bool NRF24_IsPresent(SPI_HandleTypeDef *hspi, NRF24_t *nrf);
NRF_Status NRF24_Configure_PTX(SPI_HandleTypeDef *hspi, NRF24_t *nrf,
                               const uint8_t addr[5], uint8_t channel,
                               NRF_Datarate dr, NRF_PaLevel pa);
NRF_Status NRF24_Write(SPI_HandleTypeDef *hspi, NRF24_t *nrf,
                       const void *buf, uint8_t len, uint32_t timeout_ms);

void NRF24_ClearIRQs(SPI_HandleTypeDef *hspi, NRF24_t *nrf);
void NRF24_FlushTX(SPI_HandleTypeDef *hspi, NRF24_t *nrf);
void NRF24_FlushRX(SPI_HandleTypeDef *hspi, NRF24_t *nrf);

bool NRF24_AckPayloadAvailable(SPI_HandleTypeDef *hspi, NRF24_t *nrf);
uint8_t NRF24_ReadAckPayload(SPI_HandleTypeDef *hspi, NRF24_t *nrf,
                             void *buf, uint8_t maxlen);

/* NEW: one-shot TX + optional ACK read */
NRF_Status NRF24_WriteAndReadAck(SPI_HandleTypeDef *hspi, NRF24_t *nrf,
                                 const void *txbuf, uint8_t txlen,
                                 void *ackbuf, uint8_t ackmax,
                                 uint8_t *acklen_out,
                                 uint32_t timeout_ms);
