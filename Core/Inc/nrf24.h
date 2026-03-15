#ifndef NRF24_H
#define NRF24_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    // Pines control
    GPIO_TypeDef *ce_port;
    uint16_t ce_pin;

    GPIO_TypeDef *csn_port;
    uint16_t csn_pin;

    // Config útil para debug
    uint8_t channel;      // 0..125
    uint8_t addr[5];      // pipe0/tx address (5 bytes)
} NRF24_t;

typedef enum {
    NRF_OK = 0,
    NRF_ERR_SPI = 1,
    NRF_ERR_NOT_FOUND = 2,
    NRF_ERR_BAD_ARG = 3,
    NRF_ERR_TIMEOUT = 4
} NRF_Status;

typedef enum {
    NRF_DATARATE_250K = 0,
    NRF_DATARATE_1M,
    NRF_DATARATE_2M
} NRF_Datarate;

typedef enum {
    NRF_PA_MIN = 0,
    NRF_PA_LOW,
    NRF_PA_HIGH,
    NRF_PA_MAX
} NRF_PaLevel;

// Init/config
NRF_Status NRF24_Init(SPI_HandleTypeDef *hspi, NRF24_t *nrf);
bool       NRF24_IsPresent(SPI_HandleTypeDef *hspi, NRF24_t *nrf);

NRF_Status NRF24_Configure_PTX(SPI_HandleTypeDef *hspi, NRF24_t *nrf,
                               const uint8_t addr[5],
                               uint8_t channel,
                               NRF_Datarate dr,
                               NRF_PaLevel pa);

// TX + ACK payload
NRF_Status NRF24_Write(SPI_HandleTypeDef *hspi, NRF24_t *nrf,
                       const void *buf, uint8_t len, uint32_t timeout_ms);

bool       NRF24_AckPayloadAvailable(SPI_HandleTypeDef *hspi, NRF24_t *nrf);
uint8_t    NRF24_ReadAckPayload(SPI_HandleTypeDef *hspi, NRF24_t *nrf,
                                void *buf, uint8_t maxlen);

// Utils
void       NRF24_FlushTX(SPI_HandleTypeDef *hspi, NRF24_t *nrf);
void       NRF24_FlushRX(SPI_HandleTypeDef *hspi, NRF24_t *nrf);
void       NRF24_ClearIRQs(SPI_HandleTypeDef *hspi, NRF24_t *nrf);

#endif
