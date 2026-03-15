#include "nrf24.h"
#include <string.h>

#define CMD_R_REGISTER      0x00
#define CMD_W_REGISTER      0x20
#define CMD_R_RX_PAYLOAD    0x61
#define CMD_W_TX_PAYLOAD    0xA0
#define CMD_R_RX_PL_WID     0x60
#define CMD_FLUSH_TX        0xE1
#define CMD_FLUSH_RX        0xE2
#define CMD_ACTIVATE  				0x50
#define CMD_NOP             0xFF
//reg
#define REG_CONFIG          0x00
#define REG_EN_AA           0x01
#define REG_EN_RXADDR       0x02
#define REG_SETUP_RETR      0x04
#define REG_RF_CH           0x05
#define REG_RF_SETUP        0x06
#define REG_STATUS          0x07
#define REG_RX_ADDR_P0      0x0A
#define REG_TX_ADDR         0x10
#define REG_FIFO_STATUS     0x17
#define REG_DYNPD           0x1C
#define REG_FEATURE         0x1D

//bits
#define CONFIG_PRIM_RX      (1U<<0)
#define CONFIG_PWR_UP       (1U<<1)
#define CONFIG_EN_CRC       (1U<<3)
#define CONFIG_CRCO         (1U<<2)   // 1 = 2 bytes CRC

#define STATUS_RX_DR        (1U<<6)
#define STATUS_TX_DS        (1U<<5)
#define STATUS_MAX_RT       (1U<<4)

#define FEATURE_EN_ACK_PAY  (1U<<1)
#define FEATURE_EN_DPL      (1U<<2)

// RF_SETUP bits
#define RF_DR_HIGH          (1U<<3)
#define RF_DR_LOW           (1U<<5)
#define RF_PWR_LOW          (1U<<1)
#define RF_PWR_HIGH         (1U<<2)

//helpers gpio
static inline void csn_low (NRF24_t *n){ HAL_GPIO_WritePin(n->csn_port, n->csn_pin, GPIO_PIN_RESET); }
static inline void csn_high(NRF24_t *n){ HAL_GPIO_WritePin(n->csn_port, n->csn_pin, GPIO_PIN_SET);   }
static inline void ce_low  (NRF24_t *n){ HAL_GPIO_WritePin(n->ce_port,  n->ce_pin,  GPIO_PIN_RESET); }
static inline void ce_high (NRF24_t *n){ HAL_GPIO_WritePin(n->ce_port,  n->ce_pin,  GPIO_PIN_SET);   }

static uint8_t spi_xfer(SPI_HandleTypeDef *hspi, uint8_t b)
{
    uint8_t r = 0;
    if (HAL_SPI_TransmitReceive(hspi, &b, &r, 1, HAL_MAX_DELAY) != HAL_OK) {
        return 0; // si falla spi devolvemos 0
    }
    return r;
}

static uint8_t cmd_rw(SPI_HandleTypeDef *hspi, NRF24_t *nrf,
                      uint8_t cmd, const uint8_t *tx, uint8_t *rx, uint8_t len)
{
    csn_low(nrf);
    uint8_t status = spi_xfer(hspi, cmd);
    for(uint8_t i=0;i<len;i++){
        uint8_t t = tx ? tx[i] : 0xFF;
        uint8_t r = spi_xfer(hspi, t);
        if(rx) rx[i] = r;
    }
    csn_high(nrf);
    return status;
}

static uint8_t read_reg(SPI_HandleTypeDef *hspi, NRF24_t *nrf, uint8_t reg)
{
    uint8_t v=0;
    cmd_rw(hspi, nrf, CMD_R_REGISTER | (reg & 0x1F), NULL, &v, 1);
    return v;
}

static void write_reg(SPI_HandleTypeDef *hspi, NRF24_t *nrf, uint8_t reg, uint8_t v)
{
    cmd_rw(hspi, nrf, CMD_W_REGISTER | (reg & 0x1F), &v, NULL, 1);
}

static void nrf_activate(SPI_HandleTypeDef *hspi, NRF24_t *nrf)
{
    uint8_t code = 0x73;
    csn_low(nrf);
    spi_xfer(hspi, CMD_ACTIVATE);
    spi_xfer(hspi, code);
    csn_high(nrf);
}

static void write_reg_buf(SPI_HandleTypeDef *hspi, NRF24_t *nrf, uint8_t reg, const uint8_t *buf, uint8_t len)
{
    cmd_rw(hspi, nrf, CMD_W_REGISTER | (reg & 0x1F), buf, NULL, len);
}

void NRF24_ClearIRQs(SPI_HandleTypeDef *hspi, NRF24_t *nrf)
{
    (void)hspi;
    // escribir 1 en bits RX_DR/TX_DS/MAX_RT para limpiar
    write_reg(hspi, nrf, REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);
}

void NRF24_FlushTX(SPI_HandleTypeDef *hspi, NRF24_t *nrf)
{
    cmd_rw(hspi, nrf, CMD_FLUSH_TX, NULL, NULL, 0);
}

void NRF24_FlushRX(SPI_HandleTypeDef *hspi, NRF24_t *nrf)
{
    cmd_rw(hspi, nrf, CMD_FLUSH_RX, NULL, NULL, 0);
}

static uint8_t get_status(SPI_HandleTypeDef *hspi, NRF24_t *nrf)
{
    csn_low(nrf);
    uint8_t st = spi_xfer(hspi, CMD_NOP);
    csn_high(nrf);
    return st;
}

bool NRF24_IsPresent(SPI_HandleTypeDef *hspi, NRF24_t *nrf)
{
    // eescritura/lectura simple en RF_CH para verificar spi y chip
    uint8_t ch0 = read_reg(hspi, nrf, REG_RF_CH);
    write_reg(hspi, nrf, REG_RF_CH, (uint8_t)(ch0 ^ 0x2A));
    uint8_t ch1 = read_reg(hspi, nrf, REG_RF_CH);
    write_reg(hspi, nrf, REG_RF_CH, ch0);
    return (uint8_t)(ch0 ^ 0x2A) == ch1;
}

NRF_Status NRF24_Init(SPI_HandleTypeDef *hspi, NRF24_t *nrf)
{
    if(!hspi || !nrf || !nrf->ce_port || !nrf->csn_port) return NRF_ERR_BAD_ARG;

    csn_high(nrf);
    ce_low(nrf);
    HAL_Delay(5);
		nrf_activate(hspi, nrf);

    // autoACK solo en pipe0, habilitar RXADDR pipe0
    write_reg(hspi, nrf, REG_EN_AA,     0x01);
    write_reg(hspi, nrf, REG_EN_RXADDR, 0x01);

    // CRC 16-bit PRIM_RX=0 y PWR_UP=0
    write_reg(hspi, nrf, REG_CONFIG, CONFIG_EN_CRC | CONFIG_CRCO);

    NRF24_ClearIRQs(hspi, nrf);
    NRF24_FlushTX(hspi, nrf);
    NRF24_FlushRX(hspi, nrf);

    //verificacion
    if(!NRF24_IsPresent(hspi, nrf)) return NRF_ERR_NOT_FOUND;

    return NRF_OK;
}

static void set_channel(SPI_HandleTypeDef *hspi, NRF24_t *nrf, uint8_t ch)
{
    write_reg(hspi, nrf, REG_RF_CH, ch);
    nrf->channel = ch;
}

static void set_datarate(SPI_HandleTypeDef *hspi, NRF24_t *nrf, NRF_Datarate dr)
{
    uint8_t rf = read_reg(hspi, nrf, REG_RF_SETUP);
    rf &= ~(RF_DR_HIGH | RF_DR_LOW);
    if(dr == NRF_DATARATE_250K) rf |= RF_DR_LOW;
    else if(dr == NRF_DATARATE_2M) rf |= RF_DR_HIGH;
    write_reg(hspi, nrf, REG_RF_SETUP, rf);
}

static void set_pa(SPI_HandleTypeDef *hspi, NRF24_t *nrf, NRF_PaLevel pa)
{
    uint8_t rf = read_reg(hspi, nrf, REG_RF_SETUP);
    rf &= ~(RF_PWR_LOW | RF_PWR_HIGH);
    switch(pa){
        case NRF_PA_MIN:  break;
        case NRF_PA_LOW:  rf |= RF_PWR_LOW; break;
        case NRF_PA_HIGH: rf |= RF_PWR_HIGH; break;
        case NRF_PA_MAX:  rf |= (RF_PWR_LOW | RF_PWR_HIGH); break;
    }
    write_reg(hspi, nrf, REG_RF_SETUP, rf);
}

static void set_crc16(SPI_HandleTypeDef *hspi, NRF24_t *nrf, bool en)
{
    uint8_t cfg = read_reg(hspi, nrf, REG_CONFIG);
    if(en) cfg |= (CONFIG_EN_CRC | CONFIG_CRCO);
    else   cfg &= ~(CONFIG_EN_CRC | CONFIG_CRCO);
    write_reg(hspi, nrf, REG_CONFIG, cfg);
}

static void set_retries(SPI_HandleTypeDef *hspi, NRF24_t *nrf, uint8_t ard, uint8_t arc)
{
    write_reg(hspi, nrf, REG_SETUP_RETR, ((ard & 0x0F) << 4) | (arc & 0x0F));
}

static void enable_dynamic_payloads(SPI_HandleTypeDef *hspi, NRF24_t *nrf, bool en)
{
    uint8_t f = read_reg(hspi, nrf, REG_FEATURE);
    if(en) f |= FEATURE_EN_DPL;
    else   f &= ~FEATURE_EN_DPL;
    write_reg(hspi, nrf, REG_FEATURE, f);

    // solo pipe0
    write_reg(hspi, nrf, REG_DYNPD, en ? 0x01 : 0x00);
}

static void enable_ack_payload(SPI_HandleTypeDef *hspi, NRF24_t *nrf, bool en)
{
    uint8_t f = read_reg(hspi, nrf, REG_FEATURE);
    if(en) f |= FEATURE_EN_ACK_PAY;
    else   f &= ~FEATURE_EN_ACK_PAY;
    write_reg(hspi, nrf, REG_FEATURE, f);
}

static void open_writing_pipe(SPI_HandleTypeDef *hspi, NRF24_t *nrf, const uint8_t addr[5])
{
    memcpy(nrf->addr, addr, 5);
    write_reg_buf(hspi, nrf, REG_TX_ADDR, addr, 5);
    // para autoACK RX_ADDR_P0 debe ser igual a TX_ADDR en PTX
    write_reg_buf(hspi, nrf, REG_RX_ADDR_P0, addr, 5);
}

static void stop_listening(SPI_HandleTypeDef *hspi, NRF24_t *nrf)
{
    ce_low(nrf);

    uint8_t cfg = read_reg(hspi, nrf, REG_CONFIG);
    cfg &= ~CONFIG_PRIM_RX;
    cfg |= CONFIG_PWR_UP;
    write_reg(hspi, nrf, REG_CONFIG, cfg);

    HAL_Delay(2); // tpd2stby
    NRF24_ClearIRQs(hspi, nrf);
}

NRF_Status NRF24_Configure_PTX(SPI_HandleTypeDef *hspi, NRF24_t *nrf,
                               const uint8_t addr[5],
                               uint8_t channel,
                               NRF_Datarate dr,
                               NRF_PaLevel pa)
{
    if(!hspi || !nrf || !addr) return NRF_ERR_BAD_ARG;

    // cconfig recomendada para alcance: 250k, PA max, CRC16 retries fuertes
    set_channel(hspi, nrf, channel);
    set_datarate(hspi, nrf, dr);
    set_pa(hspi, nrf, pa);
    set_crc16(hspi, nrf, true);
    set_retries(hspi, nrf, 5, 15); // 5*250us=1250us delay 15 retriessssssss

    enable_dynamic_payloads(hspi, nrf, true);
    enable_ack_payload(hspi, nrf, true);

    open_writing_pipe(hspi, nrf, addr);

    stop_listening(hspi, nrf);
    return NRF_OK;
}

NRF_Status NRF24_Write(SPI_HandleTypeDef *hspi, NRF24_t *nrf,
                       const void *buf, uint8_t len, uint32_t timeout_ms)
{
    if(!hspi || !nrf || !buf) return NRF_ERR_BAD_ARG;
    if(len == 0 || len > 32) return NRF_ERR_BAD_ARG;

    stop_listening(hspi, nrf);
    NRF24_ClearIRQs(hspi, nrf);

    cmd_rw(hspi, nrf, CMD_W_TX_PAYLOAD, (const uint8_t*)buf, NULL, len);

    // CE pulse >= 10us
    ce_high(nrf);
    for(volatile int i=0;i<350;i++) __NOP();
    ce_low(nrf);

    uint32_t t0 = HAL_GetTick();
    while(HAL_GetTick() - t0 < timeout_ms){
        uint8_t st = get_status(hspi, nrf);

        if(st & STATUS_TX_DS){
            write_reg(hspi, nrf, REG_STATUS, STATUS_TX_DS);
            return NRF_OK;
        }
        if(st & STATUS_MAX_RT){
            write_reg(hspi, nrf, REG_STATUS, STATUS_MAX_RT);
            NRF24_FlushTX(hspi, nrf);
            return NRF_ERR_TIMEOUT;
        }
    }

    NRF24_FlushTX(hspi, nrf);
    return NRF_ERR_TIMEOUT;
}

bool NRF24_AckPayloadAvailable(SPI_HandleTypeDef *hspi, NRF24_t *nrf)
{
    (void)hspi;
    return (get_status(hspi, nrf) & STATUS_RX_DR) != 0;
}

uint8_t NRF24_ReadAckPayload(SPI_HandleTypeDef *hspi, NRF24_t *nrf,
                             void *buf, uint8_t maxlen)
{
    if(!buf || maxlen == 0) return 0;

    uint8_t len = 0;
    cmd_rw(hspi, nrf, CMD_R_RX_PL_WID, NULL, &len, 1);

    if(len == 0 || len > 32){
        NRF24_FlushRX(hspi, nrf);
        write_reg(hspi, nrf, REG_STATUS, STATUS_RX_DR);
        return 0;
    }

    if(len > maxlen) len = maxlen;
    cmd_rw(hspi, nrf, CMD_R_RX_PAYLOAD, NULL, (uint8_t*)buf, len);

    // limpiar RX_DR
    write_reg(hspi, nrf, REG_STATUS, STATUS_RX_DR);
    return len;
}
