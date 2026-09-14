#include "mt6835.h"
#include "config.h"
#include <string.h>
#include "spi.h"

#define MT6835_TIMEOUT      10

volatile static int32_t angle_raw = 0;

static uint8_t tx[6];
static uint8_t rx[6];

bool mt_active = false;

volatile bool mt_available = true;

SPI_HandleTypeDef *hspi;

static inline void CS_Low(void)
{
    CS_ENC_PORT->BRR = (uint32_t)CS_ENC_PIN;
    for(int i = 0; i < 20; i++) {
        __NOP();
    }
}

static inline void CS_High(void)
{
    for(int i = 0; i < 10; i++) {
        __NOP();
    }
    CS_ENC_PORT->BSRR = (uint32_t)CS_ENC_PIN;
}

void MT6835_Init(SPI_HandleTypeDef *_hspi)
{
    hspi = _hspi;

    CS_High();

    memset(tx, 0xFF, 6);
    tx[0] = 0b1010 << 4;
    tx[1] = 0x3;

    mt_active = true;
}

static uint8_t MT6835_CRC8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;

    for(uint8_t i = 0; i < len; i++) {
        crc ^= data[i];

        for(uint8_t bit = 0; bit < 8; bit++) {
            if(crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

static bool MT6835_CheckCRC(void)
{
    uint8_t calculated_crc = MT6835_CRC8(&rx[2], 3);

    return calculated_crc == rx[5];
}

HAL_StatusTypeDef MT6835_FetchAngle(void)
{
    if(!mt_available) {
        return HAL_BUSY;
    }
    mt_available = false;

    CS_Low();

    return HAL_SPI_TransmitReceive_DMA(hspi, tx, rx, 6);
}

void MT6835_Callback(void)
{
    if(!mt_active) {
        return;
    }

    mt_available = true;

    if(MT6835_CheckCRC()) {
        angle_raw = ((int32_t)rx[2] << 13) | ((int32_t)rx[3] << 5) | ((int32_t)rx[4] >> 3);
    }

    CS_High();
}

HAL_StatusTypeDef MT6835_FetchAngleSync(void)
{
    if(!mt_available) {
        return HAL_BUSY;
    }
    mt_available = false;

    CS_Low();

    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, tx, rx, 6, MT6835_TIMEOUT);

    if(status != HAL_OK) {
        return status;
    }

    if(MT6835_CheckCRC()) {
        angle_raw = ((int32_t)rx[2] << 13) | ((int32_t)rx[3] << 5)  | ((int32_t)rx[4] >> 3);
    } else {
        return HAL_ERROR;
    }

    CS_High();

    mt_available = true;

    return status;
}

bool MT6835_DataAvailable(void)
{
    return mt_available;
}

int32_t MT6835_GetRawAngle(void)
{
    return angle_raw;
}

float MT6835_GetAngle(void)
{
    return MT6835_RAW_TO_RAD_F * (float)angle_raw;
}