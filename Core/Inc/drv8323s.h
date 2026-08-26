#ifndef __DRV8323S_H__
#define __DRV8323S_H__

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DRV8323_REG_FAULT_STATUS_1      0x00
#define DRV8323_REG_FAULT_STATUS_2      0x01
#define DRV8323_REG_DRIVER_CONTROL      0x02
#define DRV8323_REG_GATE_HS             0x03
#define DRV8323_REG_GATE_LS             0x04
#define DRV8323_REG_OCP_CONTROL         0x05
#define DRV8323_REG_CSA_CONTROL         0x06

HAL_StatusTypeDef DRV8323_Init(SPI_HandleTypeDef *hspi);

HAL_StatusTypeDef DRV8323_ReadRegister(uint8_t reg, uint16_t *data);

HAL_StatusTypeDef DRV8323_WriteRegister(uint8_t reg, uint16_t data);

//HAL_StatusTypeDef DRV8323_ClearFaults();

#ifdef __cplusplus
}
#endif

#endif