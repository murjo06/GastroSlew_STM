#ifndef __MT6835_H__
#define __MT6835_H__

#include "stm32g4xx_hal.h"

#include <stdint.h>
#include <stdbool.h>

#define MT6835_RAW_TO_RAD_F 2.99605622e-6f

void MT6835_Init(SPI_HandleTypeDef *_hspi);

HAL_StatusTypeDef MT6835_FetchAngle(void);
HAL_StatusTypeDef MT6835_FetchAngleSync(void);

bool MT6835_DataAvailable(void);

void MT6835_Callback(void);

int32_t MT6835_GetRawAngle(void);
float MT6835_GetAngle(void);

#endif