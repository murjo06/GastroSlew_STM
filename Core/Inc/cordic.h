/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    cordic.h
  * @brief   This file contains all the function prototypes for
  *          the cordic.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CORDIC_H__
#define __CORDIC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CORDIC_HandleTypeDef hcordic;

/* USER CODE BEGIN Private defines */

#define Q31_SCALE_F           2147483648.0f     // 2^31
#define Q31_SCALE_INVERSE_F   4.656612873e-10f  // 2^-31
#define Q31_TO_RAD_F          1.462918079e-9f   // pi / 2^31
#define RAD_TO_Q31_F          6.835652755e8f    // 2^31 / pi

/* USER CODE END Private defines */

void MX_CORDIC_Init(void);

/* USER CODE BEGIN Prototypes */

void CORDIC_SetRotationMode(void);

void CORDIC_SetPhaseMode(void);

void CORDIC_SinCos(int32_t angle_q31, int32_t *sin_q31, int32_t *cos_q31);

int32_t CORDIC_Atan2(int32_t y_q31, int32_t x_q31);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CORDIC_H__ */

