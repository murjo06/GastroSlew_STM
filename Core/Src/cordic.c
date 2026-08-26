/* USER CODE BEGIN Header */

/**
  ******************************************************************************
  * @file    cordic.c
  * @brief   This file provides code for the configuration
  *          of the CORDIC instances.
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
/* Includes ------------------------------------------------------------------*/
#include "cordic.h"

/* USER CODE BEGIN 0 */

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_cordic.h"

/* USER CODE END 0 */

CORDIC_HandleTypeDef hcordic;

/* CORDIC init function */
void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */

  /* USER CODE END CORDIC_Init 2 */

}

void HAL_CORDIC_MspInit(CORDIC_HandleTypeDef* cordicHandle)
{

  if(cordicHandle->Instance==CORDIC)
  {
  /* USER CODE BEGIN CORDIC_MspInit 0 */

  /* USER CODE END CORDIC_MspInit 0 */
    /* CORDIC clock enable */
    __HAL_RCC_CORDIC_CLK_ENABLE();
  /* USER CODE BEGIN CORDIC_MspInit 1 */

  /* USER CODE END CORDIC_MspInit 1 */
  }
}

void HAL_CORDIC_MspDeInit(CORDIC_HandleTypeDef* cordicHandle)
{

  if(cordicHandle->Instance==CORDIC)
  {
  /* USER CODE BEGIN CORDIC_MspDeInit 0 */

  /* USER CODE END CORDIC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CORDIC_CLK_DISABLE();
  /* USER CODE BEGIN CORDIC_MspDeInit 1 */

  /* USER CODE END CORDIC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void CORDIC_SetRotationMode(void)
{
  CORDIC_ConfigTypeDef cfg =
  {
    .Function = CORDIC_FUNCTION_COSINE,
    .Precision = CORDIC_PRECISION_6CYCLES,
    .Scale = CORDIC_SCALE_0,
    .NbWrite = CORDIC_NBWRITE_1,
    .NbRead = CORDIC_NBREAD_2,
    .InSize = CORDIC_INSIZE_32BITS,
    .OutSize = CORDIC_OUTSIZE_32BITS
  };

  HAL_CORDIC_Configure(&hcordic, &cfg);
}

void CORDIC_SetPhaseMode(void)
{
  CORDIC_ConfigTypeDef cfg =
  {
    .Function = CORDIC_FUNCTION_PHASE,
    .Precision = CORDIC_PRECISION_6CYCLES,
    .Scale = CORDIC_SCALE_0,
    .NbWrite = CORDIC_NBWRITE_2,
    .NbRead = CORDIC_NBREAD_1,
    .InSize = CORDIC_INSIZE_32BITS,
    .OutSize = CORDIC_OUTSIZE_32BITS
  };

  HAL_CORDIC_Configure(&hcordic, &cfg);
}

static inline int32_t float_to_q31(float x)
{
  if (x >= 1.0f)
    return INT32_MAX;

  if (x <= -1.0f)
    return INT32_MIN;

  return (int32_t)(x * Q31_SCALE_F);
}

static inline float q31_to_float(int32_t x)
{
  return (float)x / Q31_SCALE_F;
}

static inline void CORDIC_Write(int32_t value)
{
  while ((CORDIC->CSR & CORDIC_CSR_RRDY) != 0) {}
  CORDIC->WDATA = (uint32_t)value;
}

static inline int32_t CORDIC_Read(void)
{
  while ((CORDIC->CSR & CORDIC_CSR_RRDY) == 0) {}
  return (int32_t)CORDIC->RDATA;
}

void CORDIC_SinCos(int32_t angle_q31, int32_t *sin_q31, int32_t *cos_q31)
{
  CORDIC_Write(angle_q31);

  *cos_q31 = CORDIC_Read();
  *sin_q31 = CORDIC_Read();
}

int32_t CORDIC_Atan2(int32_t y_q31, int32_t x_q31)
{
  CORDIC_Write(x_q31);
  CORDIC_Write(y_q31);

  return CORDIC_Read();
}

/* USER CODE END 1 */

