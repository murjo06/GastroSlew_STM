/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "cordic.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "mt6835.h"
#include "drv8323s.h"
#include "config.h"
#include "serial.h"
#include "foc.h"
#include "usb.h"
#include <string.h>
#include "servo.h"
#include "constants.h"
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile bool uart_available = false;
volatile bool usb_available = false;

bool foc_ready = false;

uint32_t encoder_read_cycle = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void DWT_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *_hspi)
{
  if(_hspi == &hspi1) {
    MT6835_Callback();
  }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  FOC_ADC_Callback(hadc);
}

bool fault = false;

float v_d = 0.0f;

uint8_t foc_loop_counter = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_ADC4_Init();
  MX_CORDIC_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_USB_Device_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  __HAL_TIM_MOE_DISABLE(&htim1);  // low timer outputi

  DWT_Init();

  MT6835_Init(&hspi1);

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc4, ADC_SINGLE_ENDED);

  UART_Serial_Init(&huart2, &uart_available);

  USB_Serial_Init(&usb_available);

  Servo_Init();

  CORDIC_SetRotationMode();

  HAL_GPIO_WritePin(CS_GATE_PORT, CS_GATE_PIN, GPIO_PIN_SET);

  //while(!HAL_GPIO_ReadPin(EN_PORT, EN_PIN)) {}

  HAL_Delay(50);

  DRV8323_Init(&hspi1);

  HAL_Delay(50);

  HAL_SPI_DeInit(&hspi1);
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_Delay(50);

  #ifdef RA
  FOC_Init(&hadc1, &hadc3, &hadc2);
  #else
  FOC_Init(&hadc2, &hadc1, &hadc3);
  #endif

  HAL_ADCEx_InjectedStart_IT(&hadc1);
  HAL_ADCEx_InjectedStart_IT(&hadc2);
  HAL_ADCEx_InjectedStart_IT(&hadc3);

  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;
  TIM1->CCR3 = 0;

  HAL_TIM_Base_Start_IT(&htim1);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;
  TIM1->CCR3 = 0;

  foc_ready = true;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //uint32_t millis = HAL_GetTick();
  uint32_t millis = DWT->CYCCNT >> 17;

  while (1) {
  
    if((DWT->CYCCNT >> 17) - millis > 200) {
      
      millis = DWT->CYCCNT >> 17;

      fault = !((bool)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6));
    
      uint32_t cycle = (DWT->CYCCNT) >> 7;

      //FOC_Loop();

      //MT6835_FetchAngle();

      uint32_t diff = ((DWT->CYCCNT) >> 7) - cycle;

      //uint8_t b[32];
      //memset(b, 0, 32);
      //uint16_t len = u64ToHex(MT6835_GetRawAngle(), b);
      //uint16_t len = u64ToDec((uint32_t)(10000.0f + 1000.0f * v_d), b);
      //b[len] = '\n';
      //b[len + 1] = '\0';
      if(!fault) {
        //usb_serial.print(b, 32);
      }

      if(fault) {
        HAL_SPI_DeInit(&hspi1);
        hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
        hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
        if (HAL_SPI_Init(&hspi1) != HAL_OK)
        {
          Error_Handler();
        }

        HAL_Delay(50);

        uint16_t fault1 = 0;
        DRV8323_ReadRegister(DRV8323_REG_GATE_HS, &fault1);
        uint16_t fault2 = 0;
        DRV8323_ReadRegister(DRV8323_REG_GATE_LS, &fault2);
        uint32_t fault_regs = ((uint32_t)fault1 << 16) | (uint32_t)fault2;

        uint8_t c[32];
        memset(c, 0, 32);
        uint16_t lenc = u64ToHex(fault_regs, c);
        c[lenc] = 'x';
        c[lenc + 1] = '\n';
        c[lenc + 2] = '\0'; 
        usb_serial.print(c, 32);

        HAL_SPI_DeInit(&hspi1);
        hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
        hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
        if (HAL_SPI_Init(&hspi1) != HAL_OK)
        {
          Error_Handler();
        }

        HAL_Delay(50);
      }

      

	    if(uart_available) {
	    	handle_serial(&uart_serial);
	    }
	    if(usb_available) {
	    	handle_serial(&usb_serial);
	    }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM1 || fault || !foc_ready) {
    return;
  }

  if(htim->Instance->CR1 & TIM_CR1_DIR) {   // štetje dol, sredina pwm-ja
    if(foc_loop_counter == FOC_LOOP_PRESCALER - 1) {
      encoder_read_cycle = DWT->CYCCNT;
      MT6835_FetchAngle();
    }
  } else {
    if(++foc_loop_counter >= FOC_LOOP_PRESCALER) {
      foc_loop_counter = 0;
      FOC_Loop();
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while(1) {}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
