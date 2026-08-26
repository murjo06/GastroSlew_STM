#include "stm32g4xx_hal.h"
#include <stdbool.h>

void USB_Serial_Init(volatile bool *ready);

HAL_StatusTypeDef USB_Serial_Print(const uint8_t *data, uint16_t length);

void USB_Serial_TxComplete(void);