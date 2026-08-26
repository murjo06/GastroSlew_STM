#ifndef __SERIAL_H__
#define __SERIAL_H__

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include "usb.h"
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UART_DMA_RX_BUFFER_SIZE     256
#define UART_FRAME_MAX_LENGTH       64

#define SERIAL_START_CHAR			'<'
#define SERIAL_END_CHAR				'>'

typedef HAL_StatusTypeDef (*serial_send_t)(const uint8_t *data, uint16_t length);

typedef struct {
	uint8_t* buffer;
	uint16_t* length;
	serial_send_t print;
} serial_t;

extern serial_t uart_serial;
extern serial_t usb_serial;

void UART_Serial_Init(UART_HandleTypeDef *_huart, volatile bool *ready);

void UART_DMA_Start();

void UART_DMA_Process(uint16_t size);

void handle_serial(serial_t* serial);

uint16_t u64ToHex(uint64_t value, uint8_t *buffer);

uint16_t u64ToDec(uint64_t value, uint8_t *buffer);

#ifdef __cplusplus
}
#endif

#endif