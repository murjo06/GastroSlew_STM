#include "serial.h"
#include "foc.h"
#include "servo.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"

#include <stdint.h>

UART_HandleTypeDef *huart;
uint8_t dma_buffer[UART_DMA_RX_BUFFER_SIZE];
uint8_t frame_buffer[UART_FRAME_MAX_LENGTH];
uint16_t frame_index;
uint16_t frame_length;
bool rx_busy;
bool tx_busy;
volatile bool *frame_ready;

serial_t uart_serial;
serial_t usb_serial;

static uint8_t serial_tx_buffer[UART_FRAME_MAX_LENGTH];

static uint8_t uart_tx_buffer[UART_FRAME_MAX_LENGTH];

static const uint8_t hex_chars[] = "0123456789abcdef";

void UART_DMA_Start()
{
    HAL_UARTEx_ReceiveToIdle_DMA(huart, dma_buffer, UART_DMA_RX_BUFFER_SIZE);

    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

void UART_DMA_Process(uint16_t size)
{
    for(uint16_t i = 0; i < size; i++) {
        uint8_t byte = dma_buffer[i];

        if(byte == SERIAL_START_CHAR) {
            tx_busy = true;
            frame_index = 0;
            continue;
        } else if(byte == SERIAL_END_CHAR) {
            if(tx_busy) {
                *frame_ready = true;
            }
            tx_busy = false;
            frame_length = frame_index;
            frame_index = 0;
            continue;
        }
        if(tx_busy) {
            if(frame_index < UART_FRAME_MAX_LENGTH) {
                frame_buffer[frame_index++] = byte;
            } else {
                tx_busy = false;
                frame_index = 0;
                frame_length = 0;
            }
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *_huart, uint16_t size)
{
    if(_huart == huart) {
        UART_DMA_Process(size);

        HAL_UARTEx_ReceiveToIdle_DMA(huart, dma_buffer, UART_DMA_RX_BUFFER_SIZE);

        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}

HAL_StatusTypeDef UART_Serial_Print(const uint8_t *data, uint16_t length)
{
	if(length == 0) {
		return HAL_BUSY;
	}
	memcpy(uart_tx_buffer, data, length);
    if(tx_busy) {
        return HAL_BUSY;
    }

    tx_busy = true;

    return HAL_UART_Transmit_DMA(huart, data, length);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *_huart)
{
    if(_huart == huart) {
        tx_busy = false;
    }
}

void UART_Serial_Init(UART_HandleTypeDef *_huart, volatile bool *ready)
{
	uart_serial.buffer = frame_buffer;
	uart_serial.length = &frame_length;
	uart_serial.print = UART_Serial_Print;
    huart = _huart;
	frame_ready = ready;

    frame_index = 0;
    tx_busy = false;
}

uint64_t hexToU64(uint8_t *data, int length)
{
    if(length < 0) {
        return 0;
    }
    uint64_t value = 0;
    for (size_t i = 0; i < length; ++i) {
        uint8_t digit;
        if (data[i] >= '0' && data[i] <= '9')
            digit = data[i] - '0';
        else if (data[i] >= 'A' && data[i] <= 'F')
            digit = data[i] + 10 - 'A';
        else if (data[i] >= 'a' && data[i] <= 'f')
            digit = data[i] + 10 - 'a';
        else
            return 0;
        value = (value << 4) | digit;
    }
    return value;
}

uint16_t u64ToHex(uint64_t value, uint8_t *buffer)
{
    uint16_t pos = 0;
    uint16_t started = 0;
    for(int i = 15; i >= 0; i--) {
        uint8_t digit = (value >> (i * 4)) & 0xF;
        if(digit != 0 || started) {
            buffer[pos++] = hex_chars[digit];
            started = 1;
        }
    }
    if(!started) {
        buffer[pos++] = '0';
    }
	return pos;
}

uint16_t u64ToDec(uint64_t value, uint8_t *buffer)
{
    uint8_t temp[20];
    uint16_t digits = 0;

    do {
        uint64_t q = value / 10;
        temp[digits++] = (uint8_t)('0' + (value - q * 10));
        value = q;
    } while(value);

    for(uint16_t i = 0; i < digits; i++) {
        buffer[i] = temp[digits - 1 - i];
    }

    return digits + 1;
}

static uint16_t u32ToHexLeadingZero(uint32_t value, uint8_t *buffer)
{
    for(int i = 7; i >= 0; i--) {
        uint8_t digit = (value >> (i * 4)) & 0xF;
        buffer[7 - i] = hex_chars[digit];
    }

    return 8;
}


static bool read_float_bytes(float *dest, uint8_t *buffer, uint16_t length)
{
	if(length != 8) {
		return false;
	}
	uint32_t float_buffer = (uint32_t)hexToU64(buffer, length);
	memcpy(dest, &float_buffer, sizeof(float));

	return true;
}

static void write_float_bytes(float *source, uint8_t *buffer, uint16_t *length)
{
	uint32_t float_buffer = 0;
	memcpy(&float_buffer, source, sizeof(float));
	u32ToHexLeadingZero(float_buffer, buffer);
	buffer[8] = SERIAL_END_CHAR;
	buffer[9] = '\0';
	*length = 10;
}

void handle_serial(serial_t *s)		// s->buffer ne sme imet <>
{
    if(*(s->length) < 2) {
        return;
    }
	memset(serial_tx_buffer, 0, UART_FRAME_MAX_LENGTH);
    serial_tx_buffer[0] = SERIAL_START_CHAR;
	uint16_t length = 0;

	switch(s->buffer[0]) {
		case 'P': {			//* komandiarana lega
            position_target = (int64_t)((hexToU64(s->buffer + 1, *(s->length) - 1)));
			break;
		} case 'V': {		//* komandirana hitrost
			read_float_bytes(&velocity_target, s->buffer + 1, *(s->length) - 1);
			break;
		} case 'S': {		//* sinhroniziraj
            position = (int64_t)((hexToU64(s->buffer + 1, *(s->length) - 1)));
			break;
		} case 'M': {		//* dovoljena moč
			read_float_bytes(&maximum_power, s->buffer + 1, *(s->length) - 1);
			break;
		} case 'C': {		//* pid
			switch(s->buffer[1]) {
                case 'P': {			//* pozicija
                    switch(s->buffer[2]) {
                        case 'P': {		//* p
							read_float_bytes(&(position_pid.kp), s->buffer + 3, *(s->length) - 3);
		                	break;
		                } case 'I': {	//* i
                         	read_float_bytes(&(position_pid.ki), s->buffer + 3, *(s->length) - 3);
		                	break;
		                } case 'D': {	//* d
                         	read_float_bytes(&(position_pid.kd), s->buffer + 3, *(s->length) - 3);
		                	break;
		                } default: return;
                    }
					break;
                } case 'V': {		//* hitrost
					switch(s->buffer[2]) {
                        case 'P': {		//* p
                         	read_float_bytes(&(velocity_pid.kp), s->buffer + 3, *(s->length) - 3);
		                	break;
		                } case 'I': {	//* i
                         	read_float_bytes(&(velocity_pid.ki), s->buffer + 3, *(s->length) - 3);
		                	break;
		                } default: return;
                    }
					break;
                } case 'D': {		//* d
					switch(s->buffer[2]) {
                        case 'P': {		//* p
                         	read_float_bytes(&(d_pid.kp), s->buffer + 3, *(s->length) - 3);
		                	break;
		                } case 'I': {	//* i
                         	read_float_bytes(&(d_pid.ki), s->buffer + 3, *(s->length) - 3);
		                	break;
		                } default: return;
                    }
					break;
                } case 'Q': {		//* q
					switch(s->buffer[2]) {
                        case 'P': {		//* p
                         	read_float_bytes(&(q_pid.kp), s->buffer + 3, *(s->length) - 3);
		                	break;
		                } case 'I': {	//* i
                         	read_float_bytes(&(q_pid.ki), s->buffer + 3, *(s->length) - 3);
		                	break;
		                } default: return;
                    }
					break;
                } default: return;
            }
			break;
		}
		case 'p': {			//* komandirana lega
            u64ToHex(position_target, serial_tx_buffer + 1);
			serial_tx_buffer[10] = SERIAL_END_CHAR;
			serial_tx_buffer[11] = '\0';
			length = 11;
			break;
		} case 'v': {		//* komandirana hitrost
			write_float_bytes(&velocity_target, serial_tx_buffer + 1, &length);
			break;
		} case 's': {		//* trenutna pozicija
            u64ToHex(position, serial_tx_buffer + 1);
			serial_tx_buffer[10] = SERIAL_END_CHAR;
			serial_tx_buffer[11] = '\0';
			length = 11;
			break;
		} case 'm': {		//* dovoljena moč
			write_float_bytes(&maximum_power, serial_tx_buffer + 1, &length);
			break;
		} case 'c': {       //* pid
            switch(s->buffer[1]) {
                case 'P': {			//* pozicija
                    switch(s->buffer[2]) {
                        case 'P': {		//* p
                         	write_float_bytes(&(position_pid.kp), serial_tx_buffer + 1, &length);
		                	break;
		                } case 'I': {	//* i
                         	write_float_bytes(&(position_pid.ki), serial_tx_buffer + 1, &length);
		                	break;
		                } case 'D': {	//* d
                         	write_float_bytes(&(position_pid.kd), serial_tx_buffer + 1, &length);
		                	break;
		                } default: return;
                    }
					break;
                } case 'V': {		//* hitrost
					switch(s->buffer[2]) {
                        case 'P': {		//* p
                         	write_float_bytes(&(velocity_pid.kp), serial_tx_buffer + 1, &length);
		                	break;
		                } case 'I': {	//* i
                         	write_float_bytes(&(velocity_pid.ki), serial_tx_buffer + 1, &length);
		                	break;
		                } default: return;
                    }
					break;
                } case 'D': {		//* d
					switch(s->buffer[2]) {
                        case 'P': {		//* p
                         	write_float_bytes(&(d_pid.kp), serial_tx_buffer + 1, &length);
		                	break;
		                } case 'I': {	//* i
                         	write_float_bytes(&(d_pid.ki), serial_tx_buffer + 1, &length);
		                	break;
		                } default: return;
                    }
					break;
                } case 'Q': {		//* q
					switch(s->buffer[2]) {
                        case 'P': {		//* p
                         	write_float_bytes(&(q_pid.kp), serial_tx_buffer + 1, &length);
		                	break;
		                } case 'I': {	//* i
                         	write_float_bytes(&(q_pid.ki), serial_tx_buffer + 1, &length);
		                	break;
		                } default: return;
                    }
					break;
                } default: return;
            }
			break;
		} case 'a': {		//* ack
			serial_tx_buffer[1] = 'a';
			serial_tx_buffer[2] = SERIAL_END_CHAR;
			serial_tx_buffer[3] = '\0';
			length = 3;
			break;
		} default: return;
	}
	if(length != 0) {
		s->print(serial_tx_buffer, length);
	}
	*(s->length) = 0;
}