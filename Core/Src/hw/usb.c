#include "serial.h"
#include "foc.h"
#include "servo.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"

uint8_t frame_buffer_usb[UART_FRAME_MAX_LENGTH];
uint16_t frame_index_usb;
uint16_t frame_length_usb;

volatile bool rx_busy_usb;
volatile bool tx_busy_usb;

volatile bool *frame_ready_usb;

volatile bool *usb_transmitting;

static uint8_t usb_tx_buffer[UART_FRAME_MAX_LENGTH];

void USB_CDC_Process(uint8_t *data, uint16_t size)
{
    for(uint16_t i = 0; i < size; i++)
    {
        uint8_t byte = data[i];

        if(byte == SERIAL_START_CHAR)
        {
            rx_busy_usb = true;
            frame_index_usb = 0;
            continue;
        }
        else if(byte == SERIAL_END_CHAR || byte == '\0')
        {
            if(rx_busy_usb)
            {
                frame_length_usb = frame_index_usb;
                *frame_ready_usb = true;
            }

            rx_busy_usb = false;
            frame_index_usb = 0;
            continue;
        }

        if(rx_busy_usb)
        {
            if(frame_index_usb < UART_FRAME_MAX_LENGTH)
            {
                frame_buffer_usb[frame_index_usb++] = byte;
            }
            else
            {
                rx_busy_usb = false;
                frame_index_usb = 0;
                frame_length_usb = 0;
            }
        }
    }
}

HAL_StatusTypeDef USB_Serial_Print(const uint8_t *data, uint16_t length)
{
    memcpy(usb_tx_buffer, data, length);
    if(tx_busy_usb) {
        return HAL_BUSY;
    }
    if(CDC_Transmit_FS((uint8_t *)data, length) != USBD_OK) {
        return HAL_BUSY;
    }
    tx_busy_usb = true;
    return HAL_OK;
}

void USB_Serial_Init(volatile bool *ready)
{
    usb_serial.buffer = frame_buffer_usb;
    usb_serial.length = &frame_length_usb;
    usb_serial.print = USB_Serial_Print;
    
    frame_ready_usb = ready;
    frame_index_usb = 0;
    frame_length_usb = 0;
    rx_busy_usb = false;
    tx_busy_usb = false;

    uint32_t cy = DWT->CYCCNT >> 17;
    while((DWT->CYCCNT >> 17) - cy < 1000) {}
}

void USB_Serial_TxComplete(void)
{
    tx_busy_usb = false;
}