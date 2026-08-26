#include "drv8323s.h"
#include "config.h"
#include <string.h>

static SPI_HandleTypeDef *hspi;

static inline void CS_Delay(void)
{
    for(volatile uint32_t i = 0; i < 100; i++) {
        __NOP();
    }
}

static inline void CS_Low(void)
{
    CS_GATE_PORT->BRR = (uint32_t)CS_GATE_PIN;
    //HAL_GPIO_WritePin(CS_GATE_PORT, CS_GATE_PIN, GPIO_PIN_RESET);
    CS_Delay();
}

static inline void CS_High(void)
{
    CS_Delay();
    CS_GATE_PORT->BSRR = (uint32_t)CS_GATE_PIN;
    //HAL_GPIO_WritePin(CS_GATE_PORT, CS_GATE_PIN, GPIO_PIN_SET);
}

HAL_StatusTypeDef DRV8323_ReadRegister(uint8_t reg, uint16_t *data)
{
    uint8_t rx[2];
    uint8_t tx[2];
    tx[0] = 0x80 | (reg & 0x7) << 3;
    tx[1] = 0;

    memset(rx, 0, 2);
	
	CS_Low();

	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, tx, rx, 2, 1000);

	CS_High();

    *data = (((uint16_t)rx[0] << 8) | rx[1]) & 0x7ff;

	return status;
}

HAL_StatusTypeDef DRV8323_WriteRegister(uint8_t reg, uint16_t data)
{
    uint8_t tx[2];
    tx[0] = (((reg & 0x7) << 11) | (data & 0x7ff)) >> 8;
    tx[1] = data & 0xff;
	
	CS_Low();

	HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, tx, 2, 1000);

	CS_High();

	return status;
}

HAL_StatusTypeDef DRV8323_Init(SPI_HandleTypeDef *_hspi)
{
    hspi = _hspi;

    CS_High();

    return 1;

    uint16_t control_reg = 0;
    uint16_t control_reg_verify = 0;
    DRV8323_ReadRegister(DRV8323_REG_GATE_LS, &control_reg);
	control_reg &= (0b1U << 10);

	control_reg |= 0b1U;		        // Reset fault
    control_reg |= (0b00U << 5);        // 6x PWM mode
    control_reg |= (0b1U << 7);	        // OTW on FAULT enabled
    control_reg |= (0b0U << 8);	        // Gate drive fault enabled
	control_reg |= (0b0U << 9);	        // Charge pump UVLO fault enabled
    DRV8323_WriteRegister(DRV8323_REG_DRIVER_CONTROL, control_reg);

	uint16_t ls_reg = 0;
    uint16_t ls_reg_verify = 0;
	ls_reg |= 0b1010U;			// 740 mA sink
	ls_reg |= (0b1100U << 4);	// 570 mA source
	ls_reg |= (0b11U << 8);		// 1000 ns max čas
	ls_reg |= (0b1U << 10);		// PWM resetira OCP
	DRV8323_WriteRegister(DRV8323_REG_GATE_LS, ls_reg);

	uint16_t ocp_reg = 0;
    uint16_t ocp_reg_verify = 0;
	ocp_reg |= 0b1001U;			// 0.75 V napetost na FETu
	ocp_reg |= (0b01U << 4);	// 4 us overcurrent deglitch
	ocp_reg |= (0b00U << 6);	// latched fault
	ocp_reg |= (0b01U << 8);	// 100 ns dead time
	ocp_reg |= (0b1U << 10);	// 50 us ponovni poskus
	DRV8323_WriteRegister(DRV8323_REG_OCP_CONTROL, ocp_reg);

    uint16_t hs_reg = 0;
    uint16_t hs_reg_verify = 0;
	hs_reg |= 0b1010U;			// 740 mA sink
	hs_reg |= (0b1100U << 4);	// 570 mA source
	hs_reg |= (0b110U << 8);	// lock izklopljen
	DRV8323_WriteRegister(DRV8323_REG_GATE_HS, hs_reg);

    uint16_t fault_1 = 0;
    DRV8323_ReadRegister(DRV8323_REG_FAULT_STATUS_1, &fault_1);

    uint16_t fault_2 = 0;
    DRV8323_ReadRegister(DRV8323_REG_FAULT_STATUS_2, &fault_2);

    bool success = ((control_reg | 0b1U) == (control_reg_verify | 0b1U)) &&
        (hs_reg == hs_reg_verify) &&
        (ls_reg == ls_reg_verify) &&
        (ocp_reg == ocp_reg_verify);

    return HAL_OK;
}