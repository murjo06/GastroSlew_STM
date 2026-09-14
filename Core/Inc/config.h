#pragma once

#include "stm32g4xx_hal.h"

#include <stdint.h>


#define RA


#define TIM_PERIOD					3399

#define DEADTIME_COMPENSATION_GAIN	10.0f		// v 1/A

#define POLE_PAIRS      			7
#define TORQUE_CONSTANT				0.0127f		// K_t = 60 / (2pi * kV), v Nm/A

#define IQ_MAX						4.0f
#define IQ_MAX_RISE					20.0f		// A/s

#define MAX_ACCELERATION			5.0f		// rad/s^2

#define COMPENSATION_CUTOFF			5.0f		// Hz

#define FRICTION_VELOCITY_THRESHOLD	0.3f
#define FRICTION_CUTOFF				5.0f		// Hz

#define PEC_IQ_REF					1.0f

#define ENCODER_ANGLE_OFFSET		1033200

#define VELOCITY_LOOP_PRESCALER     100      	// glede na foc
#define POSITION_LOOP_PRESCALER     240

#define VELOCITY_AVERAGING_INTERVAL 10

#define UVLO        				6.0f

//* pinout

#define CS_GATE_PORT	GPIOA
#define CS_GATE_PIN		GPIO_PIN_15
#define CS_ENC_PORT		GPIOA
#define CS_ENC_PIN		GPIO_PIN_14
#define EN_PORT         GPIOA
#define EN_PIN          GPIO_PIN_13
#define DIR_PORT        GPIOA
#define DIR_PIN         GPIO_PIN_4