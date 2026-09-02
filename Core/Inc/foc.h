#ifndef __FOC_H__
#define __FOC_H__

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "adc.h"
#include "pid.h"
#include "config.h"


extern float powers[VELOCITY_LOOP_PRESCALER];
extern float iqs[VELOCITY_LOOP_PRESCALER];



extern float iq_target;




extern float delta;
extern bool iq_saturated;

extern float iq_compensation;

extern float setpoint_angle;
extern float setpoint_velocity;

extern float electrical_angle;

extern pid_t d_pid;
extern pid_t q_pid;

extern float v_bus;
extern float angle_offset;
extern float maximum_power;

extern volatile bool enabled;

extern const float micro_multiplier;

void FOC_Init(ADC_HandleTypeDef *_hadcA, ADC_HandleTypeDef *_hadcB, ADC_HandleTypeDef *_hadcC);

void FOC_ADC_Callback(ADC_HandleTypeDef *hadc);

void FOC_Loop();

#endif