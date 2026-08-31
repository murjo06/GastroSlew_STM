#ifndef __PID_H__
#define __PID_H__

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define PID_PROPORTIONAL    0b100
#define PID_INTEGRAL        0b010
#define PID_DERIVATIVE      0b001

typedef struct {
    float kp;
    float ki;
    float kd;

    float integral;
    float dt;

    uint32_t prev_cycle;

    uint16_t derivative_interval;
    uint16_t derivative_series;
    uint32_t prev_derivative_cycle;

    float prev_derivative;
    float prev_derivative_error;
} pid_t;

void PID_Init(pid_t *p, float kp, float ki, float kd);

float PID_GetOutput(pid_t *p, float error);

void PID_Reset(pid_t *p);

#endif