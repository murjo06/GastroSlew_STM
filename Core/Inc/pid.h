#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

typedef struct {
    uint32_t cycle;
    uint32_t prev_cycle;

    float error;
    float dt;
    float integral;
    uint8_t derivative_cycle;
    float prev_derivative;
    float prev_derivative_error;
    uint64_t prev_derivative_cycle;

    float kp;
    float ki;
    float kd;

    uint8_t components;     // p,i,d

    uint8_t derivative_interval;
} pid_t;

void Pid_Init(pid_t *p, uint8_t components);

float get_pid_output(pid_t *p, float _error);

float calculate_p(pid_t *p);

float calculate_i(pid_t *p);

float calculate_d(pid_t *p);

void reset_pid(pid_t *p);