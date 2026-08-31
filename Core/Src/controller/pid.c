#include "pid.h"
#include "stm32g4xx_hal.h"

void PID_Init(pid_t *p, float kp, float ki, float kd)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;

    p->derivative_interval = 1;

    PID_Reset(p);
}

static float calculate_p(pid_t *p, float error)
{
    return p->kp * error;
}

static float calculate_i(pid_t *p, float error)
{
    p->integral += p->dt * error;
    return p->ki * p->integral;
}

static float calculate_d(pid_t *p, float error, uint32_t cycle)
{
    if(++(p->derivative_series) < p->derivative_interval) {
        return p->kd * p->prev_derivative;
    }
    p->derivative_series = 0;

    p->prev_derivative = 170e6f * (error - p->prev_derivative_error) / (float)(cycle - p->prev_derivative_cycle);

    p->prev_derivative_cycle = cycle;
    p->prev_derivative_error = error;

    return p->kd * p->prev_derivative;
}

float PID_GetOutput(pid_t *p, float error)
{
    float output = 0.0f;
    if(p->kp != 0.0f) {
        output += calculate_p(p, error);
    }
    uint32_t cycle = DWT->CYCCNT;
    p->dt = (float)(cycle - p->prev_cycle) * 5.88235294e-9f;
    if(p->ki != 0.0f) {
        output += calculate_i(p, error);
    }
    if(p->kd != 0.0f) {
        output += calculate_d(p, error, cycle);
    }
    p->prev_cycle = cycle;
    return output;
}

void PID_Reset(pid_t *p)
{
    p->prev_cycle = DWT->CYCCNT;
    p->prev_derivative_cycle = p->prev_cycle;

    p->prev_derivative = 0.0f;
    p->derivative_series = 0;

    p->dt = 0.0f;
    p->integral = 0.0f;

    p->prev_derivative = 0.0f;
    p->prev_derivative_error = 0.0f;
}