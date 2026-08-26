#include "pid.h"
#include "stm32g4xx_hal.h"

void Pid_Init(pid_t *p, uint8_t components)
{
    p->cycle = 0;
    p->prev_cycle = 0;

    p->error = 0.0f;
    p->dt = 0.0f;
    p->integral = 0.0f;
    p->derivative_cycle = 0;
    p->prev_derivative = 0.0f;
    p->prev_derivative_error = 0.0f;
    p->prev_derivative_cycle = 0;

    p->kp = 0.0f;
    p->ki = 0.0f;
    p->kd = 0.0f;

    p->components = components;

    p->derivative_interval = 1;
}

float get_pid_output(pid_t *p, float _error)
{
    p->error = _error;
    float output = 0.0f;
    if(p->components & 0b100) {
        output += calculate_p(p);
    }
    p->cycle = DWT->CYCCNT;
    p->dt = (float)(p->cycle - p->prev_cycle) * 5.88235294e-9f;
    if(p->components & 0b010) {
        output += calculate_i(p);
    }
    if(p->components & 0b001) {
        output += calculate_d(p);
    }
    p->prev_cycle = p->cycle;
    return output;
}

float calculate_p(pid_t *p)
{
    return p->kp * p->error;
}

float calculate_i(pid_t *p)
{
    p->integral += p->dt * p->error;
    return p->ki * p->integral;
}

float calculate_d(pid_t *p)      //* optimizacija?
{
    if(++(p->derivative_cycle) < p->derivative_interval) {
        return p->kd * p->prev_derivative;
    }
    p->derivative_cycle = 0;
    p->prev_derivative = 170e6f * (p->error - p->prev_derivative_error) / (float)(p->cycle - p->prev_derivative_cycle);
    p->prev_derivative_cycle = p->cycle;
    p->prev_derivative_error = p->error;
    return p->kd * p->prev_derivative;
}

void reset_pid(pid_t *p)
{
    p->prev_cycle = DWT->CYCCNT;
    p->prev_derivative_cycle = p->prev_cycle;
    p->prev_derivative = 0.0f;
    p->derivative_cycle = 0;
    p->integral = 0.0f;
}