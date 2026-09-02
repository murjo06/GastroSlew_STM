#include "resonant.h"
#include "constants.h"

#include <math.h>

void Resonant_Init(resonant_t *r, float frequency, float damping, float gain, float sampling_period)
{
    float w0 = TWO_PI_F * frequency;
    float k = 2.0f / sampling_period;

    float k2 = k * k;
    float w02 = w0 * w0;
    float two_zeta_w_k = 2.0f * damping * w0 * k;

    float a0 = k2 + two_zeta_w_k + w02;
    float a1 = -2.0f * k2 + 2.0f * w02;
    float a2 = k2 - two_zeta_w_k + w02;

    float b0 = gain * k;

	float a0_inv = 1.0f / a0;

    r->b0 = b0 * a0_inv;
    r->b1 = 0.0f;
    r->b2 = -b0 * a0_inv;

    r->a1 = a1 * a0_inv;
    r->a2 = a2 * a0_inv;
}

float Resonant_GetOutput(resonant_t *r, float input)
{
    float output = r->b0 * input + r->b1 * r->x1 + r->b2 * r->x2 - r->a1 * r->y1 - r->a2 * r->y2;

    r->x2 = r->x1;
    r->x1 = input;

    r->y2 = r->y1;
    r->y1 = output;

    return output;
}

void Resonant_Reset(resonant_t *r)
{
    r->x1 = 0.0f;
    r->x2 = 0.0f;
    r->y1 = 0.0f;
    r->y2 = 0.0f;
}