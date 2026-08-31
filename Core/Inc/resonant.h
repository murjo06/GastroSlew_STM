#ifndef __RESONANT_H__
#define __RESONANT_H__

typedef struct {
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;

    float x1;
    float x2;
    float y1;
    float y2;
} resonant_t;

void Resonant_Init(resonant_t *r, float frequency, float damping, float gain, float sampling_period);

float Resonant_GetOutput(resonant_t *r, float input);

void Resonant_Reset(resonant_t *r);

#endif