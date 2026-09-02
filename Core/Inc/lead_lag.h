#ifndef __LEAD_LAG_H__
#define __LEAD_LAG_H__

typedef struct {
    float prev_error;
    float prev_output;

    float b;
    float b0;
    float b1;
} lead_lag_t;

void LeadLag_Init(lead_lag_t *l, float gain, float crossover_frequency, float phase_shift, float sample_period);

float LeadLag_GetOutput(lead_lag_t *l, float error);

void LeadLag_Reset(lead_lag_t *l);

#endif