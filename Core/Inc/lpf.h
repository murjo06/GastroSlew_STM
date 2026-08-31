#ifndef __LPF_H__
#define __LPF_H__

typedef struct {
	float alpha;

	float prev_output;
} lpf_t;

void LPF_Init(lpf_t *l, float cutoff_frequency, float sample_period);

float LPF_GetOutput(lpf_t *l, float input);

void LPF_Reset(lpf_t *l);

#endif