#include "lpf.h"
#include "constants.h"

void LPF_Init(lpf_t *l, float cutoff_frequency, float sample_period)
{
	float time_constant = 1.0f / (TWO_PI_F * cutoff_frequency);

	l->alpha = sample_period / (time_constant + sample_period);

	LPF_Reset(l);
}

float LPF_GetOutput(lpf_t *l, float input)
{
	float output = l->prev_output + l->alpha * (input - l->prev_output);
	l->prev_output = output;
	return output;
}

void LPF_Reset(lpf_t *l)
{
	l->prev_output = 0.0f;
}