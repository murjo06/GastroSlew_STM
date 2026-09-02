#include "lead_lag.h"
#include "constants.h"

void LeadLag_Init(lead_lag_t *l, float gain, float crossover_frequency, float phase_shift, float sample_period)
{
	float sin = sinf(phase_shift);
	float alpha = (1.0f + sin) / (1.0f - sin);
	float root_alpha = sqrtf(alpha);

	float fz = crossover_frequency / root_alpha;
	float fp = crossover_frequency * root_alpha;

	float az = 2.0f / (sample_period * TWO_PI_F * fz);
	float ap = 2.0f / (sample_period * TWO_PI_F * fp);

	l->b = (1.0f - ap) / (1.0f + ap);
	l->b0 = gain * (1.0f + az) / (1.0f + ap);
	l->b1 = gain * (1.0f - az) / (1.0f + ap);
}

float LeadLag_GetOutput(lead_lag_t *l, float error)
{
	float output = l->b * l->prev_output + l->b0 * error + l->b1 * l->prev_error;

	l->prev_error = error;
	l->prev_output = output;

	return output;
}

void LeadLag_Reset(lead_lag_t *l)
{
	l->prev_error = 0.0f;
	l->prev_output = 0.0f;
}