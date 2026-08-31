#include "lead_lag.h"
#include "constants.h"

void LeadLag_Init(lead_lag_t *l, float gain, float f_z, float f_p, float sample_period)
{
	float az = 2.0f / (sample_period * TWO_PI_F * f_z);
	float ap = 2.0f / (sample_period * TWO_PI_F * f_p);

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