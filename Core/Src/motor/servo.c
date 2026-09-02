#include "servo.h"
#include "pid.h"
#include "mt6835.h"
#include "foc.h"
#include "config.h"
#include "gpio.h"
#include "adc.h"
#include "usb.h"
#include "main.h"
#include "serial.h"
#include "constants.h"
#include "lead_lag.h"
#include "cordic.h"
#include "resonant.h"
#include "lpf.h"

#include <stdint.h>
#include <string.h>
#include <math.h>

float velocity = 0.0f;
float velocity_target = 0.036f;	// sidereal 0.035903916
static float used_velocity_target = 0.0f;

int64_t position = 0;
int64_t position_target = 0;

pid_t position_pid;
pid_t velocity_pid;

lpf_t velocity_pid_lpf;

lpf_t velocity_lpf;

lead_lag_t compensation_lead_lag;
pid_t compensation_pid;
//resonant_t compensation_resonant;

static int32_t velocity_average_counts[VELOCITY_AVERAGING_INTERVAL] = {0};
static int32_t velocity_average_cycles[VELOCITY_AVERAGING_INTERVAL] = {0};
static uint16_t velocity_average_index = 0;

static uint32_t starting_loops = 0;

static const float velocity_sampling_period = ((float)((2L * (long)TIM_PERIOD + 1L) * (long)VELOCITY_LOOP_PRESCALER)) / 170e6f;

static const float acceleration_rise_threshold = MAX_ACCELERATION * ((float)((2L * (long)TIM_PERIOD + 1L) * (long)VELOCITY_LOOP_PRESCALER)) / 170e6f;

void Servo_Init(void)
{
	PID_Init(&velocity_pid, 0.0f, 56.0f, 0.0f);		// ki = 56 mi da efektivno isti i gain kot setpoint angle prej z kp = 1, ki = p(kp + 1)I_rer

	PID_Init(&position_pid, 0.0f, 0.0f, 0.0f);
	//PID_Init(&compensation_pid, 0.7f, 0.0f, 0.0f);

	LPF_Init(&velocity_pid_lpf, 5.0f, velocity_sampling_period);
	LPF_Init(&velocity_lpf, 15.0f, velocity_sampling_period);

	//LeadLag_Init(&compensation_lead_lag, COMPENSATION_GAIN, COMPENSATION_FREQUENCY_MAX, COMPENSATION_PHASE_SHIFT, velocity_sampling_period);

	//Resonant_Init(&compensation_resonant, 0.48f, 0.15f, 10.0f, velocity_sampling_period);
}

void servo_reset_pid(void)
{
	velocity_pid.integral = 0.0f;
	velocity_pid.prev_cycle = DWT->CYCCNT;

	//compensation_pid.integral = 0.0f;
	//compensation_pid.prev_cycle = DWT->CYCCNT;

	position_pid.integral = 0.0f;
	//todo: prev_cycle za lego?

	int32_t encoder_angle = MT6835_GetRawAngle();
	uint32_t now = DWT->CYCCNT;
	for(int i = 0; i < VELOCITY_AVERAGING_INTERVAL; i++) {
		velocity_average_counts[i] = encoder_angle;
		velocity_average_cycles[i] = now;
	}
	velocity_average_index = 0;
}

void calculate_position_pid(void)
{
	return;

    velocity_target = PID_GetOutput(&position_pid, (float)(position_target - position));
}

float calculate_current_velocity(void)
{
	int32_t new_encoder = MT6835_GetRawAngle();
	int32_t encoder_diff = new_encoder - velocity_average_counts[velocity_average_index];

	if(encoder_diff > (1L << 20)) {
		encoder_diff -= (1L << 21);
	} else if(encoder_diff < -(1L << 20)) {
		encoder_diff += (1L << 21);
	}

	uint32_t new_cycle = DWT->CYCCNT;
    float current_velocity = -((float)(encoder_diff) * 509.3295584f /
		((float)(new_cycle - velocity_average_cycles[velocity_average_index])));	// 2pi / (2^21 * (1 / 170e6))
	
	velocity_average_counts[velocity_average_index] = new_encoder;
	velocity_average_cycles[velocity_average_index] = new_cycle;

	if(++velocity_average_index >= VELOCITY_AVERAGING_INTERVAL) {
		velocity_average_index = 0;
	}

    return current_velocity;
}

void calculate_velocity_pid(void)
{
	if(++starting_loops <= 500) {	// 2 s, bolj stabilna hitrost je tako, nimam blage zakaj
		return;
	}

	float velocity_error = used_velocity_target - velocity;

	float power = 0.0f;
	for(int i = 0; i < VELOCITY_LOOP_PRESCALER; i++) {
		power += powers[i];
	}
	power /= (float)VELOCITY_LOOP_PRESCALER;

	iq_target = PID_GetOutput(&velocity_pid, velocity_error);

	if(iq_saturated) {
		velocity_pid.integral -= velocity_error * velocity_pid.dt;
	}

	if(absf(used_velocity_target - velocity_target) > acceleration_rise_threshold) {
		used_velocity_target += signf(velocity_target) * MAX_ACCELERATION * velocity_pid.dt;
	} else {
		used_velocity_target = velocity_target;
	}



	//* setpoint_velocity = (float)POLE_PAIRS * (velocity_target + PID_GetOutput(&velocity_pid, velocity_error));

	/*
	*if(iq_saturated) {
	*	if(((delta > 0.0 && velocity_error > 0.0f) || (delta < 0.0 && velocity_error < 0.0f)) &&
	*		velocity_pid.components & PID_INTEGRAL && velocity_pid.ki != 0.0f)
	*	{
	*		velocity_pid.integral -= velocity_error * velocity_pid.dt;
	*	}
	*	iq_saturated = false;
	*}
	*/

	//float new_compensation = (absf(relative_error) <= 0.5f) ? PID_GetOutput(&compensation_pid, velocity_error) : 0.0f;

	//iq_compensation = iq_compensation + compensation_alpha * (new_compensation - iq_compensation);

	//iq_compensation = LeadLag_GetOutput(&compensation_lead_lag, velocity_error);
	//* iq_compensation = PID_GetOutput(&compensation_pid, velocity_error);

	/*
	float max_compensation = COMPENSATION_MAX * (absf(velocity_target)) / (absf(velocity_target) + COMPENSATION_MAX);	// f(x) = a * x/(x + a)

	if(iq_compensation > max_compensation) {
		if(compensation_pid.components & PID_INTEGRAL) {
			compensation_pid.integral -= velocity_error * compensation_pid.dt;
		}
		iq_compensation = max_compensation;
	} else if(iq_compensation < -max_compensation) {
		if(compensation_pid.components & PID_INTEGRAL) {
			compensation_pid.integral -= velocity_error * compensation_pid.dt;
		}
		iq_compensation = -max_compensation;
	}
	*/

	v_d = velocity;

	if(velocity >= 50.0f) {
		PID_Reset(&velocity_pid);
	}

	uint8_t b[64] = {0};
    //uint16_t len = u64ToDec(diff, b);
    uint16_t len = u64ToDec((uint64_t)(1e9f + 1e6f * v_d), b);
    b[len] = ',';
	len += u64ToDec((uint64_t)(1e9f + 1e6f * power), b + len + 1);
	b[len + 1] = '\n';
    usb_serial.print(b, 64);
}