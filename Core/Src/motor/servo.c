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
#include "pec.h"

#include <stdint.h>
#include <string.h>
#include <math.h>

float velocity = 0.0f;
float velocity_target = 0.050f;	// sidereal 0.05119632472517
static float used_velocity_target = 0.0f;

int64_t position = 0;
int64_t position_target = 0;

pid_t position_pid;
pid_t velocity_pid;

lpf_t velocity_lpf;

pid_t compensation_pid;
lpf_t compensation_lpf;

lpf_t friction_lpf;

static int32_t velocity_average_counts[VELOCITY_AVERAGING_INTERVAL] = {0};
static int32_t velocity_average_cycles[VELOCITY_AVERAGING_INTERVAL] = {0};
static uint16_t velocity_average_index = 0;

static uint32_t starting_loops = 0;

const float velocity_sampling_period = ((float)((2L * (long)TIM_PERIOD + 1L) * (long)VELOCITY_LOOP_PRESCALER)) / 170e6f;

void Servo_Init(void)
{
	PID_Init(&velocity_pid, 1.0f, 0.0f, 0.0f);	// efektivno integrator, ki = (1 + kp) * p * I_ref
	PID_Init(&compensation_pid, 0.0f, 0.0f, 0.0f);

	PID_Init(&position_pid, 0.0f, 0.0f, 0.0f);
	LPF_Init(&friction_lpf, FRICTION_CUTOFF, velocity_sampling_period);

	LPF_Init(&velocity_lpf, 15.0f, velocity_sampling_period);
	LPF_Init(&compensation_lpf, COMPENSATION_CUTOFF, velocity_sampling_period);
}

void servo_reset_pid(void)
{
	velocity_pid.integral = 0.0f;
	velocity_pid.prev_cycle = DWT->CYCCNT;

	compensation_pid.integral = 0.0f;
	compensation_pid.prev_cycle = DWT->CYCCNT;

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
	float max_velocity_step = MAX_ACCELERATION * velocity_pid.dt;
	float velocity_diff = velocity_target - used_velocity_target;
	if(absf(velocity_diff) <= max_velocity_step) {
	    used_velocity_target = velocity_target;
	} else {
	    used_velocity_target += signf(velocity_diff) * max_velocity_step;
	}

	float velocity_error = used_velocity_target - velocity;

	float power = 0.0f;
	float iq_avg = 0.0f;
	for(int i = 0; i < VELOCITY_LOOP_PRESCALER; i++) {
		power += powers[i];
		iq_avg += iqs[i];
	}
	power /= (float)VELOCITY_LOOP_PRESCALER;
	iq_avg /= (float)VELOCITY_LOOP_PRESCALER;

	if(iq_saturated) {
		velocity_pid.integral -= velocity_error * velocity_pid.dt;
	}

	setpoint_velocity = (float)POLE_PAIRS * (velocity_target + PID_GetOutput(&velocity_pid, velocity_error));
	
	float friction_velocity = 0.0f;
	if(velocity_target > 0.0f) {
		if(velocity >= velocity_target * FRICTION_VELOCITY_THRESHOLD) {
			friction_velocity = velocity;
		} else {
			friction_velocity = velocity_target;
		}
	} else if(velocity_target < 0.0f) {
		if(velocity <= velocity_target * FRICTION_VELOCITY_THRESHOLD) {
			friction_velocity = velocity;
		} else {
			friction_velocity = velocity_target;
		}
	}

	float friction_ff = 0.0f;
	if(velocity_target > 0.0f) {
		friction_ff = -0.0292f * friction_velocity + 0.0475f / (friction_velocity + 0.2547f) + 2.19f;
 	} else if(velocity_target < 0.0f) {
		friction_ff = -0.0255f * velocity - 2.17f;
	}

	friction_ff = LPF_GetOutput(&friction_lpf, friction_ff);

	float pe_error = 0.0f;
	float pe_derivative = 0.0f;
	get_pe(MT6835_GetAngle(), &pe_error, &pe_derivative);

	float pec_ff = compensation_pid.prev_derivative * pe_error + velocity * velocity_target * pe_derivative;

	float compensation = LPF_GetOutput(&compensation_lpf, PID_GetOutput(&compensation_pid, velocity_error));

	compensation = 0.0f;
	friction_ff = 0.0f;
	pec_ff = 0.0f;

	iq_compensation = compensation + 0.9f * friction_ff + pec_ff;

	if(absf(velocity) >= 50.0f) {
		PID_Reset(&velocity_pid);
	}

	uint8_t b[SERIAL_MAX_SIZE] = {0};
	//uint16_t len = u64ToDec(MT6835_GetRawAngle(), b);
    uint16_t len = u64ToDec((uint64_t)(1e9f + 1e6f * velocity), b);
    //b[len++] = ',';
	//len += u64ToDec((uint64_t)(1e9f + 1e6f * pec_ff), b + len);
	//b[len++] = ',';
	//len += u64ToDec((uint64_t)(MT6835_GetRawAngle()), b + len);
	b[len++] = '\n';
    usb_serial.print(b, len);
}