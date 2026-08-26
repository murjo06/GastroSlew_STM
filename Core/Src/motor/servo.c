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

#include <string.h>

float velocity = 0.0f;
float target_velocity = 10.0f;	// sidereal 0.035903916f

float iq_offset = 0.0f;

pid_t position_pid;
pid_t velocity_pid;

int64_t position = 0;
volatile int64_t target_position = 0;

int32_t prev_encoder_position = 0;

static int32_t velocity_average_counts[VELOCITY_AVERAGING_INTERVAL] = {0};
static int32_t velocity_average_cycles[VELOCITY_AVERAGING_INTERVAL] = {0};
static uint16_t velocity_average_index = 0;

const float torque_constant_inverse = 1.0f / (float)(TORQUE_CONSTANT);

void Servo_Init(void)
{
	Pid_Init(&velocity_pid, 0b110);
	Pid_Init(&position_pid, 0b000);

	velocity_pid.kp = 0.08f;
	velocity_pid.ki = 0.0f;	//0.21
	velocity_pid.kd = 0.0f;

	velocity_pid.derivative_interval = 3;
}

void servo_reset_pid(void)
{
	velocity_pid.integral = 0.0f;
	position_pid.integral = 0.0f;
}

void calculate_position_pid(void)
{
	return;
    int32_t encoder_position = MT6835_GetRawAngle();        // 21 biten kot
    position += (int64_t)(encoder_position - prev_encoder_position);	//todo: wrap
    target_velocity = get_pid_output(&position_pid, (float)(target_position - position));
    prev_encoder_position = encoder_position;
}

static float calculate_current_velocity(void)
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

void calculate_velocity_pid(float iq)
{
	velocity = calculate_current_velocity();
    iq_target = get_pid_output(&velocity_pid, target_velocity - velocity) - velocity_pid.kd * iq + iq_offset;

	//iq_target = iq_offset + ((DWT->CYCCNT & 0x8000000) ? 2.0f : 0.0f);

	iq_target = 0.0f;

	if(iq_target > 6.0f) {
		iq_target = 6.0f;
	} else if(iq_target < -6.0f) {
		iq_target = -6.0f;
	}

	v_d = velocity;

	if(velocity >= 50.0f) {
		reset_pid(&velocity_pid);
		iq_target = 0.0f;
	}

	uint8_t b[32] = {0};
    //uint16_t len = u64ToDec(MT6835_GetRawAngle(), b);
    uint16_t len = u64ToDec((uint32_t)(100000.0f + 1000.0f * v_d), b);
    b[len] = '\n';
	//len += u64ToDec((uint32_t)(100000.0f + 1000.0f * (iq_target - iq_offset)), b + len + 1);
	//b[len + 1] = '\n';
    usb_serial.print(b, 16);
}

void set_pid_calibration(uint8_t pid, uint8_t index, float value)
{
	switch(pid) {
		case 0: {		//* pozicija
			switch(index) {
				case 0: {		// p
					position_pid.kp = value;
					break;
				} case 1: {		// i
					position_pid.ki = value;
					break;
				} case 2: {		// d
					position_pid.kd = value;
					break;
				} default: return;
			}
			break;
		} case 1: {		//* hitrost
			switch(index) {
				case 0: {		// p
					velocity_pid.kp = value;
					break;
				} case 1: {		// i
					velocity_pid.ki = value;
					break;
				} case 2: {		// d
					velocity_pid.kd = value;
					break;
				} default: return;
			}
			break;
		} case 2: {		//* d
			switch(index) {
				case 0: {		// p
					d_pid.kp = value;
					break;
				} case 1: {		// i
					d_pid.ki = value;
					break;
				} default: return;
			}
			break;
		} case 3: {		//* q
			switch(index) {
				case 0: {		// p
					q_pid.kp = value;
					break;
				} case 1: {		// i
					q_pid.ki = value;
					break;
				} default: return;
			}
			break;
		}
	}
}

float get_pid_calibration(uint8_t pid, uint8_t index)
{
	switch(pid) {
		case 0: {		//* pozicija
			switch(index) {
				case 0: {		// p
					return position_pid.kp;
				} case 1: {		// i
					return position_pid.ki;
				} case 2: {		// d
					return position_pid.kd;
				} default: return 0.0f;
			}
			break;
		} case 1: {		//* hitrost
			switch(index) {
				case 0: {		// p
					return velocity_pid.kp;
				} case 1: {		// i
					return velocity_pid.ki;
				} case 2: {		// d
					return velocity_pid.kd;
				} default: return 0.0f;
			}
			break;
		} case 2: {		//* d
			switch(index) {
				case 0: {		// p
					return d_pid.kp;
				} case 1: {		// i
					return d_pid.ki;
				} default: return 0.0f;
			}
			break;
		} case 3: {		//* q
			switch(index) {
				case 0: {		// p
					return q_pid.kp;
				} case 1: {		// i
					return q_pid.ki;
				} default: return 0.0f;
			}
			break;
		} default: return 0.0f;
	}
	return 0.0f;
}
