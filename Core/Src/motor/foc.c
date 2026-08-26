#include "foc.h"
#include "constants.h"
#include "config.h"
#include "cordic.h"
#include "tim.h"
#include <math.h>
#include "servo.h"
#include "serial.h"
#include "usb.h"
#include "mt6835.h"
#include "pid.h"
#include <string.h>
#include <stdint.h>

ADC_HandleTypeDef *hadcA;
ADC_HandleTypeDef *hadcB;
ADC_HandleTypeDef *hadcC;

uint16_t adc_a[FOC_LOOP_PRESCALER];
uint16_t adc_b[FOC_LOOP_PRESCALER];
uint16_t adc_c[FOC_LOOP_PRESCALER];

uint8_t adc_a_index = 0;
uint8_t adc_b_index = 0;
uint8_t adc_c_index = 0;

float ia = 0.0f;
float ib = 0.0f;
float ic = 0.0f;

float iq_target = 0.0f;
float v_bus = 0.0f;
float maximum_power = 10.0f;		// v vatih

static float deadtime_compensation_voltage = 0.0f;

volatile bool enabled = false;

volatile bool a_ready = false;
volatile bool b_ready = false;
volatile bool c_ready = false;

const float two_pi = 2.0f * PI_F;

uint16_t position_counter = POSITION_LOOP_PRESCALER - 1;
uint16_t velocity_counter = VELOCITY_LOOP_PRESCALER - 2;	// hitrost je iz faze s pozicijo

pid_t q_pid;
pid_t d_pid;

static float i_alpha = 0.0f;
static float i_beta = 0.0f;

static float id = 0.0f, iq = 0.0f;

static int32_t s = 0, c = 0;

static float v_alpha = 0.0f, v_beta = 0.0f;

static float va, vb, vc;

static float vd = 0.0f, vq = 0.0f;

static uint16_t loop_start_counter = 0;

void static inline clarke_transform(float _ia, float _ib, float _ic, float *_i_alpha, float *_i_beta)
{
    *_i_alpha = _ia;
    *_i_beta = INVERSE_ROOT_THREE_F * (_ib - _ic);
}

void inverse_clarke_transform(float _v_alpha, float _v_beta, float *_va, float *_vb, float *_vc)
{
    *_va = _v_alpha;
	*_vb = (-_v_alpha + ROOT_THREE_F * _v_beta) * 0.5f;
	*_vc = (-_v_alpha - ROOT_THREE_F * _v_beta) * 0.5f;
}

void park_transform(float _i_alpha, float _i_beta, float _s, float _c, float *_id, float *_iq)
{
    *_id = _i_alpha * _c + _i_beta * _s;
    *_iq = -_i_alpha * _s + _i_beta * _c;
}

void inverse_park_transform(float _vd, float _vq, float _s, float _c, float *_v_alpha, float *_v_beta)
{
	*_v_alpha = _vd * _c - _vq * _s;
	*_v_beta = _vd * _s + _vq * _c;
}

static inline float adc_to_i(int16_t adc)
{
#ifdef RA
	return -3.934337267e-3f * (float)adc;		// Vref / ((2^12 - 1) * gain * R_sense), R_sense je 9 mO
#else
	return 3.934337267e-3f * (float)adc;
#endif
}

static inline void get_v_bus()
{
	v_bus = (float)(hadc4.Instance->DR) * 0.01112260288f;	// Vref * (R1 + R2) / ((2^12 - 1) * R2)
	deadtime_compensation_voltage = v_bus * 0.375f * 0.00526742301f;	// 63 mV na fazo za v_bus = 12 V, T_dead / T_s, mogoče brez * 2.0f?
}

static inline float wrap_pi(float x)
{
	while(x < 0.0f) {
		x += two_pi;
	}
    x = fmodf(x + PI_F, two_pi);
    return x - PI_F;
}

static float deadtime_compensation(float i)
{
	float k = i * DEADTIME_COMPENSATION_SCALING_F;
	if(k > 1.0f) {
		k = 1.0f;
	} else if (k < -1.0f) {
		k = -1.0f;
	}
	return k * deadtime_compensation_voltage;
}

static inline int32_t rad_to_q31(float angle_rad)
{
    return (int32_t)(angle_rad * RAD_TO_Q31_F);
}

static inline float q31_to_float(int32_t x)
{
    return (float)x * Q31_SCALE_INVERSE_F;
}

void FOC_ADC_Callback(ADC_HandleTypeDef *hadc)
{
	if(hadc == hadcA) {
		adc_a[adc_a_index] = hadcA->Instance->JDR1;
		a_ready = true;
		if(++adc_a_index >= FOC_LOOP_PRESCALER) {
			adc_a_index = 0;
		}
	} else if(hadc == hadcB) {
		adc_b[adc_b_index] = hadcB->Instance->JDR1;
		b_ready = true;
		if(++adc_b_index >= FOC_LOOP_PRESCALER) {
			adc_b_index = 0;
		}
	} else if(hadc == hadcC) {
		adc_c[adc_c_index] = hadcC->Instance->JDR1;
		c_ready = true;
		if(++adc_c_index >= FOC_LOOP_PRESCALER) {
			adc_c_index = 0;
		}
	}
}

static int16_t adc_average(uint16_t *data, uint16_t size)
{
	float sum = 0.0f;
	for(uint16_t i = 0; i < size; i++) {
		sum += (float)(data[i]);
	}
	return (int16_t)(0.5f + sum / (float)size);
}

void FOC_Init(ADC_HandleTypeDef *_hadcA, ADC_HandleTypeDef *_hadcB, ADC_HandleTypeDef *_hadcC)
{
	hadcA = _hadcA;
	hadcB = _hadcB;
	hadcC = _hadcC;

	Pid_Init(&d_pid, 0b110);
	Pid_Init(&q_pid, 0b110);

	float bandwidth = 1.5f;		// v kHz, baje

	d_pid.kp = 0.25f * bandwidth;	// K_u je 0.48 @ 8.3 kHz, 	0.56 @ 25 kHz		L = 37.6 uH za R = 0.221 ohm (L graf da R = 0.136 ohm)
	d_pid.ki = 1470.0f * bandwidth;	// T_u je 					10 ms @ 25 kHz

	q_pid.kp = 0.25f * bandwidth;
	q_pid.ki = 1470.0f * bandwidth;

	memset(adc_a, 0x7fb, FOC_LOOP_PRESCALER * sizeof(uint16_t));
	memset(adc_b, 0x7fa, FOC_LOOP_PRESCALER * sizeof(uint16_t));
	memset(adc_c, 0x7fc, FOC_LOOP_PRESCALER * sizeof(uint16_t));

	HAL_ADC_Start(&hadc4);
	HAL_ADC_PollForConversion(&hadc4, 10);
	get_v_bus();
	HAL_ADC_Start(&hadc4);
	HAL_ADC_PollForConversion(&hadc4, 10);
}

void FOC_Loop()
{
	if(!enabled) {
		servo_reset_pid();
		reset_pid(&d_pid);
		reset_pid(&q_pid);
	}
	//* druga pida
    if(++position_counter >= POSITION_LOOP_PRESCALER) {
		get_v_bus();
		HAL_ADC_Start(&hadc4);
        calculate_position_pid();
        position_counter = 0;
    }
	if(++velocity_counter >= VELOCITY_LOOP_PRESCALER) {
		calculate_velocity_pid(iq);
		velocity_counter = 0;
	}

	if(EN_PORT->IDR & EN_PIN) {
		enabled = true;
	} else {
		enabled = false;
		return;
	}

	while(!(a_ready && b_ready && c_ready)) {}
	a_ready = false; b_ready = false; c_ready = false;

	ia = adc_to_i(adc_average(adc_a, FOC_LOOP_PRESCALER) - 0x7fb);		//todo: kalibracija offsetov pri startupu?
	ib = adc_to_i(adc_average(adc_b, FOC_LOOP_PRESCALER) - 0x7fa);
	ic = adc_to_i(adc_average(adc_c, FOC_LOOP_PRESCALER) - 0x7fc);

	float i_avg = (ia + ib + ic) * 0.333333333f;

	ia -= i_avg;
	ib -= i_avg;
	ic -= i_avg;

    //* transformaciji
	clarke_transform(ia, ib, ic, &i_alpha, &i_beta);

	//while(!MT6835_DataAvailable()) {}

	int32_t raw_angle = MT6835_GetRawAngle() - (int32_t)ENCODER_ANGLE_OFFSET;
	if(raw_angle > (1 << 20)) {
    	raw_angle -= (1 << 21);
	} else if(raw_angle < -(1 << 20)) {
    	raw_angle += (1 << 21);
	}

	float encoder_ff = velocity * (float)(DWT->CYCCNT - encoder_read_cycle) * 5.88235294e-9f;
	float electrical_angle = wrap_pi(encoder_ff - MT6835_RAW_TO_RAD_F * (float)(POLE_PAIRS * (raw_angle)));	// enkoder se prebere pol cikla prej
	CORDIC_SinCos(rad_to_q31(electrical_angle), &s, &c);

	float sin_electrical = q31_to_float(s);
	float cos_electrical = q31_to_float(c);

	park_transform(i_alpha, i_beta, sin_electrical, cos_electrical, &id, &iq);

    //* dq pida
	vd = get_pid_output(&d_pid, 0.0f - id);
	vq = get_pid_output(&q_pid, iq_target - iq);	// pozitiven iq je navor CCW

	//* omejitev napetosti
	float v_ref2 = vd*vd + vq*vq;
	float v_peak = v_bus * INVERSE_ROOT_THREE_F;
	float v_peak2 = v_peak * v_peak;
	if(v_ref2 > v_peak2) {
		float scale = v_peak / sqrtf(v_ref2);
		d_pid.integral += 5880.0f * vd * (scale - 1.0f) * d_pid.dt;	// 1 / (kp / ki)
		q_pid.integral += 5880.0f * vq * (scale - 1.0f) * q_pid.dt;	// anti-windup

	    vd *= scale;
	    vq *= scale;
	}

	//* omejitev moči
	float power = 1.5f * (id * vd + iq * vq);	//? pametnejše upravljanje z močjo? mogoče je ta račun napačen?
	if(power > maximum_power) {					//? to je trenutna moč, mogoče dej povprečje?
		//float scale = maximum_power / power;
		//vd *= scale;
		//vq *= scale;
	}

    //* inverzni transformaciji
	inverse_park_transform(vd, vq, sin_electrical, cos_electrical, &v_alpha, &v_beta);

	inverse_clarke_transform(v_alpha, v_beta, &va, &vb, &vc);

	va += deadtime_compensation(ia);
	vb += deadtime_compensation(ib);
	vc += deadtime_compensation(ic);

	float v_max = fmaxf(va, fmaxf(vb, vc));
	float v_min = fminf(va, fminf(vb, vc));

	float voltage_offset = 0.5f * (v_max + v_min);

	va -= voltage_offset;
	vb -= voltage_offset;
	vc -= voltage_offset;

    float da = fminf(fmaxf(0.5f + va / v_bus, 0.0f), 1.0f);
    float db = fminf(fmaxf(0.5f + vb / v_bus, 0.0f), 1.0f);
    float dc = fminf(fmaxf(0.5f + vc / v_bus, 0.0f), 1.0f);

	if(v_bus < UVLO) {
		d_pid.integral = 0.0f;
		q_pid.integral = 0.0f;
		return;
	}

	if(loop_start_counter < 30) {
		loop_start_counter++;
		servo_reset_pid();		// lahko se poveča d / hitrost, zaradi poznega call je dt manjši
		reset_pid(&d_pid);
		reset_pid(&q_pid);
		return;
	}

	int16_t tim_a = (int16_t)(da * (float)TIM1->ARR);
	int16_t tim_b = (int16_t)(db * (float)TIM1->ARR);
	int16_t tim_c = (int16_t)(dc * (float)TIM1->ARR);

	uint16_t tim_max = TIM1->ARR - 1;

	int16_t max_a = max(tim_a, 0);
	int16_t max_b = max(tim_b, 0);
	int16_t max_c = max(tim_c, 0);

    TIM1->CCR3 = min(max_a, tim_max);
    TIM1->CCR2 = min(max_b, tim_max);
    TIM1->CCR1 = min(max_c, tim_max);
}