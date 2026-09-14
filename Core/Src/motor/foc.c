#include "foc.h"
#include "constants.h"
#include "config.h"
#include "cordic.h"
#include "tim.h"
#include "servo.h"
#include "serial.h"
#include "usb.h"
#include "mt6835.h"
#include "pid.h"
#include "lpf.h"

#include <math.h>
#include <string.h>
#include <stdint.h>

static ADC_HandleTypeDef *hadcA;
static ADC_HandleTypeDef *hadcB;
static ADC_HandleTypeDef *hadcC;

static volatile uint16_t adc_a;
static volatile uint16_t adc_b;
static volatile uint16_t adc_c;

static float adc_a_zero = 0.0f;
static float adc_b_zero = 0.0f;
static float adc_c_zero = 0.0f;
static uint32_t adc_avg_index = 0;

float ia = 0.0f;
float ib = 0.0f;
float ic = 0.0f;

float v_bus = 0.0f;
float maximum_power = 10.0f;		// v vatih


float powers[VELOCITY_LOOP_PRESCALER] = {0.0f};
uint16_t power_series = 0;
float iqs[VELOCITY_LOOP_PRESCALER] = {0.0f};


lpf_t iq_lpf;

pid_t q_pid;
pid_t d_pid;

float used_iq_target = 0.0f;

float iq_reference = 4.0f;
float iq_compensation = 0.0f;

float delta = 0.0f;
bool iq_saturated = false;

float setpoint_angle = 0.0f;
float setpoint_velocity = 0.0f;


static float deadtime_compensation_voltage = 0.0f;

volatile bool enabled = false;

volatile bool a_ready = false;
volatile bool b_ready = false;
volatile bool c_ready = false;

uint16_t position_counter = POSITION_LOOP_PRESCALER - 1;
uint16_t velocity_counter = VELOCITY_LOOP_PRESCALER - 1;

static float i_alpha = 0.0f;
static float i_beta = 0.0f;

static float id = 0.0f, iq = 0.0f;

static int32_t s = 0, c = 0;

static float v_alpha = 0.0f, v_beta = 0.0f;

static float va, vb, vc;

static float vd = 0.0f, vq = 0.0f;

static uint16_t loop_start_counter = 0;

static int32_t prev_encoder_position = 0;

static const float foc_loop_period = (float)(2 * TIM_PERIOD + 1) / 170e6f;

void static inline clarke_transform(float _ia, float _ib, float _ic, float *_i_alpha, float *_i_beta)
{
    *_i_alpha = _ia;
    *_i_beta = ROOT_THREE_INVERSE_F * (_ib - _ic);
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

static inline float adc_to_i(float adc)
{
#ifdef RA
	return -3.934337267e-3f * adc;		// Vref / ((2^12 - 1) * gain * R_sense), R_sense je 9 mO
#else
	return 3.934337267e-3f * adc;
#endif
}

static inline void get_v_bus()
{
	v_bus = (float)(hadc4.Instance->DR) * 0.01112260288f;	// Vref * (R1 + R2) / ((2^12 - 1) * R2)
	deadtime_compensation_voltage = v_bus * 0.375f * 0.00526742301f;	// 63 mV na fazo za v_bus = 12 V, T_dead / T_s, mogoče brez * 2.0f?
}

static float deadtime_compensation(float i)
{
	float k = i * DEADTIME_COMPENSATION_GAIN;
	if(k > 1.0f) {
		k = 1.0f;
	} else if (k < -1.0f) {
		k = -1.0f;
	}
	return k * deadtime_compensation_voltage;
}

void FOC_Calibrate_ADCs(void)
{
	while (!(a_ready && b_ready && c_ready)) {}
    a_ready = false; b_ready = false; c_ready = false;

    adc_avg_index++;

    adc_a_zero += ((float)adc_a - adc_a_zero) / (float)adc_avg_index;
    adc_b_zero += ((float)adc_b - adc_b_zero) / (float)adc_avg_index;
    adc_c_zero += ((float)adc_c - adc_c_zero) / (float)adc_avg_index;
}

void FOC_ADC_Callback(ADC_HandleTypeDef *hadc)
{
	if(hadc == hadcA) {
		adc_a = hadcA->Instance->JDR1;
		a_ready = true;
	} else if(hadc == hadcB) {
		adc_b = hadcB->Instance->JDR1;
		b_ready = true;
	} else if(hadc == hadcC) {
		adc_c = hadcC->Instance->JDR1;
		c_ready = true;
	}
}

static void set_setpoint_angle(void)
{
	int32_t raw_angle = MT6835_GetRawAngle() - (int32_t)ENCODER_ANGLE_OFFSET;
	if(raw_angle > (1 << 20)) {
    	raw_angle -= (1 << 21);
	} else if(raw_angle < -(1 << 20)) {
    	raw_angle += (1 << 21);
	}
	float electrical_angle = wrap_pi(- MT6835_RAW_TO_RAD_F * (float)(POLE_PAIRS * (raw_angle)));
	setpoint_angle = electrical_angle;
}

void FOC_Init(ADC_HandleTypeDef *_hadcA, ADC_HandleTypeDef *_hadcB, ADC_HandleTypeDef *_hadcC)
{
	hadcA = _hadcA;
	hadcB = _hadcB;
	hadcC = _hadcC;

	PID_Init(&d_pid, 0.375f, 2205.0f, 0.0f);
	PID_Init(&q_pid, 0.375f, 2205.0f, 0.0f);

	LPF_Init(&iq_lpf, 30.0f, foc_loop_period);

	adc_a = 0x7fb;
	adc_b = 0x7fb;
	adc_c = 0x7fb;

	HAL_ADC_Start(&hadc4);

	MT6835_FetchAngleSync();

	set_setpoint_angle();

	HAL_ADC_PollForConversion(&hadc4, 10);
	get_v_bus();
}

void FOC_Loop()
{
	if(velocity_counter + 1 >= VELOCITY_LOOP_PRESCALER) {
		velocity = calculate_current_velocity();
	}

	get_v_bus();
	HAL_ADC_Start(&hadc4);

	if (!(EN_PORT->IDR & EN_PIN) || v_bus < UVLO) {
	    enabled = false;
		TIM1->BDTR &= ~TIM_BDTR_MOE;		// izklopi tim1 izhode

		set_setpoint_angle();

	    servo_reset_pid();

		PID_Reset(&d_pid);
		PID_Reset(&q_pid);
		LPF_Reset(&iq_lpf);

	    return;
	}

	if(!enabled) {
		enabled = true;
		TIM1->BDTR |= TIM_BDTR_MOE;			// vklopi tim1 izhode
	}

	//* druga pida
    if(++position_counter >= POSITION_LOOP_PRESCALER) {
        calculate_position_pid();
        position_counter = 0;
    }
	if(++velocity_counter >= VELOCITY_LOOP_PRESCALER) {
		calculate_velocity_pid();
		velocity_counter = 0;
	}

	while(!(a_ready && b_ready && c_ready)) {}
	a_ready = false; b_ready = false; c_ready = false;

	ia = adc_to_i((float)adc_a - adc_a_zero);
	ib = adc_to_i((float)adc_b - adc_b_zero);
	ic = adc_to_i((float)adc_c - adc_c_zero);

	float i_avg = (ia + ib + ic) * 0.333333333f;

	ia -= i_avg;
	ib -= i_avg;
	ic -= i_avg;

    //* clarke transformacija
	clarke_transform(ia, ib, ic, &i_alpha, &i_beta);

	//while(!MT6835_DataAvailable()) {}

	//* inkrementacija pozicije
	int32_t encoder_position = MT6835_GetRawAngle();
	int32_t encoder_diff = encoder_position - prev_encoder_position;

	if(encoder_diff > (1L << 20)) {
		encoder_diff -= (1L << 21);
	} else if(encoder_diff < -(1L << 20)) {
		encoder_diff += (1L << 21);
	}

    position += (int64_t)encoder_diff;
	prev_encoder_position = encoder_position;

	int32_t raw_angle = encoder_position - (int32_t)ENCODER_ANGLE_OFFSET;
	if(raw_angle > (1 << 20)) {
    	raw_angle -= (1 << 21);
	} else if(raw_angle < -(1 << 20)) {
    	raw_angle += (1 << 21);
	}

	//* električni kot
	float encoder_ff = (float)POLE_PAIRS * velocity * (float)(DWT->CYCCNT - encoder_read_cycle) * 5.88235294e-9f;
	float electrical_angle = wrap_pi(-MT6835_RAW_TO_RAD_F * (float)(POLE_PAIRS * (raw_angle)) + encoder_ff);	// enkoder se prebere pol cikla prej
	
	float setpoint_difference = setpoint_velocity * foc_loop_period;

	setpoint_angle = wrap_pi(setpoint_angle + setpoint_difference);

	delta = wrap_pi(setpoint_angle - electrical_angle);

	setpoint_angle = wrap_pi(electrical_angle + delta);

	//* park transformacija
	CORDIC_SinCos(CORDIC_RadToQ31(electrical_angle), &s, &c);

	float sin_electrical = CORDIC_Q31ToTrig(s);
	float cos_electrical = CORDIC_Q31ToTrig(c);

	park_transform(i_alpha, i_beta, sin_electrical, cos_electrical, &id, &iq);

    //* dq pi-ja
	vd = PID_GetOutput(&d_pid, 0.0f - id);

	float iq_target = (delta * iq_reference) + iq_compensation;

	iq_target = LPF_GetOutput(&iq_lpf, iq_target);

	float max_iq_step = IQ_MAX_RISE * d_pid.dt;
	float iq_diff = iq_target - used_iq_target;
	if (absf(iq_diff) <= max_iq_step) {
	    used_iq_target = iq_target;
	} else {
	    used_iq_target += signf(iq_diff) * max_iq_step;
	}

	if(used_iq_target > IQ_MAX) {
		setpoint_angle = wrap_pi(setpoint_angle - setpoint_difference);
		used_iq_target = IQ_MAX;
		iq_saturated = true;
	} else if(used_iq_target < -IQ_MAX) {
		setpoint_angle = wrap_pi(setpoint_angle - setpoint_difference);
		used_iq_target = -IQ_MAX;
		iq_saturated = true;
	}

	vq = PID_GetOutput(&q_pid, used_iq_target - iq);		// pozitiven iq je navor CCW

	//* omejitev napetosti
	float v_ref2 = vd*vd + vq*vq;
	float v_peak = v_bus * ROOT_THREE_INVERSE_F;
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
	if(++power_series >= VELOCITY_LOOP_PRESCALER) {
		power_series = 0;
	}
	powers[power_series] = power;
	iqs[power_series] = iq;
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

	/*
	if(loop_start_counter < 30) {
		loop_start_counter++;
		servo_reset_pid();		// lahko se poveča d / hitrost, zaradi poznega call je dt manjši

		PID_Reset(&d_pid);
		PID_Reset(&q_pid);

		LPF_Reset(&iq_lpf);

		return;
	}
	*/

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

	/*
	if(velocity_counter != 1) {
		return;
	}

	uint8_t b[SERIAL_MAX_SIZE] = {0};
    uint16_t len = u64ToDec((uint64_t)(1e4f + 1000.0f * ia), b);
    b[len++] = ',';
	len += u64ToDec((uint64_t)(1e4f + 1000.0f * ib), b + len);
	b[len++] = ',';
	len += u64ToDec((uint64_t)(1e4f + 1000.0f * ic), b + len);
	b[len++] = '\n';
    usb_serial.print(b, len);
	*/
}