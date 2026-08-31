#ifndef __SERVO_H__
#define __SERVO_H__

#include <stdint.h>
#include "pid.h"

extern float velocity;
extern float target_velocity;

extern float iq_offset;

extern int64_t position;
extern int64_t target_position;

extern pid_t position_pid;
extern pid_t velocity_pid;

void Servo_Init(void);

void servo_reset_pid(void);

float calculate_current_velocity(void);

void calculate_position_pid(void);
void calculate_velocity_pid(void);

#endif