# pragma once

#include <Arduino.h>

extern const int ENC_PITCH;
extern const int ENC_ROLL;
extern double now_motor_angle_pitch_old;
extern double now_motor_angle_roll_old;

#define ENCODER_HIGHTIME_RANGE_ROLL    300
#define ENCODER_HIGHTIME_RANGE_PITCH   560
#define ROLL_PWM_OFFSET          5
#define PITCH_PWM_OFFSET         -100
#define ROLL_ANGLE_OFFSET        -40
#define PITCH_ANGLE_OFFSET       130

double read_pitch_motor_angle();
double read_roll_motor_angle();

