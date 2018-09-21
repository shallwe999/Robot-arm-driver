/*************************/
/* Definitions           */
/*************************/

#pragma once
#include <string.h>
#include <Arduino.h>
// added
typedef signed char int8_t;   //8bit有符号类型
typedef unsigned char uint8_t; // 8bit无符号类型
typedef signed int int16_t; //16bit有符号类型
typedef unsigned int uint16_t;//16bit有符号类型
typedef signed long int int32_t;   //328bit有符号类型

struct angle {double roll;double pitch;};


// data of motor (DIY)
//pid
extern double pitch_angle_error_sum;
extern double pitch_angle_error_old;
extern double roll_angle_error_sum;
extern double roll_angle_error_old;
extern double angle_Kp;
extern double angle_Ki;
extern double angle_Kd;
extern double k_convert_angle_pwm_p;
extern double k_convert_angle_pwm_r;
//angle control
extern double set_motor_angle_pitch;
extern double set_motor_angle_roll;
extern double increment_p;
extern double increment_r;
extern double now_motor_angle_pitch_old;
extern double now_motor_angle_roll_old;
extern int max_move_pwm_roll;
extern int max_move_pwm_pitch;

extern String cmd;
extern int Char;
