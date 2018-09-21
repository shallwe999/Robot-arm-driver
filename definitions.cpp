#include "definitions.h"


// data of motor (DIY)
double pitch_angle_error_sum = 0;
double pitch_angle_error_old = 0;
double roll_angle_error_sum  = 0;
double roll_angle_error_old  = 0;
double angle_Kp = 0.03;
double angle_Ki = 0;
double angle_Kd = 0;
double k_convert_angle_pwm_p = 10;
double k_convert_angle_pwm_r = 4;

double set_motor_angle_pitch = 90;
double set_motor_angle_roll  = 180;
double increment_p = 0;
double increment_r = 0;
double now_motor_angle_pitch_old;
double now_motor_angle_roll_old;
int max_move_pwm_roll  = 12;
int max_move_pwm_pitch = 16;

String cmd="";
int Char;


