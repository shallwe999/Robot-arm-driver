#include "Encoder.h"

double read_pitch_motor_angle()
{
    double result;
    int data;
    data = pulseIn(ENC_PITCH, HIGH, 5000);
    if(data == 0)
      result = now_motor_angle_pitch_old;
    else
      result = (static_cast<double>(data) + PITCH_PWM_OFFSET) / ENCODER_HIGHTIME_RANGE_PITCH * 360 + PITCH_ANGLE_OFFSET;
    return result;
}

double read_roll_motor_angle()
{
    double result;
    int data;
    data = pulseIn(ENC_ROLL, HIGH, 5000);
    if(data == 0)
      result = now_motor_angle_roll_old;
    else
      result = (static_cast<double>(data) + ROLL_PWM_OFFSET) / ENCODER_HIGHTIME_RANGE_ROLL * 360 + ROLL_ANGLE_OFFSET;
    return result;
}
