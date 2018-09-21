#include "Encoder.h"
#include "definitions.h"
#include "serial_com.h"
const int MEN0=2;
const int MEN1=4;

const int M0C=3;
const int M0B=5;
const int M0A=6;

const int M1C=9;
const int M1B=10;
const int M1A=11;

const int ENC_PITCH=8;
const int ENC_ROLL=12;
const int LOCK_PIN=7;

//33%占空比
//int pwmSin[]={43,45,46,47,48,50,51,52,54,55,56,57,59,60,61,62,64,65,66,67,68,70,71,72,73,74,76,77,78,79,80,80,81,81,81,82,82,82,83,83,83,83,84,84,84,84,85,85,85,85,85,85,85,85,86,86,86,86,86,86,86,86,86,86,86,86,86,85,85,85,85,85,85,85,85,84,84,84,84,83,83,83,83,82,82,82,81,81,81,80,80,80,81,81,81,82,82,82,83,83,83,83,84,84,84,84,85,85,85,85,85,85,85,85,86,86,86,86,86,86,86,86,86,86,86,86,86,85,85,85,85,85,85,85,85,84,84,84,84,83,83,83,83,82,82,82,81,81,81,80,80,79,78,77,76,74,73,72,71,70,68,67,66,65,64,62,61,60,59,57,56,55,54,52,51,50,48,47,46,45,43,42,41,39,38,37,36,34,33,32,31,29,28,27,26,24,23,22,21,19,18,17,16,15,13,12,11,10,9,8,7,6,6,6,5,5,5,4,4,4,3,3,3,3,3,2,2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,3,3,3,3,3,4,4,4,5,5,5,6,6,6,7,6,6,6,5,5,5,4,4,4,3,3,3,3,3,2,2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,3,3,3,3,3,4,4,4,5,5,5,6,6,6,7,8,9,10,11,12,13,15,16,17,18,19,21,22,23,24,26,27,28,29,31,32,33,34,36,37,38,39,41,42};
//50%占空比
int pwmSin[]={51,51,52,53,54,55,56,57,58,58,59,60,61,62,63,64,65,65,66,67,68,69,70,70,71,72,73,74,74,75,76,77,78,78,79,80,80,81,82,83,83,84,85,85,86,87,87,88,88,89,90,90,91,91,92,92,93,93,94,94,95,95,96,96,96,97,97,97,98,98,98,99,99,99,100,100,100,100,100,101,101,101,101,101,101,101,101,101,101,101,102,101,101,101,101,101,101,101,101,101,101,101,100,100,100,100,100,99,99,99,98,98,98,97,97,97,96,96,96,95,95,94,94,93,93,92,92,91,91,90,90,89,88,88,87,87,86,85,85,84,83,83,82,81,80,80,79,78,78,77,76,75,74,74,73,72,71,70,70,69,68,67,66,65,65,64,63,62,61,60,59,58,58,57,56,55,54,53,52,51,51,50,49,48,47,46,45,44,43,43,42,41,40,39,38,37,36,36,35,34,33,32,31,31,30,29,28,27,27,26,25,24,23,23,22,21,21,20,19,18,18,17,16,16,15,14,14,13,13,12,11,11,10,10,9,9,8,8,7,7,6,6,5,5,5,4,4,4,3,3,3,2,2,2,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,2,2,2,3,3,3,4,4,4,5,5,5,6,6,7,7,8,8,9,9,10,10,11,11,12,13,13,14,14,15,16,16,17,18,18,19,20,21,21,22,23,23,24,25,26,27,27,28,29,30,31,31,32,33,34,35,36,36,37,38,39,40,41,42,43,43,44,45,46,47,48,49,50};


int currentStepA_r;
int currentStepB_r;
int currentStepC_r;


int currentStepA_p;
int currentStepB_p;
int currentStepC_p;

int fastdown = 0;
int sineArraySize;
int increment = 1;
boolean direct = 1; // direction true=forward, false=backward

angle angle1;
const int angle0=1500;
int count=0;
void (*resetFunc)(void) = 0;
int not_arrive = 0;
//////////////////////////////////////////////////////////////////////////////

void setup() {
  for (int idx = 0; idx < 360; idx++)
    pwmSin[idx] += 5;
  
  // put your setup code here, to run once:
  setPwmFrequency(M0A); // Increase PWM frequency to 32 kHz  (make unaudible)
  setPwmFrequency(M0B);
  setPwmFrequency(M0C);

  setPwmFrequency(M1A); // Increase PWM frequency to 32 kHz  (make unaudible)
  setPwmFrequency(M1B);
  setPwmFrequency(M1C);  

  Serial.begin(9600);
  
  pinMode(MEN0,OUTPUT);
  pinMode(MEN1,OUTPUT);

  digitalWrite(MEN0,HIGH);
  digitalWrite(MEN1,HIGH);
  
  pinMode(M0A,OUTPUT);
  pinMode(M0B,OUTPUT);
  pinMode(M0C,OUTPUT);

  pinMode(M1A,OUTPUT);
  pinMode(M1B,OUTPUT);
  pinMode(M1C,OUTPUT);

  pinMode(ENC_PITCH,INPUT);
  pinMode(ENC_ROLL,INPUT);
  
  pinMode(LOCK_PIN,OUTPUT);
  digitalWrite(LOCK_PIN,LOW);

  sineArraySize = sizeof(pwmSin)/sizeof(int); // Find lookup table size
  int phaseShift = sineArraySize / 3;         // Find phase shift and initial A, B C phase values
  currentStepA_r = 0;
  currentStepB_r = currentStepA_r + phaseShift;
  currentStepC_r = currentStepB_r + phaseShift;

  currentStepA_p = 0;
  currentStepB_p = currentStepA_p + phaseShift;
  currentStepC_p = currentStepB_p + phaseShift;

  sineArraySize--; // Convert from array Size to last PWM array number
  serial_setup();
}

void loop() {
  // put your main code here, to run repeatedly:

    analogWrite(M0A, pwmSin[currentStepA_p]);
    analogWrite(M0B, pwmSin[currentStepB_p]);
    analogWrite(M0C, pwmSin[currentStepC_p]);

    analogWrite(M1A, pwmSin[currentStepA_r]);
    analogWrite(M1B, pwmSin[currentStepB_r]);
    analogWrite(M1C, pwmSin[currentStepC_r]);

    read_serial_data();
   
    now_motor_angle_pitch_old = read_pitch_motor_angle();
    now_motor_angle_roll_old  = read_roll_motor_angle();

    increment_p = ComputePID(5, 200, now_motor_angle_pitch_old, set_motor_angle_pitch, &pitch_angle_error_sum, &pitch_angle_error_old, angle_Kp, angle_Ki, angle_Kd);
    increment_r  = ComputePID(5, 200, now_motor_angle_roll_old,  set_motor_angle_roll,  &roll_angle_error_sum,  &roll_angle_error_old,  angle_Kp, angle_Ki, angle_Kd);


    //move the motor(+- can be changed accoding to three motor lines)
    increment_p = abs_min(max_move_pwm_pitch, static_cast<int>(k_convert_angle_pwm_p * increment_p));
    if (fastdown == 1)
    {
        Serial.println("Reset.");
        delay(200);
        resetFunc();
      }
    else if (fastdown != 0)
    {
        increment_p = max_move_pwm_pitch + 94 - fastdown;
        fastdown--;
    }
    currentStepA_p = currentStepA_p + increment_p;
    currentStepB_p = currentStepB_p + increment_p;
    currentStepC_p = currentStepC_p + increment_p;

    if(currentStepA_p > sineArraySize)  currentStepA_p = 0;
    if(currentStepA_p < 0)  currentStepA_p = sineArraySize;
   
    if(currentStepB_p > sineArraySize)  currentStepB_p = 0;
    if(currentStepB_p < 0)  currentStepB_p = sineArraySize;
   
    if(currentStepC_p > sineArraySize)  currentStepC_p = 0;
    if(currentStepC_p < 0)  currentStepC_p = sineArraySize;
    

    increment_r = abs_min(max_move_pwm_roll, static_cast<int>(k_convert_angle_pwm_r * increment_r));
    currentStepA_r = currentStepA_r - increment_r;
    currentStepB_r = currentStepB_r - increment_r;
    currentStepC_r = currentStepC_r - increment_r;
  
    //Check for lookup table overflow and return to opposite end if necessary
    if(currentStepA_r > sineArraySize)  currentStepA_r = 0;
    if(currentStepA_r < 0)  currentStepA_r = sineArraySize;
   
    if(currentStepB_r > sineArraySize)  currentStepB_r = 0;
    if(currentStepB_r < 0)  currentStepB_r = sineArraySize;
  
    if(currentStepC_r > sineArraySize)  currentStepC_r = 0;
    if(currentStepC_r < 0)  currentStepC_r = sineArraySize;

    delay(500);
    /*
    Serial.println("********   Arguments   ********");
    Serial.print("pitch high pulse time (now)  :  ");Serial.println(pulseIn(ENC_PITCH, HIGH, 5000));
    Serial.print("roll  high pulse time (now)  :  ");Serial.println(pulseIn(ENC_ROLL, HIGH, 5000));
    Serial.print("pitch motor angle (set)      :  ");Serial.println(set_motor_angle_pitch);
    Serial.print("roll  motor angle (set)      :  ");Serial.println(set_motor_angle_roll);
    Serial.print("pitch motor angle (now)      :  ");Serial.println(now_motor_angle_pitch_old);
    Serial.print("roll  motor angle (now)      :  ");Serial.println(now_motor_angle_roll_old);
    Serial.print("pitch motor angle (next step):  ");Serial.println(increment_p);
    Serial.print("roll  motor angle (next step):  ");Serial.println(increment_r);
    */

    if (set_motor_angle_pitch - now_motor_angle_pitch_old > 5 || set_motor_angle_pitch - now_motor_angle_pitch_old  < -5)
        not_arrive += 1;
    else
        not_arrive = 0;
    if (not_arrive >= 400)
    {
        Serial.println("Reset.");
        delay(300);
        resetFunc();
    }
    
}

void setPwmFrequency(int pin) {
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | 0x01;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | 0x01;
    }
  }
  else if(pin == 3 || pin == 11) {
    TCCR2B = TCCR2B & 0b11111000 | 0x01;
  }
}


/************************/
/* PID Controller       */
/************************/
// PID integer inplementation
//   DTms  ... sample period (ms)
//   DTinv ... sample frequency (Hz), inverse of DT (just to avoid division)
double ComputePID(int32_t DTms, int32_t DTinv, double in, double setPoint, double *errorSum, double *errorOld, double Kp, double Ki, double Kd)
{
  double error = setPoint - in;
  double Ierr;
   
  Ierr = error * Ki * DTms;
  *errorSum += Ierr;
 
  /*Compute PID Output*/
  double out = (Kp * error) + *errorSum + Kd * (error - *errorOld) * DTinv;
  *errorOld = error;
  
  return out;  //delta  
}



//math func

int abs_min(int num1, int num2)  // restrict num2 in [-num1, num1]
{
  if(num2 > 0 && num2 > num1)
    return static_cast<int>(num1);
  else if(num2 < 0 && num2 < (-num1))
    return static_cast<int>(-num1);
  else
    return num2;
}
