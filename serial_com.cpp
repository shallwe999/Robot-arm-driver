#include <string.h>
#include <Arduino.h>
#include "serial_com.h"
/*
command
  get 
  setpitch 100
  setroll 100
  up
  down
  qdown
  touch
  lock
  unlock
*/

void serial_setup()
{
  Serial.begin(9600);
  }

void read_serial_data()
{
     while(Serial.available()>0)
        {
            Char=Serial.read();
            delay(50);//不能删掉，否则串口缓冲区不够时间接受数据。即使调小延时也会出错。具体数值也可以实验决定。 
            if (Char=="\n")
            {
              Serial.println(cmd);
              break;
            }
              else
            {
              cmd+=char(Char);
          //    Serial.print(cmd);
              continue;
              }
        } 
        
     if(  cmd.startsWith("get"))
     {
        angle get_angle=Get();
        Serial.print("pitch ");Serial.println(set_motor_angle_pitch);
        Serial.print("roll ");Serial.println(set_motor_angle_roll);          
      }
      else if (cmd.startsWith("up"))
      {
        up();
        }
        else if(cmd.startsWith("down"))
          {
            down();
            }
      else if(cmd.startsWith("qdown"))
      {
        qdown();
        }
      else if(cmd.startsWith("touch"))
      {
        touch();
        }
      else if (cmd.startsWith("lock"))
      {
        lock();
        }
        else if(cmd.startsWith("unlock"))
          {
            unlock();
            }
         else if(cmd.startsWith("setpitch"))
         {
            int  val=getVal(cmd,"pitch");
              if(val!=-1)
                setPitch(val);
              else
                Serial.println("get pitch error!!!");
          }
          else if(cmd.startsWith("setroll"))
          {
            int val=getVal(cmd,"roll");
            if(val!=-1)
              setRoll(val);
              else
                Serial.println("get roll error!!!");
          }
          cmd="";     
  }

//
 angle Get()
 { 
  return angle1;
  }


void up()
{
  setPitch(86);
  Serial.println("up successfully");
 }

void down()
{
  setPitch(185);
  Serial.println("down successfully");
 }
void qdown()
{
  setPitch(90);
  fastdown = 88;
  Serial.println("quick down successfully");
 }
void touch()
{
  setPitch(88);
  fastdown = 60;
  Serial.println("touch successfully");
 }

void lock()
{
  digitalWrite(LOCK_PIN,HIGH);
  Serial.println("locked.");
  }
void unlock()
{
  digitalWrite(LOCK_PIN,LOW);
  Serial.println("unlocked.");
  }
void setPitch(int pitch_val)
{
  angle1.pitch=pitch_val;
  set_motor_angle_pitch = angle1.pitch;
  Serial.print("set pitch ");Serial.println(pitch_val);
  }
void setRoll(int roll_val)
{
  angle1.roll=roll_val;
  set_motor_angle_roll = angle1.roll;
  Serial.print("set roll ");Serial.println(roll_val);
  }

int getVal(String str, String agl)
{
  int val=0;
  int len=0;
  if (agl=="pitch")
  {
    String sub_str=str.substring(9);
    Serial.println(sub_str);
    val=sub_str.toInt();
    Serial.print(val);
    return val;
    }
  else if(agl="roll")
    {
    String sub_str=str.substring(8);
    Serial.println(sub_str);
    val=sub_str.toInt();
    Serial.print(val);
    return val;
      }
      else
      {
        return -1;
        Serial.print(-1);
        }
  }
