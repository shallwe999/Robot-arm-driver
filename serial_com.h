#pragma once

#include <string.h>
#include "definitions.h"

extern const int MEN0;
extern angle angle1;
extern const int M0A, M0B, M0C;
extern const int LOCK_PIN;
extern int fastdown;

void serial_setup();
void read_serial_data();
angle Get();
void up();
void down();
void qdown();
void touch();
void lock();
void unlock();
void setRoll(int roll_val);
void setPitch(int pitch_val);

int getVal(String str, String agl);



