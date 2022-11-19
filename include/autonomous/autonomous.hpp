#pragma once
#include "main.h"

extern bool continueFlywheel;
extern double speed;
void flywheelTask();

// routes
void leftOld(); // left side auton
void left();
void right(); // right side auton
void awp(); // solo win point
