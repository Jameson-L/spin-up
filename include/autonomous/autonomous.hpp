#pragma once
#include "main.h"

extern bool continueFlywheel;
extern double speed;
void flywheelTask();
extern double tbhGain;

// routes
void left(); // left side auton
void right(); // right side auton
void awp(); // solo win point
void skills(); // skills
