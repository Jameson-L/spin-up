#pragma once
#include "main.h"

extern bool continueFlywheel;
extern double speed;
void flywheelTask();

// routes
void left(); // left side auton
void right(); // right side auton
void awpLeft(); // solo win point starting left side
void awpRight(); // solo win point starting righth side
