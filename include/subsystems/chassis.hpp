#pragma once
#include "main.h"

// port numbers (6 motor drive)
extern const int8_t kDriveLFPort; // left front
extern const int8_t kDriveLMPort; // left motor
extern const int8_t kDriveLBPort; // left back
extern const int8_t kDriveRFPort; // right front
extern const int8_t kDriveRMPort; // right middle
extern const int8_t kDriveRBPort; // right back

// chassis
extern std::shared_ptr<okapi::OdomChassisController> chassis;

// motorgroup of all motors for control over brake type etc.
extern okapi::MotorGroup allMotors;
extern okapi::MotorGroup leftMotors;
extern okapi::MotorGroup rightMotors;
