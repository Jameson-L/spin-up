#pragma once
#include "main.h"

// variables
// port numbers
extern const int8_t kDriveLBPort;
extern const int8_t kDriveLIPort;
extern const int8_t kDriveLOPort;
extern const int8_t kDriveRBPort;
extern const int8_t kDriveRIPort;
extern const int8_t kDriveROPort;

// chassis
extern std::shared_ptr<okapi::OdomChassisController> chassis;
