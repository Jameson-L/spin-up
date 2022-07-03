#pragma once
#include "main.h"

// variables
// portNumbers

// odometry chassis declared in chassis.hpp

// IMUs
extern okapi::IMU imu1;
extern okapi::IMU imu2;

// ADI Encoders
extern okapi::ADIEncoder LTrackingWheel;
extern okapi::ADIEncoder RTrackingWheel;
extern okapi::ADIEncoder MTrackingWheel;

// drive PIDs
extern okapi::IterativePosPIDController chassisTurnPid;
extern okapi::IterativePosPIDController chassisDrivePid;
extern okapi::IterativePosPIDController chasissSwingPid;

// functions
// helper functions
double getHeading(bool safe = false);
bool isMoving();

void odomDriveToPoint(double x, double y, bool forward=true, double offset = 0.0, double speedMultiplier = 1, double time = 4);
void jCurve(double x, double y, bool forward=true, double offset = 0.0, double speedMultiplier = 1, double time = 4);
void imuTurnToAngle(double deg);
void imuZeroToAngle(double deg, double time = 2);
void relative(double x, double time = 2);
