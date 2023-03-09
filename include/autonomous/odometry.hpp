#pragma once
#include "main.h"

// IMUs
extern okapi::IMU imu1;
extern okapi::IMU imu2; // used in case we want to counter imu drift

extern pros::Vision vision;
extern pros::vision_signature_s_t blue;
extern pros::vision_signature_s_t red;

extern okapi::OpticalSensor optical;

// ADI Encoders
extern okapi::ADIEncoder LTrackingWheel; // left tracking wheel
extern okapi::ADIEncoder RTrackingWheel; // right tracking wheel
extern okapi::ADIEncoder MTrackingWheel; // middle tracking wheel

// drive PIDs
extern okapi::IterativePosPIDController chassisTurnPid; // turning based on imu reading
extern okapi::IterativePosPIDController chassisDrivePid; // translations
extern okapi::IterativePosPIDController chassisSwingPid; // turning with only one side of the drivetrain
extern okapi::IterativePosPIDController chassisVisionPid; // for auto aim

// functions
// returns current imu reading within [-180, 180]
double getHeading();
// Return the angle difference between the robot and the target
double getAngleDiff(double target, int direction = 0);
// if the bot is within a certain RPM threshold it is considered stopped
bool isMoving();

bool isRed();
bool isBlue();

/*
general info:
distances are in ft
x = forward/backward
y = left/right
forward = direction boolean
offset = shortcut method of tuning an overshoot, rarely used
speedMultiplier = max speed
time = max time spent before breaking
*/

// in-place turn, then translation to any point
void odomDriveToPoint(double x, double y, bool forward = true, double offset = 0.0, double speedMultiplier = 1, double time = 4, int direction = 0);
// in-place turn, then translation to any point
void fastDriveToPoint(double x, double y, bool forward = true, double offset = 0.0, double speedMultiplier = 1, double time = 4, int direction = 0);
// curve to any point in a J-shaped path (turn while driving)
void jCurve(double x, double y, bool forward=true, double offset = 0.0, double speedMultiplier = 1, double time = 4, int direction = 0);
 // turn to any angle
void imuTurnToAngle(double deg, bool fast=false, int direction = 0);
// translate forward/backward any amount of distance, regardless of position
void relative(double x, double speedMultiplier = 1, double time = 2);
// turn towards any point
void turnToPoint(double x, double y);
// for use in a loop
void stepAutoAim();
