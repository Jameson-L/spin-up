#pragma once
#include "main.h"

// IMUs
extern okapi::IMU imu1;
extern okapi::IMU imu2; // used in case we want to counter imu drift

extern pros::Vision vision;
extern pros::vision_signature_s_t blue;
extern pros::vision_signature_s_t red;

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
double getHeading(bool safe = false); // returns current imu reading (-180, 180),
                                      // safe is used to protect PID from 180 to -180 jump
bool isMoving(); // if the bot is within a certain RPM threshold it is considered stopped

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

void odomDriveToPoint(double x, double y, bool forward=true, double offset = 0.0, double speedMultiplier = 1, double time = 4);
// in-place turn, then translation to any point
void jCurve(double x, double y, bool forward=true, double offset = 0.0, double speedMultiplier = 1, double time = 4);
// curve to any point in a J-shaped path (turn while driving)
void imuTurnToAngle(double deg);
// turn to any angle
void relative(double x, double speedMultiplier = 1, double time = 2);
// translate forward/backward any amount of distance, regardless of position
void turnToPoint(double x, double y);
// turn to any angle
void stepAutoAim();
// for use in a loop
