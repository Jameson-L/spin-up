#include "autonomous/odometry.hpp"
#include "subsystems/chassis.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/flywheel.hpp"
#include "subsystems/pneumatics.hpp"
#include "autonomous/autonomous.hpp"

bool continueFlywheel = false;
double speed = 0;
double tbhGain = 0.0007; // tune this


// task functions for auton
void flywheelTask() {
  continueFlywheel = true;
  okapi::Rate rate;
  // while (continueFlywheel) {
  //   if (flywheel.getActualVelocity() < speed-50) {
  //     flywheel.controllerSet(1);
  //   } else {
  //     flywheel.moveVelocity(speed);
  //   }
  //   rate.delay(40_Hz);
  // }
  // flywheel.controllerSet(0);

  double error;
	double prevError = 1;
	double output = 0;
	double tbh = speed / 600.0; // maybe tune this, unlikely

  while (continueFlywheel) {
    error = speed - flywheel.getActualVelocity();
    output += tbhGain * error;
    if (signbit(error) != signbit(prevError)) {
      output = 0.5 * (output + tbh);
      tbh = output;
      prevError = error;
    }

    if (flywheel.getActualVelocity() < speed - 40) {
      flywheel.controllerSet(1);
    } else {
      flywheel.controllerSet(output);
    }
    rate.delay(40_Hz);
  }
}

void roller() {
  relative(-2, 0.3, 2);
}

void right() {
  speed = 650;
  // pros::Task startFlywheel(flywheelTask);
  flywheel.controllerSet(1);
  intake.controllerSet(1);
  // jCurve(8, 0, true, 0, 1, 0.3);
  jCurve(2.6, 0.4, true, 0, 1, 1);
  imuTurnToAngle(24);
  pros::delay(2500);
  intake.controllerSet(0);
  intake.moveRelative(-100, 600);
  pros::delay(1400);
  intake.moveRelative(-100, 600);
  pros::delay(1400);
  intake.moveRelative(-100, 600);
  pros::delay(300);
  jCurve(1.6, 0.8, false, 0, 1, 0.7);
  intake.controllerSet(1);
  odomDriveToPoint(4.75, -3, true, 1.6, 1, 2);
  // jCurve(2.9, 0.2, false, 0, 1, 1);
  imuTurnToAngle(49);
  intake.controllerSet(0);
  // relative(0.4, 1, 0.5);
  relative(1, 1, 0.5);
  intake.moveRelative(-100, 600);
  pros::delay(900);
  intake.moveRelative(-100, 600);
  pros::delay(300);
  flywheel.controllerSet(0);
  jCurve(0.3, 2.2, false, 0, 1, 2);
  imuTurnToAngle(0);
  pros::Task rollerTask(roller);
  pros::delay(250);
  intake.moveRelative(300, 600);
}

void left() {
  // pros::Task rollerTask(roller);
  // pros::delay(500);
  // intake.moveRelative(1000, 600);
  flywheel.controllerSet(1);
  intake.controllerSet(1);
  jCurve(0.6, -0.5, true, 0.1, 1, 1);
  pros::delay(500);
  relative(-0.8, 1, 0.5);
  odomDriveToPoint(5, 5, false, 0, 1, 2);
  imuTurnToAngle(-34);
  relative(0.7, 1, 0.5);
  intake.controllerSet(0);
  pros::delay(2000);
  intake.moveRelative(-100, 600);
  pros::delay(1400);
  intake.moveRelative(-100, 600);
  pros::delay(1400);
  intake.moveRelative(-100, 600);
  pros::delay(300);
  jCurve(0, 0.5, false, 0, 1, 2);
  imuTurnToAngle(0);
  pros::Task rollerTask(roller);
  pros::delay(250);
  intake.moveRelative(300, 600);
}

void awp() {}
