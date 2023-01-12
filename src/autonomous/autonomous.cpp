#include "autonomous/odometry.hpp"
#include "subsystems/chassis.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/flywheel.hpp"
#include "subsystems/pneumatics.hpp"
#include "autonomous/autonomous.hpp"

bool continueFlywheel = false;
double speed = 0;

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
	double prevError;
	double output = 0;
	double tbh = speed / 600.0; // maybe tune this, unlikely
	double tbhGain = 0.001; // tune this

  while (continueFlywheel) {
    error = speed - flywheel.getActualVelocity();
    output += tbhGain * error;
    if (signbit(error) != signbit(prevError)) {
      output = 0.5 * (output + tbh);
      tbh = output;
      prevError = error;
    }
    if (flywheel.getActualVelocity() < speed - 100) {
      flywheel.controllerSet(1);
    } else {
      flywheel.controllerSet(output);
    }
    rate.delay(40_Hz);
  }
}

void roller() {
  relative(-2, 0.5, 2);
}

void right() {
  // speed = 350;
  // pros::Task startFlywheel(flywheelTask);
  flywheel.controllerSet(0.85);
  intake.controllerSet(1);
  jCurve(1.7, 0, true, 0, 1, 2);
  turnToPoint(10.3, 2.3);
  relative(5, 1, 0.2);
  turnToPoint(10.3, 2.3);
  pros::c::adi_digital_write(kPneumaticIndexerPort, HIGH);
  pros::delay(300);
  pros::c::adi_digital_write(kPneumaticIndexerPort, LOW);
  pros::delay(1600);
  pros::c::adi_digital_write(kPneumaticIndexerPort, HIGH);
  pros::delay(300);
  pros::c::adi_digital_write(kPneumaticIndexerPort, LOW);
  pros::delay(1600);
  pros::c::adi_digital_write(kPneumaticIndexerPort, HIGH);
  pros::delay(300);
  pros::c::adi_digital_write(kPneumaticIndexerPort, LOW);
  intake.controllerSet(0);
  flywheel.controllerSet(0);
  relative(-5, 1, 0.3);
  continueFlywheel = false;
  odomDriveToPoint(0.5, 2.3, false, 0, 1, 1.8);
  imuTurnToAngle(0);
  pros::Task rollerTask(roller);
  pros::delay(750);
  intake.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  intake.moveRelative(300, 600);
}

void leftOld() {
  speed = 650;
  pros::Task startFlywheel(flywheelTask);
  intake.controllerSet(1);
  jCurve(1, -0.8, true, 0, 0.6, 1.5);
  turnToPoint(8, -3.5);
  relative(-1.4, 1, 1);
  relative(5, 1, 0.3);
  turnToPoint(8, -3.5);
  pros::c::adi_digital_write(kPneumaticIndexerPort, HIGH);
  pros::delay(300);
  pros::c::adi_digital_write(kPneumaticIndexerPort, LOW);
  pros::delay(1600);
  pros::c::adi_digital_write(kPneumaticIndexerPort, HIGH);
  pros::delay(300);
  pros::c::adi_digital_write(kPneumaticIndexerPort, LOW);
  pros::delay(1600);
  pros::c::adi_digital_write(kPneumaticIndexerPort, HIGH);
  pros::delay(300);
  pros::c::adi_digital_write(kPneumaticIndexerPort, LOW);
  intake.controllerSet(0);
  // relative(-5, 1, 0.3);
  continueFlywheel = false;
  odomDriveToPoint(-0.4, -0.8, false, 0, 1, 1.8);
  imuTurnToAngle(0);
  pros::Task rollerTask(roller);
  pros::delay(750);
  intake.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  intake.moveRelative(300, 600);
}

void left() {
  speed = 650;
  pros::Task startFlywheel(flywheelTask);
  intake.controllerSet(1);
  jCurve(2, 0, true, 0, 1, 0.5);
  jCurve(2, 1, true, 0.2, 0.6, 2);
  jCurve(0, 0, false, 0, 1, 1.5);
  turnToPoint(2, 11);
  relative(5, 1, 0.3);
  turnToPoint(2, 11);
  pros::c::adi_digital_write(kPneumaticIndexerPort, HIGH);
  pros::delay(300);
  pros::c::adi_digital_write(kPneumaticIndexerPort, LOW);
  pros::delay(1600);
  pros::c::adi_digital_write(kPneumaticIndexerPort, HIGH);
  pros::delay(300);
  pros::c::adi_digital_write(kPneumaticIndexerPort, LOW);
  pros::delay(1600);
  pros::c::adi_digital_write(kPneumaticIndexerPort, HIGH);
  pros::delay(300);
  pros::c::adi_digital_write(kPneumaticIndexerPort, LOW);
  intake.controllerSet(0);
  continueFlywheel = false;
  odomDriveToPoint(1.5, -0.2, false, 0, 1, 1.8);
  imuTurnToAngle(90);
  pros::Task rollerTask(roller);
  pros::delay(750);
  intake.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  intake.moveRelative(300, 600);
}

void awp() {}
