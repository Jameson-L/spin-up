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
  while (continueFlywheel) {
    if (flywheel.getActualVelocity() < speed-50) {
      flywheel.controllerSet(1);
    } else {
      flywheel.moveVelocity(speed);
    }
  }
  flywheel.controllerSet(0);
}

void rightRoller() {
  relative(-2, 0.5, 2);
}

void right() {
  speed = 565;
  pros::Task startFlywheel(flywheelTask);
  intake.controllerSet(1);
  jCurve(1.7, 0, true, 0, 1, 2);
  turnToPoint(10.3, 2.9);
  relative(5, 1, 0.3);
  turnToPoint(10.3, 2.9);
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
  relative(-5, 1, 0.3);
  continueFlywheel = false;
  odomDriveToPoint(0.8, 2.5, false, 0, 1, 1.8);
  imuTurnToAngle(0);
  pros::Task rollerTask(rightRoller);
  pros::delay(750);
  intake.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  intake.moveRelative(-300, 600);
}

void left() {
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
  pros::Task rollerTask(rightRoller);
  pros::delay(750);
  intake.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  intake.moveRelative(-300, 600);
}

void awpLeft() {
  // left();
  // odomDriveToPoint(-6.52, -7.35, true, 0, 1, 4);
  // imuTurnToAngle(-90);
  // pros::Task intakeTask1(intakeTask);
  // pros::delay(250);
  // intake.controllerSet(-1);
  // pros::delay(710);
  // intake.controllerSet(0);
}

void awpRight() {}
