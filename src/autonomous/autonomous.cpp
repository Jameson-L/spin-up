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
    // if (flywheel.getActualVelocity() < speed-50) {
      flywheel.controllerSet(1);
    // } else {
      // flywheel.moveVelocity(speed);
    // }
  }
  flywheel.controllerSet(0);
}

void rightRoller() {
  relative(-2, 0.5, 2);
}

void right() {
  speed = 600;
  pros::Task startFlywheel(flywheelTask);
  intake.controllerSet(1);
  jCurve(1.5, 0, true, 0, 1, 2);
  turnToPoint(10, 4.3);
  relative(1, 1, 0.3);
  pros::delay(200);
  pros::c::adi_digital_write(kPneumaticIndexerPort, HIGH);
  pros::delay(300);
  pros::c::adi_digital_write(kPneumaticIndexerPort, LOW);
  pros::delay(800);
  intake.controllerSet(0);
  pros::c::adi_digital_write(kPneumaticIndexerPort, HIGH);
  pros::delay(300);
  pros::c::adi_digital_write(kPneumaticIndexerPort, LOW);
  pros::delay(800);
  pros::c::adi_digital_write(kPneumaticIndexerPort, HIGH);
  pros::delay(300);
  pros::c::adi_digital_write(kPneumaticIndexerPort, LOW);
  relative(-0.9, 1, 0.3);
  intake.controllerSet(1);
  odomDriveToPoint(4.5, -3, true, 0.5, 0.8, 2);
  turnToPoint(10, 4.3);
  relative(1, 1, 0.3);
  pros::delay(200);
  pros::c::adi_digital_write(kPneumaticIndexerPort, HIGH);
  pros::delay(300);
  pros::c::adi_digital_write(kPneumaticIndexerPort, LOW);
  pros::delay(800);
  intake.controllerSet(0);
  pros::c::adi_digital_write(kPneumaticIndexerPort, HIGH);
  pros::delay(300);
  pros::c::adi_digital_write(kPneumaticIndexerPort, LOW);
  relative(-0.9, 1, 0.3);
  continueFlywheel = false;
  odomDriveToPoint(0.8, 2.5, false, 0, 1, 3);
  imuTurnToAngle(0);
  pros::Task rollerTask(rightRoller);
  pros::delay(750);
  intake.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  intake.moveRelative(-300, 600);
}

void left() {
  // pros::Task intakeTask1(intakeTask2);
  // pros::delay(700);
  // flywheel.controllerSet(0.9);
  // intake.controllerSet(-1);
  // pros::delay(620);
  // intake.controllerSet(0);
  // pros::delay(680);
  // relative(-0.5, 0.7);
  // bool thing = chassis->getState().x.convert(okapi::foot) >= 0;
  // odomDriveToPoint(0, 0, thing, 0, 0.5, 1);
  // imuTurnToAngle(165);
  // pros::delay(300);
  // pros::c::adi_digital_write(kPneumaticIndexerPort, HIGH);
  // pros::delay(500);
  // pros::c::adi_digital_write(kPneumaticIndexerPort, LOW);
  // pros::delay(1000);
  // pros::c::adi_digital_write(kPneumaticIndexerPort, HIGH);
  // pros::delay(500);
  // pros::c::adi_digital_write(kPneumaticIndexerPort, LOW);
  // flywheel.controllerSet(0);
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
