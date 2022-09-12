#include "autonomous/odometry.hpp"
#include "subsystems/chassis.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/flywheel.hpp"
#include "subsystems/pneumatics.hpp"
#include "autonomous/autonomous.hpp"

// functions
okapi::MotorGroup allMotors = okapi::MotorGroup({kDriveLBPort, kDriveLIPort, kDriveLOPort,
kDriveRBPort, kDriveRIPort, kDriveROPort});

void intakeTask() {
  relative(1.5, 0.2, 2.5);
}
void intakeTask2() {
  relative(1.4, 0.2, 2.5);
}

void right() {
  relative(1.9, 1);
  imuTurnToAngle(90);
  pros::Task intakeTask1(intakeTask);
  pros::delay(600);
  intake.controllerSet(-1);
  flywheel.controllerSet(0.85);
  pros::delay(760);
  intake.controllerSet(0);
  pros::delay(640);
  relative(-0.5, 0.7);
  bool thing = chassis->getState().y.convert(okapi::foot) >= 0.3;
  odomDriveToPoint(1.5, -0.3, thing, 0, 0.5, 1);
  imuTurnToAngle(-90);
  pros::c::adi_digital_write(kPneumaticIndexerPort, HIGH);
  pros::delay(500);
  pros::c::adi_digital_write(kPneumaticIndexerPort, LOW);
  pros::delay(1250);
  pros::c::adi_digital_write(kPneumaticIndexerPort, HIGH);
  pros::delay(500);
  pros::c::adi_digital_write(kPneumaticIndexerPort, LOW);
  flywheel.controllerSet(0);
}


void left() {
  pros::Task intakeTask1(intakeTask2);
  pros::delay(700);
  flywheel.controllerSet(0.9);
  intake.controllerSet(-1);
  pros::delay(620);
  intake.controllerSet(0);
  pros::delay(680);
  relative(-0.5, 0.7);
  bool thing = chassis->getState().x.convert(okapi::foot) >= 0;
  odomDriveToPoint(0, 0, thing, 0, 0.5, 1);
  imuTurnToAngle(165);
  pros::delay(300);
  pros::c::adi_digital_write(kPneumaticIndexerPort, HIGH);
  pros::delay(500);
  pros::c::adi_digital_write(kPneumaticIndexerPort, LOW);
  pros::delay(1000);
  pros::c::adi_digital_write(kPneumaticIndexerPort, HIGH);
  pros::delay(500);
  pros::c::adi_digital_write(kPneumaticIndexerPort, LOW);
  flywheel.controllerSet(0);
}

void awpLeft() {
  left();
  odomDriveToPoint(-6.52, -7.35, true, 0, 1, 4);
  imuTurnToAngle(-90);
  pros::Task intakeTask1(intakeTask);
  pros::delay(250);
  intake.controllerSet(-1);
  pros::delay(710);
  intake.controllerSet(0);
}
