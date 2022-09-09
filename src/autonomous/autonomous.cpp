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

void right() {
  relative(1.9, 1);
  imuTurnToAngle(90);
  pros::Task intakeTask1(intakeTask);
  pros::delay(850);
  intake.controllerSet(-1);
  pros::delay(710);
  intake.controllerSet(0);
  flywheel.controllerSet(0.85);
  relative(-1, 0.7);
  imuTurnToAngle(-92);
  relative(-0.5);
  pros::delay(1000);
  pros::c::adi_digital_write(kPneumaticIndexerPort, HIGH);
  pros::delay(500);
  pros::c::adi_digital_write(kPneumaticIndexerPort, LOW);
  pros::delay(1500);
  pros::c::adi_digital_write(kPneumaticIndexerPort, HIGH);
  pros::delay(500);
  pros::c::adi_digital_write(kPneumaticIndexerPort, LOW);
  flywheel.controllerSet(0);
}


void left() {
  pros::Task intakeTask1(intakeTask);
  pros::delay(800);
  intake.controllerSet(-1);
  pros::delay(710);
  intake.controllerSet(0);
  flywheel.controllerSet(0.9);
  relative(-1, 0.7);
  imuTurnToAngle(164);
  relative(-0.45);
  pros::delay(1000);
  pros::c::adi_digital_write(kPneumaticIndexerPort, HIGH);
  pros::delay(500);
  pros::c::adi_digital_write(kPneumaticIndexerPort, LOW);
  pros::delay(1500);
  pros::c::adi_digital_write(kPneumaticIndexerPort, HIGH);
  pros::delay(500);
  pros::c::adi_digital_write(kPneumaticIndexerPort, LOW);
  flywheel.controllerSet(0);
}
