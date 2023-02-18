#include "autonomous/odometry.hpp"
#include "subsystems/chassis.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/flywheel.hpp"
#include "subsystems/pneumatics.hpp"
#include "autonomous/autonomous.hpp"

bool continueFlywheel = false;
double speed = 600;
double tbhGain = 0.00002;// tune this, try 0.00003 LOWER TARGET SPEED = HIGHER TBHGAIN


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

  // double error;
	// double prevError = 1;
	// double output = 0;
	// double tbh = speed / 600.0; // maybe tune this, unlikely

  while (continueFlywheel) {
    // error = speed - flywheel.getActualVelocity();
    // output += tbhGain * error;
    // if (signbit(error) != signbit(prevError)) {
    //   output = 0.5 * (output + tbh);
    //   tbh = output;
    //   prevError = error;
    // }
    //
    // if (flywheel.getActualVelocity() < speed - 30) {
    //   flywheel.controllerSet(1);
    // } else {
    //   flywheel.controllerSet(output);
    // }
    if (flywheel.getActualVelocity() < speed - 100) {
      flywheel.controllerSet(1);
    } else {
      flywheel.controllerSet(
        // -1 * (1 - targetSpeed / 600) * 0.01 * flywheel.getActualVelocity()
        // + (-1 * targetSpeed * targetSpeed + 700 * targetSpeed) / 60000.0
        speed / 600.0
      );
      // std::cout << flywheel.getActualVelocity() << " " << -1 * (1 - targetSpeed / 600) * 0.01 * flywheel.getActualVelocity()
      // + (-1 * targetSpeed * targetSpeed + 700 * targetSpeed) / 60000.0 << "\n";
    }
    rate.delay(100_Hz);
  }
}

void awaitFlywheel() {
  okapi::Timer timer;
  while (flywheel.getActualVelocity() <= 570 && timer.millis().convert(okapi::second) < 1.5) {
  }
}

void right() {
  // starting flywheel
  speed = 600;
  pros::Task startFlywheel(flywheelTask);

  // first disc and aim
  intake.controllerSet(1);
  odomDriveToPoint(1.8, 0, true, 0.2, 1, 1);
  imuTurnToAngle(30);

  // shoot 3
  intake.controllerSet(0);
  pros::delay(1450);
  // awaitFlywheel();
  // intake.moveRelative(-235, 600);
  // pros::delay(1500);
  for (int i = 0; i < 3; i++) {
    // awaitFlywheel();
    intake.moveRelative(-235, 600);
    pros::delay(900);
  }
  // pros::delay(200);

  // line of 2 discs and aim
  intake.controllerSet(1);
  odomDriveToPoint(4.7, -3, true, 1.4, 1, 1.5);
  imuTurnToAngle(54);
  pros::delay(500);

  // shoot 2
  for (int i = 0; i < 2; i++) {
    // awaitFlywheel();
    intake.controllerSet(0);
    intake.moveRelative(-235, 600);
    pros::delay(900);
  }
  // pros::delay(200);
  continueFlywheel = false;

  // to roller
  odomDriveToPoint(0, 2.1, false, 0, 1, 1.6);

  // roll
  imuTurnToAngle(0);

  intake.controllerSet(1);
  relative(-2, 1, 0.4);
  leftMotors.controllerSet(-0.25);
  rightMotors.controllerSet(-0.5);
  // intake.moveRelative(375, 600);
  pros::delay(300);
  intake.controllerSet(0);
  allMotors.controllerSet(0);
}

void left() {
  // start flywheel
  speed = 600;
  pros::Task startFlywheel(flywheelTask);

  // roller
  intake.controllerSet(1);
  pros::delay(100);
  relative(-2, 1, 0.1);
  // leftMotors.controllerSet(-0.25);
  // rightMotors.controllerSet(-0.5);
  // intake.moveRelative(375, 600);
  pros::delay(200);
  intake.controllerSet(0);
  allMotors.controllerSet(0);

  // aim
  odomDriveToPoint(1.3, 0.5, true, 0, 1, 1);
  imuTurnToAngle(-15);
  pros::delay(1500);

  // shoot 3
  for (int i = 0; i < 2; i++) {
    // awaitFlywheel();
    intake.moveRelative(-235, 600);
    pros::delay(1100);
  }

  // stack of discs and aim
  intake.controllerSet(-1);
  odomDriveToPoint(2, 3.5, true, 1.6, 0.6, 0.5);
  intake.controllerSet(1);
  odomDriveToPoint(2, 3.5, true, 0, 0.3, 3.5);
  imuTurnToAngle(-37);
  relative(1.7, 1, 1);
  intake.controllerSet(0);
  pros::delay(200);

  for (int i = 0; i < 3; i++) {
    // awaitFlywheel();
    intake.moveRelative(-235, 600);
    pros::delay(1100);
  }
  continueFlywheel = false;
}

void awp() {
}
