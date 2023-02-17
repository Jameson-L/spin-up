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
  while (flywheel.getActualVelocity() <= 590) {
  }
}

void right() {
  // starting flywheel
  speed = 600;
  pros::Task startFlywheel(flywheelTask);

  // first disc and aim
  intake.controllerSet(1);
  odomDriveToPoint(1.8, 0, true, 0.2, 1, 1.5);
  imuTurnToAngle(30);

  // shoot 3
  for (int i = 0; i < 3; i++) {
    awaitFlywheel();
    intake.controllerSet(0);
    intake.moveRelative(-245, 600);
    pros::delay(500);
  }
  pros::delay(200);

  // line of 2 discs and aim
  intake.controllerSet(1);
  odomDriveToPoint(4.7, -3, true, 1.4, 1, 2.5);
  imuTurnToAngle(54);

  // shoot 2
  for (int i = 0; i < 2; i++) {
    awaitFlywheel();
    intake.controllerSet(0);
    intake.moveRelative(-245, 600);
    pros::delay(500);
  }
  pros::delay(200);
  continueFlywheel = false;

  // to roller
  odomDriveToPoint(0.2, 2.5, false, 0, 1, 2);

  // roll
  odomDriveToPoint(-0.1, 2.5, false, 0, 1, 0.6);
  leftMotors.moveVelocity(-20);
  rightMotors.moveVelocity(-10);
  intake.moveRelative(375, 600);
  pros::delay(400);
  allMotors.moveVelocity(0);
}

void left() {
  // start flywheel
  speed = 600;
  pros::Task startFlywheel(flywheelTask);

  // roller
  leftMotors.moveVelocity(-20);
  rightMotors.moveVelocity(-10);
  intake.moveRelative(375, 600);
  pros::delay(400);
  allMotors.moveVelocity(0);

  // line of discs and aim
  relative(0.5, 1, 0.7);
  intake.controllerSet(1);
  odomDriveToPoint(2.5, 2.5, 0, 0.5, 4);
  imuTurnToAngle(-40);

  for (int i = 0; i < 3; i++) {
    awaitFlywheel();
    intake.controllerSet(0);
    intake.moveRelative(-245, 600);
    pros::delay(500);
  }
  continueFlywheel = false;
}

// void awp();
