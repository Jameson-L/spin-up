#include "autonomous/odometry.hpp"
#include "subsystems/chassis.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/flywheel.hpp"
#include "subsystems/pneumatics.hpp"
#include "autonomous/autonomous.hpp"

bool continueFlywheel = false;
double speed = 600;
// double tbhGain = 0.00002;// tune this, try 0.00003 LOWER TARGET SPEED = HIGHER TBHGAIN


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

// void awaitFlywheel() {
//   okapi::Timer timer;
//   while (flywheel.getActualVelocity() <= 570 && timer.millis().convert(okapi::second) < 1.5) {
//   }
// }

void right() {
  // starting flywheel
  imu1.reset();
  speed = 560;
  pros::Task startFlywheel(flywheelTask);

  // first disc and aim
  intake.controllerSet(1);
  odomDriveToPoint(1.8, 0, true, 0.2, 1, 1);
  imuTurnToAngle(31);

  // shoot 3
  intake.controllerSet(0);
  pros::delay(1750);
  // awaitFlywheel();
  // intake.moveRelative(-235, 600);
  // pros::delay(1500);
  for (int i = 0; i < 2; i++) {
    // awaitFlywheel();
    intake.moveRelative(-235, 600);
    pros::delay(850);
  }
  intake.controllerSet(-1);
  // intake.moveRelative(-255, 600);
  pros::delay(900);
  // pros::delay(200);

  // line of 2 discs and aim
  intake.controllerSet(1);
  odomDriveToPoint(4.7, -3, true, 1.4, 1, 1.5);
  imuTurnToAngle(53);
  speed = 510;
  pros::delay(300);
  intake.controllerSet(0);
  pros::delay(500);
  // shoot 2
  for (int i = 0; i < 1; i++) {
    // awaitFlywheel();
    intake.moveRelative(-235, 600);
    pros::delay(600);
  }
  intake.controllerSet(-1);
  // intake.moveRelative(-255, 600);
  pros::delay(600);
  intake.controllerSet(0);
  // pros::delay(200);
  continueFlywheel = false;

  // to roller
  odomDriveToPoint(0, 2, false, 0, 1, 1.6);

  // roll
  imuTurnToAngle(0);

  intake.controllerSet(1);
  relative(-2, 1, 0.4);
  leftMotors.controllerSet(-0.25);
  rightMotors.controllerSet(-0.5);
  // intake.moveRelative(375, 600);
  pros::delay(100);
  intake.controllerSet(0);
  allMotors.controllerSet(0);
}

void stopIntake() {
  pros::delay(500);
  intake.controllerSet(0);
}

void left() {
  // start flywheel
  imu1.reset();
  speed = 540;
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
  imuTurnToAngle(-16);
  pros::delay(1500);

  // shoot 3
  for (int i = 0; i < 1; i++) {
    // awaitFlywheel();
    intake.moveRelative(-235, 600);
    pros::delay(1100);
  }
  intake.controllerSet(-1);
  pros::delay(1100);

  // stack of discs and aim
  // intake.controllerSet(-1);
  speed = 520;
  odomDriveToPoint(2, 3.5, true, 1.6, 0.6, 0.5);
  intake.controllerSet(1);
  odomDriveToPoint(2, 3.5, true, 0, 0.3, 3.5);
  imuTurnToAngle(-37);
  // pros::Task stop(stopIntake);
  relative(1.7, 1, 1);
  intake.controllerSet(0);
  pros::delay(200);

  for (int i = 0; i < 2; i++) {
    // awaitFlywheel();
    intake.moveRelative(-235, 600);
    pros::delay(800);
  }
  intake.controllerSet(-1);
  // intake.moveRelative(-255, 600);
  pros::delay(1000);
  continueFlywheel = false;
}

void awp() {
}

void skills () {
  // start bot straight, initialize, turn towards goal
  // /*
  chassis->setState({0.3_ft, 6.1_ft});
  speed = 450;
  pros::Task startFlywheel(flywheelTask);
  compression2.set_value(1);
  pros::delay(1000);
  compression1.set_value(1);
  compression2.set_value(0);
  intake.controllerSet(-0.3);
  pros::delay(2000);
  intake.controllerSet(0);
  compression1.set_value(0);
  compression2.set_value(1);
  pros::delay(3000);
  compression1.set_value(1);
  compression2.set_value(0);
  intake.controllerSet(-0.3);
  pros::delay(2000);
  intake.controllerSet(0);
  compression1.set_value(0);
  compression2.set_value(1);
  pros::delay(3000);
  compression1.set_value(1);
  compression2.set_value(0);
  intake.controllerSet(-0.3);
  pros::delay(2000);
  intake.controllerSet(0);
  compression1.set_value(0);
  jCurve(1.5, 3, false);
  imuTurnToAngle(0);
  // fastDriveToPoint(1, 3);
  //*/
  // cutoff for no matchloader
  // chassis->setState({1_ft, 3_ft});
  // speed = 480;
  // pros::Task startFlywheel(flywheelTask);
  intake.controllerSet(1);
  relative(-2, 0.5, 0.2);
  relative(-2, 0.2, 0.4);
  jCurve(3.5, 3, true, 0, 0.3);
  jCurve(2.8, 0.8, false);
  imuTurnToAngle(90);
  // relative(-2, 0.7, 0.2);
  relative(-2, 0.2, 0.2);
  // relative(1, 1, 1);
  intake.controllerSet(0);
  jCurve(6, 0.9);
  imuTurnToAngle(10);
  compression1.set_value(1);
  intake.controllerSet(-0.3);
  pros::delay(2000);
  compression1.set_value(0);
  intake.controllerSet(1);
  speed = 460;
  fastDriveToPoint(4.9, 2.6);
  fastDriveToPoint(7.1, 4.9, true, 0, 0.7);
  imuTurnToAngle(-51);
  compression1.set_value(1);
  intake.controllerSet(-0.3);
  pros::delay(2000);
  intake.controllerSet(1);
  compression1.set_value(0);
  // imuTurnToAngle(-20);
  // relative(4, 0.25, 4);
  // imuTurnToAngle(-100);
  // compression1.set_value(1);
  // intake.controllerSet(-1);
  // pros::delay(1200);
  // compression1.set_value(0);
  // intake.controllerSet(1);
  fastDriveToPoint(10.9, 8.5, true, 0, 0.7, 0.7);
  jCurve(10.9, 8.5, true, 0, 0.3);
  imuTurnToAngle(180);
  relative(-2, 0.7, 0.2);
  relative(-2, 0.3, 0.5);
  jCurve(9, 6);
  imuTurnToAngle(-75);
  compression1.set_value(1);
  intake.controllerSet(-0.3);
  pros::delay(2000);
  intake.controllerSet(1);
  compression1.set_value(0);
  jCurve(9.5, 8, false, 0, 1);
  fastDriveToPoint(9.5, 11, true, 0, 0.3);
  imuTurnToAngle(-90);
  intake.controllerSet(1);
  relative(-2, 0.8, 0.5);
  relative(-2, 0.3, 0.5);
  speed = 480;
  intake.controllerSet(0);
  relative(1, 1, 1);
  fastDriveToPoint(6, 10.5, true, 0, 1);
  imuTurnToAngle(180);
  compression1.set_value(1);
  intake.controllerSet(-0.3);
  pros::delay(2000);
  compression1.set_value(0);
  intake.controllerSet(1);
  fastDriveToPoint(7, 9);
  fastDriveToPoint(5, 7);
  imuTurnToAngle(13, 5);
  compression1.set_value(1);
  intake.controllerSet(-0.3);
  pros::delay(2000);
  compression1.set_value(0);
  speed = 600;
  intake.controllerSet(1);
  fastDriveToPoint(2, 4, true, 0, 0.7, 0.7);
  fastDriveToPoint(2, 4, true, 0, 0.3, 0.7);
  imuTurnToAngle(90);
  compression1.set_value(1);
  intake.controllerSet(-0.2);
  pros::delay(2500);
  compression1.set_value(0);
  intake.controllerSet(-0);
  jCurve(2, 2, false);
  imuTurnToAngle(45);
}
