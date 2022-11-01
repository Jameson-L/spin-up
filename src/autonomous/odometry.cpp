#include "subsystems/chassis.hpp"
#include "autonomous/odometry.hpp"

using namespace okapi::literals; // for units namespace

okapi::Rate rate; // for consistent rate of loops

// okapi::ADIEncoder LTrackingWheel = okapi::ADIEncoder({2, 0, 0}, false);
// okapi::ADIEncoder RTrackingWheel = okapi::ADIEncoder({2, 0, 0}, false);
// okapi::ADIEncoder MTrackingWheel = okapi::ADIEncoder(0, 0, true);

okapi::IMU imu1 = okapi::IMU(9, okapi::IMUAxes::z);
// okapi::IMU imu2 = okapi::IMU(0, okapi::IMUAxes::x);

// pid constants
okapi::IterativePosPIDController chassisTurnPid = okapi::IterativeControllerFactory::posPID(0.025, 0.0, 0.001);
okapi::IterativePosPIDController chassisDrivePid = okapi::IterativeControllerFactory::posPID(0.55, 0.01, 0.02);
okapi::IterativePosPIDController chassisSwingPid = okapi::IterativeControllerFactory::posPID(0.25, 0.0, 0.0025);

double getHeading(bool safe) {
  if (!safe) {
    return (imu1.controllerGet() + imu1.controllerGet()) / 2.0; // average of the two, but only using one for now
  } else {
    return std::fmod((getHeading(false) + 360), 360); // mapping the readings to a safer interval
  }
}

bool isMoving() {
  // not using allMotors motorgroup because both sides of drive moving in opposite direction might cancel the overall rpm
  return
  abs(okapi::Motor(kDriveLFPort).getActualVelocity()) +
  abs(okapi::Motor(kDriveLMPort).getActualVelocity()) +
  abs(okapi::Motor(kDriveLBPort).getActualVelocity()) +
  abs(okapi::Motor(kDriveRFPort).getActualVelocity()) +
  abs(okapi::Motor(kDriveRMPort).getActualVelocity()) +
  abs(okapi::Motor(kDriveRBPort).getActualVelocity()) > 10;
}

void imuTurnToAngle(double deg) {
  bool safe = deg < -150 || deg > 150; // checking if degree is dangerous
  if (deg < -150) {
    deg += 360; // remapping
  }
  chassisTurnPid.setTarget(deg); // pid target

  double chassisPidValue; // temporary variable to store pid value
  okapi::Timer timer; // timing to ensure it doesnt take too long
  double init = timer.millis().convert(okapi::second); // saving initial time to calculate time elapsed

  while (!(abs(deg - getHeading(safe)) < 4 && !isMoving())) { // if close enough and stopped moving
    if (timer.millis().convert(okapi::second) - init > 2) {
      break; // break if too long
    }

    chassisPidValue = chassisTurnPid.step(getHeading(safe)); // stepping the pid with current reading
    chassis->getModel()->tank(chassisPidValue, -1*chassisPidValue); // sending power to the drivetrain
    rate.delay(100_Hz); // need consistent timing for pid loop
  }
  chassisTurnPid.reset(); // resetting pid
  chassis->getModel()->tank(0, 0); // stopping the bot completely

  chassis->setState({chassis->getState().x, chassis->getState().y, getHeading(false) * okapi::degree});
  // setting current degree to imu reading to minimize odometry drift
}

void odomDriveToPoint(double x, double y, bool forward, double offset, double speedMultiplier, double time) {
  // storing initial x and y for later jCurve call
  double targetX = x;
  double targetY = y;

  // initial in-place turn:
  x -= chassis->getState().x.convert(okapi::foot); // getting target displacement x
  y -= chassis->getState().y.convert(okapi::foot); // getting target displacement y

  double angle = atan(y / x) * 180 / M_PI; // absolute angle needed to turn
  if (forward) { // tangent function domain/range checking and corrections
    if (x < 0 && y > 0) {
      angle += 180;
    } else if (x < 0 && y < 0) {
      angle -= 180;
    }
  } else {
    if (x > 0 && y > 0) {
      angle -= 180;
    } else if (x > 0 && y < 0) {
      angle += 180;
    }
  }

  imuTurnToAngle(angle); // turn to the desired angle
  jCurve(targetX, targetY, forward, offset, speedMultiplier, time); // let this function handle the rest
}

void jCurve(double x, double y, bool forward, double offset, double speedMultiplier, double time) {
  // storing initial target x and y for later
  double targetX = x;
  double targetY = y;

  // pythagorean theorem for distance to travel
  double target = sqrt(powf(targetX-chassis->getState().x.convert(okapi::foot), 2) + powf(targetY-chassis->getState().y.convert(okapi::foot), 2));
  target -= offset; // to stop earlier
  if (!forward) {
    target *= -1; // go backwards
  }
  chassisDrivePid.setTarget(target); // setting pid target

  double drivePidValue = 0; // drive
  double turnPidValue = 0; // turn
  double turnStrength = 0.7; // how sharp the turns can be; bigger = turn more
  double dX, dY; // current displacement
  double encoderReading = 1000000000.0; // arbitrarily large starting value so the while loop can start without breaking due to unitialized variable
  double finalDriveValue; // drive value after speed calculations etc.

  // initial x and y to calculate displacement
  double startX = chassis->getState().x.convert(okapi::foot);
  double startY = chassis->getState().y.convert(okapi::foot);

  okapi::Timer timer; // timer to not take too long
  double init = timer.millis().convert(okapi::second); // storing initial time to calculate elapsed time

  while (!(abs(target - encoderReading) < 0.25 && !isMoving())) { // if distance small enough and stopped moving
    // setting odometry angle to imu reading to minimize drift
    chassis->setState({chassis->getState().x, chassis->getState().y, getHeading(false) * okapi::degree});
    if (timer.millis().convert(okapi::second) - init > time) {
      break; // break if too long
    }
    x = targetX - chassis->getState().x.convert(okapi::foot); // target displacement x
    y = targetY - chassis->getState().y.convert(okapi::foot); // target displacement y

    // calculating desired angle during the current iteration
    double angle = atan(y / x) * 180 / M_PI; // absolute angle to the point
    // tangent function math
    if (forward) {
      if (x < 0 && y > 0) {
        angle += 180;
      } else if (x < 0 && y < 0) {
        angle -= 180;
      }
    } else {
      if (x > 0 && y > 0) {
        angle -= 180;
      } else if (x > 0 && y < 0) {
        angle += 180;
      }
    }
    // if angle is dangerous (180 jumping to -180), remap to safer interval
    bool safe = angle < -150 || angle > 150;
    if (angle < -150) {
      angle += 360;
    }
    chassisTurnPid.setTarget(angle); // angle pid target
    turnPidValue = chassisTurnPid.step(getHeading(safe)); // stepping the turn pid based on current reading

    // calculating current distance to target:
    dX = chassis->getState().x.convert(okapi::foot) - startX; // current displacement x
    dY = chassis->getState().y.convert(okapi::foot) - startY; // current displacement y
    encoderReading = sqrt(powf(dX, 2) + powf(dY, 2)); // displacement hypotenuse, acts as current drive movement reading
    if (!forward) {
      encoderReading *= -1; // go backwards so negative distance
    }
    drivePidValue = chassisDrivePid.step(encoderReading); // stepping drive pid based on displacement

    // speed modifications:
    if (abs(drivePidValue) > abs(speedMultiplier)) { // limit to max speed
      if (drivePidValue > 0) { // < and > flip for negatives
        finalDriveValue = speedMultiplier;
      } else {
        finalDriveValue = speedMultiplier * -1;
      }
    } else {
      finalDriveValue = drivePidValue;
    }

    // sending power to the drivetrain:
    chassis->getModel()->tank(
      finalDriveValue + turnPidValue*turnStrength*abs(finalDriveValue), // left
      finalDriveValue - turnPidValue*turnStrength*abs(finalDriveValue)); // right
    // turn cannot be stronger than the drivePid to avoid turning while the bot isn't trying to translate
    rate.delay(100_Hz); // necessary consistent rate for pid loop
  }

  // resetting pids
  chassisDrivePid.reset();
  chassisTurnPid.reset();
  chassis->getModel()->tank(0, 0); // stopping the bot completely

  // one more odom angle correction
  chassis->setState({chassis->getState().x, chassis->getState().y, getHeading(false) * okapi::degree});
}

void relative(double x, double speedMultiplier, double time) {
  // refer to jCurve documentation
  double target = x;
  chassisDrivePid.setTarget(target);

  double chassisPidValue;
  double dX, dY; // current displacement
  double encoderReading = 1000000000.0;

  // initial x and y to calculate displacement
  double startX = chassis->getState().x.convert(okapi::foot);
  double startY = chassis->getState().y.convert(okapi::foot);

  okapi::Timer timer;
  double init = timer.millis().convert(okapi::second);
  double finalDriveValue = 0;

  while (!(abs(target - encoderReading) < 0.25 && !isMoving())) {
    chassis->setState({chassis->getState().x, chassis->getState().y, getHeading(false) * okapi::degree});
    dX = chassis->getState().x.convert(okapi::foot) - startX;
    dY = chassis->getState().y.convert(okapi::foot) - startY;
    encoderReading = sqrt(powf(dX, 2) + powf(dY, 2)); // displacement hypotenuse
    if (x < 0) {
      encoderReading *= -1;
    }
    chassisPidValue = chassisDrivePid.step(encoderReading);
    // std::cout << encoderReading << " " << target << "\n";

    if (abs(chassisPidValue) > abs(speedMultiplier)) {
      if (chassisPidValue > 0) {
        finalDriveValue = speedMultiplier;
      } else {
        finalDriveValue = speedMultiplier * -1;
      }
    } else {
      finalDriveValue = chassisPidValue;
    }
    chassis->getModel()->tank(finalDriveValue, finalDriveValue);
    if (timer.millis().convert(okapi::second) - init > time) {
      break;
    }

    rate.delay(100_Hz);
  }

  chassisDrivePid.reset();
  chassis->getModel()->tank(0, 0);
}
