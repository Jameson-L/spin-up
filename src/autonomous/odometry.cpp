#include "subsystems/chassis.hpp"
#include "autonomous/odometry.hpp"

using namespace okapi::literals;

okapi::Rate rate;

okapi::ADIEncoder LTrackingWheel = okapi::ADIEncoder({2, 0, 0}, false);
okapi::ADIEncoder RTrackingWheel = okapi::ADIEncoder({2, 0, 0}, false);
// okapi::ADIEncoder MTrackingWheel = okapi::ADIEncoder(0, 0, true);

okapi::IMU imu1 = okapi::IMU(9, okapi::IMUAxes::z);
okapi::IMU imu2 = okapi::IMU(0, okapi::IMUAxes::x);

okapi::IterativePosPIDController chassisTurnPid = okapi::IterativeControllerFactory::posPID(0.029, 0.0, 0.001);
okapi::IterativePosPIDController chassisDrivePid = okapi::IterativeControllerFactory::posPID(0.55, 0.01, 0.02);
okapi::IterativePosPIDController chassisSwingPid = okapi::IterativeControllerFactory::posPID(0.25, 0.0, 0.0025);

double getHeading(bool safe) {
  if (!safe) {
    return (imu1.controllerGet() + imu1.controllerGet()) / 2.0;
  } else {
    return std::fmod((getHeading(false) + 360), 360);
  }
}

bool isMoving() {
  return abs(okapi::Motor(kDriveLBPort).getActualVelocity()) +
  abs(okapi::Motor(kDriveLIPort).getActualVelocity()) +
  abs(okapi::Motor(kDriveLOPort).getActualVelocity()) +
  abs(okapi::Motor(kDriveRBPort).getActualVelocity()) +
  abs(okapi::Motor(kDriveRIPort).getActualVelocity()) +
  abs(okapi::Motor(kDriveROPort).getActualVelocity()) > 10;
};

void imuTurnToAngle(double deg) {
  // okapi::MotorGroup allMotors({kDriveLTPort, kDriveLMPort, kDriveLBPort, kDriveRBPort, kDriveRMPort, kDriveRTPort});
  // allMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
  bool safe = deg < -150 || deg > 150;
  if (deg < -150) {
    deg += 360;
  }
  chassisTurnPid.setTarget(deg);

  double chassisPidValue;

  okapi::Timer timer;
  double init = timer.millis().convert(okapi::second);

  while (!(abs(deg - getHeading(safe)) < 4 && !isMoving())) { // test issettled
    // std::cout << "in progress" << '\n';
    if (timer.millis().convert(okapi::second) - init > 2) {
      break;
    }
    // std::cout << getHeading() << "\n";
    // std::cout << getHeading(safe) << " " << chassisPidValue << "\n";
    chassisPidValue = chassisTurnPid.step(getHeading(safe));
    chassis->getModel()->tank(chassisPidValue, -1*chassisPidValue);
    rate.delay(100_Hz);
  }
  chassisTurnPid.reset();
  chassis->getModel()->tank(0, 0);
  // allMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  chassis->setState({chassis->getState().x, chassis->getState().y, getHeading(false) * okapi::degree});

  // std::cout << "turn complete" << '\n';
}

void imuZeroToAngle(double deg, double time){
  chassisTurnPid.setTarget(deg);
  double chassisPidValue;
  okapi::Timer timer;
  double init = timer.millis().convert(okapi::second);
  while (!(abs(deg - getHeading(false)) < 5 && !isMoving())) { // test issettled
    // std::cout << "in progress" << '\n';
    // std::cout << getHeading(false) << "\n";
    if (timer.millis().convert(okapi::second) - init > time) {
      break;
    }
    chassisPidValue = chassisTurnPid.step(getHeading(false));
    chassis->getModel()->tank(chassisPidValue, 0);
    rate.delay(100_Hz);
  }
  chassisTurnPid.reset();
  chassis->getModel()->tank(0, 0);
  chassis->setState({chassis->getState().x, chassis->getState().y, getHeading(false) * okapi::degree});
}

void odomDriveToPoint(double x, double y, bool forward, double offset, double speedMultiplier, double time) { // in feet, x is forward, y is sideways
  double copyX = x;
  double copyY = y;
  // turn first
  x -= chassis->getState().x.convert(okapi::foot); // displacement x
  y -= chassis->getState().y.convert(okapi::foot); // displacement y
  double angle = atan(y / x) * 180 / M_PI; // absolute angle
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
  imuTurnToAngle(angle);
  jCurve(copyX, copyY, forward, offset, speedMultiplier, time);
}

void jCurve(double x, double y, bool forward, double offset, double speedMultiplier, double time) { // in feet, x is forward, y is sideways
  // std::cout << vision.get_object_count();

  double copyX = x;
  double copyY = y;
  okapi::MedianFilter<5> visionFilter;

  // pythagorean theorem for distance to travel
  double target = sqrt(powf(copyX-chassis->getState().x.convert(okapi::foot), 2) + powf(copyY-chassis->getState().y.convert(okapi::foot), 2));
  target -= offset;
  if (!forward) {
    target *= -1; // go backwards
  }
  chassisDrivePid.setTarget(target);

  double chassisPidValue = 0; // drive
  double chassisPidValue2; // turn
  double dX, dY; // current displacement
  double encoderReading = 1000000000.0;
  double modified;

  // initial x and y to calculate displacement
  double startX = chassis->getState().x.convert(okapi::foot);
  double startY = chassis->getState().y.convert(okapi::foot);

  okapi::Timer timer;
  double init = timer.millis().convert(okapi::second);
  // okapi::MotorGroup allMotors({kDriveLIPort, kDriveLOPort, kDriveLBPort, kDriveRBPort, kDriveRIPort, kDriveROPort});

  while (!(abs(target - encoderReading) < 0.25 && !isMoving())) {
    chassis->setState({chassis->getState().x, chassis->getState().y, getHeading(false) * okapi::degree});
    if (timer.millis().convert(okapi::second) - init > time) {
      break; // otherwise, just break normally
    }
    x = copyX - chassis->getState().x.convert(okapi::foot); // displacement x
    y = copyY - chassis->getState().y.convert(okapi::foot); // displacement y

    double angle = atan(y / x) * 180 / M_PI; // absolute angle
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
    bool safe = angle < -150 || angle > 150;
    if (angle < -150) {
      angle += 360;
    }
    chassisTurnPid.setTarget(angle);

    chassisPidValue2 = chassisTurnPid.step(getHeading(safe));
    dX = chassis->getState().x.convert(okapi::foot) - startX;
    dY = chassis->getState().y.convert(okapi::foot) - startY;
    encoderReading = sqrt(powf(dX, 2) + powf(dY, 2)); // displacement hypotenuse
    if (!forward) {
      encoderReading *= -1;
    }
    // std::cout << encoderReading << " " << target << "\n";

    chassisPidValue = chassisDrivePid.step(encoderReading);

    if (abs(chassisPidValue) > abs(speedMultiplier)) {
      if (chassisPidValue > 0) {
        modified = speedMultiplier;
      } else {
        modified = speedMultiplier * -1;
      }
    } else {
      modified = chassisPidValue;
    }


    chassis->getModel()->tank(modified + chassisPidValue2*0.8*abs(modified), modified - chassisPidValue2*0.8*abs(modified));
    // chassis->getModel()->tank(chassisPidValue + chassisPidValue2*0.9, chassisPidValue - chassisPidValue2*0.9);
    rate.delay(100_Hz);
    }
  chassisDrivePid.reset();
  chassisTurnPid.reset();
  chassis->getModel()->tank(0, 0);
  chassis->setState({chassis->getState().x, chassis->getState().y, getHeading(false) * okapi::degree});
  // std::cout << "target: " << copyX << ", " << copyY << "\n";
  // std::cout << "actual: " << chassis->getState().x.convert(okapi::foot) << ", " << chassis->getState().y.convert(okapi::foot) << "\n";
}

void relative(double x, double time) { // in feet, x is forward, y is sideways
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
    chassis->getModel()->tank(chassisPidValue, chassisPidValue);
    if (timer.millis().convert(okapi::second) - init > time) {
      break;
    }

    rate.delay(100_Hz);
  }

  chassisDrivePid.reset();
  chassis->getModel()->tank(0, 0);
}
