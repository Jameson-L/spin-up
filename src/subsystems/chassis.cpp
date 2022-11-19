#include "subsystems/chassis.hpp"
#include "autonomous/odometry.hpp"

// units
using namespace okapi::literals;

// defining chassis ports, negative is reversed
// i is inner, o is outer
const int8_t kDriveLFPort = -9;
const int8_t kDriveLMPort = -7;
const int8_t kDriveLBPort = -1;
const int8_t kDriveRFPort = 12;
const int8_t kDriveRMPort = 6;
const int8_t kDriveRBPort = 2;

// creating chassis object
std::shared_ptr<okapi::OdomChassisController> chassis = okapi::ChassisControllerBuilder()
  .withMotors(
    {kDriveLFPort, kDriveLMPort, kDriveLBPort},
    {kDriveRFPort, kDriveRMPort, kDriveRBPort}
  )
  .withDimensions(okapi::AbstractMotor::gearset::blue, okapi::ChassisScales({3.25_in, 10_in}, okapi::imev5BlueTPR * 5./3.)) // driving/driven
  // .withSensors(LTrackingWheel, RTrackingWheel/*, MTrackingWheel*/)
  .withOdometry(/*{{2.75_in, 6.75_in}, okapi::quadEncoderTPR}*/)
  .buildOdometry();

// motor group of all motors
okapi::MotorGroup allMotors = okapi::MotorGroup({kDriveLFPort, kDriveLMPort, kDriveLBPort, kDriveRFPort, kDriveRMPort, kDriveRBPort});
