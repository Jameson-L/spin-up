#include "subsystems/chassis.hpp"
#include "autonomous/odometry.hpp"

// units
using namespace okapi::literals;

// defining chassis ports, negative is reversed
// i is inner, o is outer
const int8_t kDriveLBPort = -20;
const int8_t kDriveLIPort = 0;
const int8_t kDriveLOPort = -15;
const int8_t kDriveRBPort = 14;
const int8_t kDriveRIPort = -16;
const int8_t kDriveROPort = 2;

// creating chassis object
std::shared_ptr<okapi::OdomChassisController> chassis = okapi::ChassisControllerBuilder()
  .withMotors(
    {kDriveLBPort, kDriveLIPort, kDriveLOPort},
    {kDriveRBPort, kDriveRIPort, kDriveROPort}
  )
  .withDimensions(okapi::AbstractMotor::gearset::blue, okapi::ChassisScales({3.25_in, 10_in}, okapi::imev5GreenTPR * 5./3.)) // driving/driven
  // .withSensors(LTrackingWheel, RTrackingWheel/*, MTrackingWheel*/)
  .withOdometry(/*{{2.75_in, 6.75_in}, okapi::quadEncoderTPR}*/)
  .buildOdometry();
