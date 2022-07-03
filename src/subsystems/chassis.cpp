#include "subsystems/chassis.hpp"
#include "autonomous/odometry.hpp"

// units
using namespace okapi::literals;

// defining chassis ports, negative is reversed
// i is inner, o is outer
const int8_t kDriveLBPort = -16;
const int8_t kDriveLIPort = -19;
const int8_t kDriveLOPort = 11;
const int8_t kDriveRBPort = 20;
const int8_t kDriveRIPort = 12;
const int8_t kDriveROPort = -15;

// creating chassis object
std::shared_ptr<okapi::OdomChassisController> chassis = okapi::ChassisControllerBuilder()
  .withMotors(
    {kDriveLBPort, kDriveLIPort, kDriveLOPort},
    {kDriveRBPort, kDriveRIPort, kDriveROPort}
  )
  .withDimensions(okapi::AbstractMotor::gearset::green, okapi::ChassisScales({3.25_in, 13._in}, okapi::imev5GreenTPR * 3./5.))
  .withSensors(LTrackingWheel, RTrackingWheel/*, MTrackingWheel*/)
  .withOdometry({{2.75_in, 6.75_in}, okapi::quadEncoderTPR})
  .buildOdometry();
