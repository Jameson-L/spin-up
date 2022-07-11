#include "main.h"

#include "autonomous/odometry.hpp"
#include "subsystems/chassis.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/flywheel.hpp"
#include "subsystems/pneumatics.hpp"
#include "autonomous/autonomous.hpp"

// units
// using namespace okapi::literals;

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	// okapi::Logger::setDefaultLogger(
  //   std::make_shared<okapi::Logger>(
  //       okapi::TimeUtilFactory::createDefault().getTimer(), // It needs a Timer
  //       "/ser/sout", // Output to the PROS terminal
  //       okapi::Logger::LogLevel::debug // Show errors and warnings
  //   )
	// );

	// creating logger
	// auto logger = okapi::Logger::getDefaultLogger();

	// default initialization example
	okapi::Rate rate;
	pros::lcd::initialize();
	pros::lcd::set_text(1, "This initialization is dedicated to Shubham Kumar.");
	pros::lcd::register_btn1_cb(on_center_button);

	pros::c::adi_pin_mode(kPneumaticIndexerPort, OUTPUT);
	pros::c::adi_pin_mode(kPneumaticExpansionPort, OUTPUT);

	// messages
	// LOG_DEBUG_S("Initializing...");
	// LOG_DEBUG_S("Initialization Complete.");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	okapi::MotorGroup allMotors({kDriveLIPort, kDriveLOPort, kDriveLBPort, kDriveRBPort, kDriveRIPort, kDriveROPort});
	allMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */


void opcontrol() {
	// OUR CODE:
	// creating logger
	auto logger = okapi::Logger::getDefaultLogger();
	okapi::Controller controller;
	okapi::Controller controller2 = okapi::Controller(okapi::ControllerId::partner);
	okapi::Timer timer;

	// power variables
	double leftY;
	double rightY;
	bool flywheelToggle = false;

	// bool reverseDrive = false;

	// okapi::MotorGroup allMotors({kDriveLTPort, kDriveLMPort, kDriveLBPort, kDriveRBPort, kDriveRMPort, kDriveRTPort});
	okapi::Rate rate;

	// rate.delay(40_Hz);

	// vision.set_signature(1, &NEUTRAL);
	// powershare.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

	while (true) {
		// std::cout <<"running op control" << "\n";

		// printing odometry tests
		// okapi::OdomState pos = chassis->getState();
		// pros::lcd::set_text(1, std::to_string(pos.x.convert(okapi::foot)));

		// std::cout << "left: " << LTrackingWheel.controllerGet() << '\n';
		// std::cout << "right: " << RTrackingWheel.controllerGet() << '\n';
		// std::cout << "middle: " << MTrackingWheel.controllerGet() << '\n';
		// std::cout << "x-pos: " << pos.x.convert(okapi::foot) << '\n';
		// std::cout << "y-pos: " << pos.y.convert(okapi::foot) << '\n';
		// std::cout << "theta: " << pos.theta.convert(okapi::degree) << ' ';
		// std::cout<< "imu: " << getHeading() << '\n';
		// std::cout << allMotors.getEfficiency() << "\n";

		if (controller[okapi::ControllerDigital::R1].changedToPressed()) {
			flywheelToggle = !flywheelToggle;
		}
		if (flywheelToggle) {
			flywheel.moveVelocity(600);
		} else {
			flywheel.moveVelocity(0);
		}
		if (controller[okapi::ControllerDigital::R2].isPressed()) {
			pros::c::adi_digital_write(kPneumaticIndexerPort, HIGH);
		} else {
			pros::c::adi_digital_write(kPneumaticIndexerPort, LOW);
		}
		if (controller[okapi::ControllerDigital::L1].isPressed()) {
			intake.controllerSet(1);
		} else {
			intake.controllerSet(0);
		}
		if (controller[okapi::ControllerDigital::L2].isPressed()) {
			intake.controllerSet(-1);
		} else {
			intake.controllerSet(0);
		}
		if (controller[okapi::ControllerDigital::L1].isPressed()
		&& controller[okapi::ControllerDigital::L2].isPressed()
		&& controller[okapi::ControllerDigital::R1].isPressed()
		&& controller[okapi::ControllerDigital::R2].isPressed()) {
			pros::c::adi_digital_write(kPneumaticIndexerPort, HIGH);
		}
		if (controller[okapi::ControllerDigital::X].isPressed()) {
			intake.controllerSet(0.1);
		} else {
			intake.controllerSet(0);
		}


		// set power variables
		leftY = controller.getAnalog(okapi::ControllerAnalog::leftY);
		rightY = controller.getAnalog(okapi::ControllerAnalog::rightY);
		chassis->getModel()->tank(leftY, rightY);

		rate.delay(100_Hz);
	}
}
