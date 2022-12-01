#include "main.h"
#include "autonomous/odometry.hpp"
#include "subsystems/chassis.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/flywheel.hpp"
#include "subsystems/pneumatics.hpp"
#include "autonomous/autonomous.hpp"

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "klshshsh"); // sus
	pros::lcd::register_btn1_cb(on_center_button);

	// setting pin modes and starting positions
	pros::c::adi_pin_mode(kPneumaticIndexerPort, OUTPUT);
	pros::c::adi_pin_mode(kPneumaticExpansionPort, OUTPUT);
	pros::c::adi_digital_write(kPneumaticExpansionPort, LOW);
	pros::c::adi_digital_write(kPneumaticIndexerPort, LOW);

	// for proper rpm control
	flywheel.setGearing(okapi::AbstractMotor::gearset::blue);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	allMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::hold); // for tighter movements
	flywheel.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	pros::c::adi_digital_write(kPneumaticExpansionPort, LOW); // default position
	pros::c::adi_digital_write(kPneumaticIndexerPort, LOW); // default position
	right();
	// left();
}

void opcontrol() {
	okapi::Controller controller;
	okapi::Timer timer; // in case of timer-based functions for match
	double init  = timer.millis().convert(okapi::second);
	okapi::Rate rate;

	continueFlywheel = false;
	double leftY; // left joystick y direction
	double rightY; // right joystick y direction
	bool flywheelToggle = false; // false = off
	bool expandToggle = false; // false = off
	double targetSpeed = 455; // target speed of flywheel - blue is 600 max
	bool holdDrive = false;
	chassisVisionPid.reset();

	// setting all motors to coast
	allMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	flywheel.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);

	// main loop:
	while (true) {

		// printing odometry tests:
		// okapi::OdomState pos = chassis->getState();
		// pros::lcd::set_text(1, std::to_string(pos.x.convert(okapi::foot)));
		// pros::lcd::set_text(2, std::to_string(pos.y.convert(okapi::foot)));
		// std::cout << "left: " << LTrackingWheel.controllerGet() << '\n';
		// std::cout << "right: " << RTrackingWheel.controllerGet() << '\n';
		// std::cout << "middle: " << MTrackingWheel.controllerGet() << '\n';
		// std::cout << "x-pos: " << pos.x.convert(okapi::foot) << '\n';
		// std::cout << "y-pos: " << pos.y.convert(okapi::foot) << '\n';
		// std::cout << "theta: " << pos.theta.convert(okapi::degree) << ' ';
		if (isBlue()) {
			std::cout << "blue" << "\n";
		}
		if (isRed()) {
			std::cout << "red" << "\n";
		}
		
		// other debugging:
		// std::cout<< "imu: " << getHeading() << '\n';
		// std::cout << allMotors.getEfficiency() << "\n";
		// std::cout << "current: " << flywheel.getActualVelocity() << " target: " << targetSpeed * 2 - flywheel.getActualVelocity() << "\n";


		if (controller.getAnalog(okapi::ControllerAnalog::leftX) == -1 && controller.getAnalog(okapi::ControllerAnalog::rightX) == 1) {
					holdDrive = false;
					controller.setText(1, 0, "coast");
				}
		if (controller.getAnalog(okapi::ControllerAnalog::leftX) == 1 && controller.getAnalog(okapi::ControllerAnalog::rightX) == -1) {
			holdDrive = true;
			controller.setText(1, 0, "hold ");
		}

		// flywheel toggling
		if (controller[okapi::ControllerDigital::R1].changedToPressed()) {
			flywheelToggle = !flywheelToggle;
		}
		if (flywheelToggle) {
			// flywheel.controllerSet(0.8);
			if (flywheel.getActualVelocity() < targetSpeed-50) {
				flywheel.controllerSet(1);
			} else {
				flywheel.moveVelocity(targetSpeed);
			}
			// flywheel.moveVelocity(targetSpeed * 2 - flywheel.getActualVelocity()); // power correction, makes it target an extra high rpm if too low
			controller.setText(0, 0, "flywheel on ");
		} else {
			// flywheel.controllerSet(0);
			flywheel.moveVelocity(0);
			controller.setText(0, 0, "flywheel off");
		}

		// piston indexer
		if (controller[okapi::ControllerDigital::R2].isPressed()) {
			pros::c::adi_digital_write(kPneumaticIndexerPort, HIGH);
		} else {
			pros::c::adi_digital_write(kPneumaticIndexerPort, LOW);
		}

		// intake or roller (hold to use)
		if (controller[okapi::ControllerDigital::L1].isPressed()) {
			intake.controllerSet(1);
		} else if (controller[okapi::ControllerDigital::R2].isPressed()) {
			intake.controllerSet(-1);
		} else {
			intake.controllerSet(0);
		}

		// expansion piston
		if (controller[okapi::ControllerDigital::up].changedToPressed()) {
			expandToggle = !expandToggle;
		}

		if (expandToggle) {
			pros::c::adi_digital_write(kPneumaticExpansionPort, HIGH);
		} else {
			pros::c::adi_digital_write(kPneumaticExpansionPort, LOW);
		}

		// set power variables
		leftY = controller.getAnalog(okapi::ControllerAnalog::leftY);
		rightY = controller.getAnalog(okapi::ControllerAnalog::rightY);

		if (controller[okapi::ControllerDigital::L2].isPressed()) {
			allMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
			stepAutoAim();
		} else {
			if (holdDrive) {
				allMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
			} else {
				allMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
			}
			chassis->getModel()->tank(leftY, rightY);
		}
		// chassis->setState({chassis->getState().x, chassis->getState().y, getHeading(false) * okapi::degree});
		rate.delay(100_Hz);
	}
}
