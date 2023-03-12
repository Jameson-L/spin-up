#include "main.h"
#include "autonomous/odometry.hpp"
#include "subsystems/chassis.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/flywheel.hpp"
#include "subsystems/pneumatics.hpp"
#include "autonomous/autonomous.hpp"

#include <iostream>

int auton = 3;

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

void leftAuton() {
	auton = 0;
	pros::lcd::set_text(1, "Left");
}

void awpAuton() {
	auton = 1;
	pros::lcd::set_text(1, "AWP");
}

void rightAuton() {
	auton = 2;
	pros::lcd::set_text(1, "Right");
}

void initialize() {
	pros::lcd::initialize();
	// pros::lcd::set_text(1, "Among."); // sus
	pros::lcd::register_btn0_cb(leftAuton);
	pros::lcd::register_btn1_cb(awpAuton);
	pros::lcd::register_btn2_cb(rightAuton);

	// setting pin modes and starting positions
	// pros::c::adi_pin_mode(kPneumaticIndexerPort, OUTPUT);
	// pros::c::adi_pin_mode(kPneumaticExpansionPort, OUTPUT);
	// pros::c::adi_pin_mode(kPneumaticExpansionPort2, OUTPUT);
	// pros::c::adi_pin_mode(kPneumaticBlooperPort, OUTPUT);
	// // pros::c::adi_digital_write(kPneumaticIndexerPort, LOW);
	// pros::c::adi_digital_write(kPneumaticExpansionPort, LOW);
	// pros::c::adi_digital_write(kPneumaticExpansionPort2, LOW);
	// pros::c::adi_digital_write(kPneumaticBlooperPort, LOW);
	expansion.set_value(0);
	blooper.set_value(0);
	compression1.set_value(0);
	// compression2.set_value(0);
	compression2.set_value(1);
	// vision.set_signature(1, &blue);
	// vision.set_signature(2, &red);

	// for proper rpm control
	flywheel.setGearing(okapi::AbstractMotor::gearset::blue);
	intake.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);

	// pros::c::adi_pin_mode(3, INPUT); // auton selector
	// pros::ADIAnalogIn select = pros::ADIAnalogIn(3);
	// if (select.get_value() > 1380 && select.get_value() < 2450) {
	// 	rightAuton();
	// } else if (select.get_value() > 220 && select.get_value() < 1380) {
	// 	leftAuton();
	// } else if (select.get_value() > 2450 || select.get_value() < 220) {
	// 	awpAuton();
	// }
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	allMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::hold); // for tighter movements
	flywheel.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	// pros::c::adi_digital_write(kPneumaticIndexerPort, LOW); // default position
	// pros::c::adi_digital_write(kPneumaticExpansionPort, LOW); // default position
	// pros::c::adi_digital_write(kPneumaticExpansionPort2, LOW); // default position
	// pros::c::adi_pin_mode(kPneumaticBlooperPort, LOW);

	// if (auton == 0) {
	// 	left();
	// } else if (auton == 1) {
	// 	awp();
	// } else if (auton == 2){
	// 	right();
	// }
	skills();
	// std::cout << auton;
}

void opcontrol() {
	okapi::Controller controller;
	okapi::Timer timer; // in case of timer-based functions for match
	double init  = timer.millis().convert(okapi::second);
	okapi::Rate rate;

	continueFlywheel = false;
	double leftY; // left joystick y direction
	double rightY; // right joystick y direction
	// double rightX; // right joystick x direction
	bool flywheelToggle = false; // false = off
	bool expandToggle = false; // false = off
	double targetSpeed = 500; // target speed of flywheel - blue is 600 max
	bool holdDrive = false;
	bool hold = false; // if this is true, it means ur holding L2 and it should shoot one disc while blocking regular intake control. if you release L2, even if the thing hasnt finished moving, it works
	// chassisVisionPid.reset();
	// double error;
	// double prevError = 1;
	// double tbh = targetSpeed / 600.0; // maybe tune this, unlikely
	// double output = 0;
	bool blooperOn = true;
	bool compressionUp = false;

	// setting all motors to coast
	allMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	flywheel.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	// okapi::MedianFilter<40> filter; // 40 is good, higher number = less noise but slower

	// okapi::Motor LF = okapi::Motor(kDriveLFPort);
	// okapi::Motor LM = okapi::Motor(kDriveLMPort);
	// okapi::Motor LB = okapi::Motor(kDriveLBPort);
	// okapi::Motor RF = okapi::Motor(kDriveRFPort);
	// okapi::Motor RM = okapi::Motor(kDriveRMPort);
	// okapi::Motor RB = okapi::Motor(kDriveRBPort);

	// pros::ADIAnalogIn select = pros::ADIAnalogIn(3);

	// okapi::MedianFilter<10> filter;

	// main loop:
	while (true) {

		// printing odometry tests:
		// okapi::OdomState pos = chassis->getState();
		// pros::lcd::set_text(1, std::to_string(pos.x.convert(okapi::foot)));
		// pros::lcd::set_text(2, std::to_string(pos.y.convert(okapi::foot)));
		// pros::lcd::set_text(3, std::to_string(getHeading(false)));
		// pros::lcd::set_text(4, std::to_string(pos.theta.convert(okapi::degree)));

/*
		pros::lcd::set_text(1, std::to_string(LF.getPosition()));
		pros::lcd::set_text(2, std::to_string(LM.getPosition()));
		pros::lcd::set_text(3, std::to_string(LB.getPosition()));
		pros::lcd::set_text(4, std::to_string(RF.getPosition()));
		pros::lcd::set_text(5, std::to_string(RM.getPosition()));
		pros::lcd::set_text(6, std::to_string(RB.getPosition()));
		// */

		// std::cout << "left: " << LTrackingWheel.controllerGet() << '\n';
		// std::cout << "right: " << RTrackingWheel.controllerGet() << '\n';
		// std::cout << "middle: " << MTrackingWheel.controllerGet() << '\n';
		// std::cout << "x-pos: " << pos.x.convert(okapi::foot) << '\n';
		// std::cout << "y-pos: " << pos.y.convert(okapi::foot) << '\n';
		// std::cout << "theta: " << pos.theta.convert(okapi::degree) << ' ';
		// if (isBlue()) {
		// 	std::cout << "blue" << "\n";
		// }
		// if (isRed()) {
		// 	std::cout << "red" << "\n";
		// }
		//
		// vision.set_signature(1, &blue);
		// vision.set_signature(2, &red);
	  // std::cout << filter.filter(vision.get_by_size(0).x_middle_coord) << "\n";
		// other debugging:
		// std::cout<< "imu: " << getHeading() << '\n';
		// std::cout << allMotors.getEfficiency() << "\n";
		// std::cout << "current: " << flywheel.getActualVelocity() << " target: " << targetSpeed * 2 - flywheel.getActualVelocity() << "\n";
		// std::cout << select.get_value() << "\n";

		if (controller.getAnalog(okapi::ControllerAnalog::leftX) == -1 && controller.getAnalog(okapi::ControllerAnalog::rightX) == 1) {
			// holdDrive = false;
			allMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
			controller.setText(1, 0, "coast");
		}
		if (controller.getAnalog(okapi::ControllerAnalog::leftX) == 1 && controller.getAnalog(okapi::ControllerAnalog::rightX) == -1) {
			// holdDrive = true;
			allMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
			controller.setText(1, 0, "hold ");
		}

		// flywheel toggling
		if (controller[okapi::ControllerDigital::R1].changedToPressed()) {
			flywheelToggle = !flywheelToggle;
		}

		if (blooperOn) {
			targetSpeed = 500;
			blooper.set_value(1);
		} else {
			targetSpeed = 500;
			blooper.set_value(0);
		}

		if (flywheelToggle) {
			// // flywheel.controllerSet(0.8);
			// if (flywheel.getActualVelocity() < targetSpeed-30) {
			// 	flywheel.controllerSet(1);
			// } else {
			// 	flywheel.moveVelocity(targetSpeed);
			// }
			// // flywheel.moveVelocity(targetSpeed * 2 - flywheel.getActualVelocity()); // power correction, makes it target an extra high rpm if too low

			// error = targetSpeed - flywheel.getActualVelocity();
			// output += tbhGain * error;
			// if (signbit(error) != signbit(prevError)) {
			// 	output = 0.5 * (output + tbh);
			// 	tbh = output;
			// 	prevError = error;
			// }
			// if (flywheel.getActualVelocity() < targetSpeed - 30) {
	    //   flywheel.controllerSet(1);
	    // } else {
	    //   flywheel.controllerSet(output);
	    // }
			// std::cout << flywheel.getActualVelocity() << " " << output << "\n";

			if (flywheel.getActualVelocity() < targetSpeed - 100 || controller[okapi::ControllerDigital::R1].isPressed()) {
				flywheel.controllerSet(1);
			} else {
				flywheel.controllerSet(
					// -1 * (1 - targetSpeed / 600) * 0.01 * flywheel.getActualVelocity()
					// + (-1 * targetSpeed * targetSpeed + 700 * targetSpeed) / 60000.0
					targetSpeed / 600.0
				);
				// std::cout << flywheel.getActualVelocity() << " " << -1 * (1 - targetSpeed / 600) * 0.01 * flywheel.getActualVelocity()
				// + (-1 * targetSpeed * targetSpeed + 700 * targetSpeed) / 60000.0 << "\n";
			}

			controller.setText(0, 0, "flywheel on ");
		} else {
			if (controller[okapi::ControllerDigital::R1].isPressed()) {
				flywheel.controllerSet(1);
			} else {
				flywheel.controllerSet(0);
			}
			// tbh = targetSpeed / 600.0;
			// output = 0;
			// prevError = 1;
			// flywheel.moveVelocity(0);
			controller.setText(0, 0, "flywheel off");
		}

		// piston indexer
		// if (controller[okapi::ControllerDigital::R2].isPressed()) {
		// 	pros::c::adi_digital_write(kPneumaticIndexerPort, HIGH);
		// } else {
		// 	pros::c::adi_digital_write(kPneumaticIndexerPort, LOW);
		// }

		// intake or roller (hold to use)
		if (controller[okapi::ControllerDigital::L1].isPressed()) {
			hold = false;
			intake.controllerSet(1);
			compression1.set_value(0);
		} else if (controller[okapi::ControllerDigital::R2].isPressed()) {
			hold = false;
			intake.controllerSet(-1);
			if (!compressionUp && flywheel.getTargetVelocity() > 0) {
				compression1.set_value(1);
			}
		} else if (!hold) {
			intake.controllerSet(0);
			compression1.set_value(0);
		}

		if (controller[okapi::ControllerDigital::L2].changedToPressed()) {
			hold = true;
			intake.moveRelative(-240, 600);
			if (!compressionUp && flywheel.getTargetVelocity() > 0) {
				compression1.set_value(1);
			}
		}
		if (!controller[okapi::ControllerDigital::L2].isPressed()) {
			hold = false;
		}

		// blooper controls
		if (controller[okapi::ControllerDigital::X].changedToPressed()) {
			blooperOn = !blooperOn;
		}

		// expansion controls
		if (controller[okapi::ControllerDigital::up].changedToPressed()) {
			compressionUp = !compressionUp;
		}
		if (controller[okapi::ControllerDigital::down].changedToPressed()) {
			expandToggle = !expandToggle;
		}
		if (compressionUp) {
			compression1.set_value(0);
			compression2.set_value(1);
		} else {
			// compression1.set_value(1);
			compression2.set_value(0);
		}

		if (expandToggle) {
			expansion.set_value(1);
		} else {
			expansion.set_value(0);
		}

		// set power variables
		leftY = controller.getAnalog(okapi::ControllerAnalog::leftY);
		rightY = controller.getAnalog(okapi::ControllerAnalog::rightY);
		// rightX = controller.getAnalog(okapi::ControllerAnalog::rightX);

		// if (controller[okapi::ControllerDigital::A].isPressed()) {
		// 	allMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
		// 	stepAutoAim();
		// } else {
		// 	if (holdDrive) {
		// 		allMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
		// 	} else {
		// 		allMotors.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
		// 	}
		// 	chassis->getModel()->tank(leftY, rightY);
		// }


		// if (holdDrive || flywheelToggle || controller[okapi::ControllerDigital::R1].isPressed()) { // if holding or flywheel on
		//
		// } else {
		//
		// }
		chassis->getModel()->tank(leftY, rightY);
		// if (leftY >= 0) {
		// 	chassis->getModel()->arcade(leftY, rightX);
		// } else {
		// 	chassis->getModel()->arcade(leftY, -rightX);
		// }

		// chassis->setState({chassis->getState().x, chassis->getState().y, getHeading(false) * okapi::degree});
		rate.delay(100_Hz);
	}
}
