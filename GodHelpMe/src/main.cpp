#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups

//CHANGE PORTS LATER
pros::MotorGroup leftMotors({-5, 4, -3},
pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({6, -9, 7}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)
pros::MotorGroup intake({1,4});
pros::adi::DigitalOut clamp ('A');
// Inertial Sensor on port 10
pros::Imu imu(10);


// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 12.5, lemlib::Omniwheel::NEW_325,450, 8);

// lateral motion controller
lemlib::ControllerSettings linearController(10,0, 3, 3, 1, 100, 3, 500, 20 );

// angular motion controller
lemlib::ControllerSettings angularController(2, 0, 10,3, 1, 100,3, 500,0 );

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu );

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019 );

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019 );

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); //brain screen
    chassis.calibrate(); // calibrate sensors

    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

void intakeForwards(bool direction) {
	if (direction) {
		intake.move(100);
	} else {
		intake.move(-100);
	}
}
bool clampBool2;
void clampFunc (bool clampBool) {
	if (clampBool2) {
		clamp.set_value(false);
		clampBool2 = false;
	} else {
		clamp.set_value(true);
		clampBool2 = true;
	}
}
/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */

ASSET(path_jerryio_txt);
ASSET(path_jerryio2_txt);
void autonomous() {
	//wall stake
	intakeForwards(true);
    chassis.moveToPoint(-45,20, 5, {.forwards = false}, true);
	clamp.set_value(true);
	//rings
	chassis.follow(path_jerryio_txt, 10, 3000);
	chassis.moveToPoint(-60,60, 5000, {.forwards = false}, true);
	clamp.set_value(false);
	//go to other goal
	chassis.moveToPoint(-45,-20, 5, {.forwards = false}, true);
	clamp.set_value(true);
	chassis.follow(path_jerryio2_txt,10,200000);
	chassis.moveToPoint(-60, -60, 40000, {.forwards=false}, true);
	clamp.set_value(false);
	intake.brake();
	chassis.moveToPoint(50, -30, 450000, {.forwards=false},true);
	chassis.moveToPoint(60, -60, 4000, {.forwards = false}, true);
	clamp.set_value(false);
	chassis.moveToPoint(-50, 12, 500000, {.forwards = false}, true);
	clamp.set_value(true);
	chassis.moveToPoint(60, -60, 50000, {.forwards = false}, true);
	clamp.set_value(true);
}

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
		while (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			intakeForwards(true);
		} 
		while (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			intakeForwards(false);
		}
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
		while(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
			clampFunc(true);
		}
        // delay to save resources
        pros::delay(10);
    }
}
