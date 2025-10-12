#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "config.hpp"
#include "MyInclude/functions.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "cmath"
#include "pros/motors.h"

using namespace pros;
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
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
	//pros::Task smartliftTask(smartlift);
	pros::Task pushback(pushback_intake);
	pros::lcd::initialize();
	chassis.calibrate();

	pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
	
			///pros::lcd::print(3, "intake ticks: %f", fmod(hooks.get_position(), 775));     
			
			// delay to save resources
            pros::delay(30);
        }
    });

	color.set_led_pwm(100);

	//pros::Task smartintakeTask(smartintake);

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
void competition_initialize() {}

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
BluePos6Ring();
}



bool clampValue = false;
bool intakeliftValue = false;
bool doinkerValue = false;
bool doinkerValue2 = false;


void opcontrol() {

	pros::Task driverTask(drive);

	while(true){
		
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
			clampValue = !clampValue;
		}

		// flipper
	//	if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
	//		intakeliftValue = !intakeliftValue;
	//}
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
			doinkerValue2 = !doinkerValue2;
		}
		// doinker
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
			doinkerValue = !doinkerValue;
		}
		clamp.set_value(clampValue);
		//intakelift.set_value(intakeliftValue);
		doinker.set_value(doinkerValue);
		doinker2.set_value(doinkerValue2);

		pros::delay(20);
	}
		//pros::lcd::print(0, "X: %f", (float) ColorSensor.get_hue());

	}// end of loop


