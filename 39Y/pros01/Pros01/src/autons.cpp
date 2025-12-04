#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "main.h"
#include "config.hpp"
#include "MyInclude/functions.hpp"
#include "pros/device.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"


using namespace pros;
// Global variables for PID
static double prevError = 0;
static double integral = 0;

int intakemode = 0;
int liftmode = 0;

void Skills() {

    Left.move(12000); 
    Right.move(12000);

        pros::delay(800);

        Left.move(0);
        Right.move(0);
    
  

}
void RedRightAuton(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-53.5,-9,295, false);
    ChangeIntakeState(2, 2);
    chassis.moveToPoint(-24, -23, 3000, {.forwards = false, .maxSpeed = 60},false);

    //chassis.moveToPoint(-12.5, -12.5, 1000, {.forwards = false},false); 
    chassis.turnToHeading(225, 2000, {}, false);

    //chassis.moveToPoint(-11, -10, 1000, {.forwards = false},false);
    //ChangeIntakeState(7, 1);
   // pros::delay(1000);
   // ChangeIntakeState(2, 1);

   // pros::delay(200);

    chassis.moveToPoint(-47, -48, 2000, {.forwards = true},false);

    doinker.set_value(true);
    ChangeIntakeState(2, 2);

    chassis.turnToHeading(90, 2000, {}, false);

    chassis.moveToPoint(-63.25, -46, 1000, {.forwards = false, .maxSpeed = 65},false);

    pros::delay(800);
    ChangeIntakeState(5, 2);
    //chassis.turnToHeading(90, 2000, {}, false);

    //pros::delay(2000);



    

    chassis.moveToPoint(-25, -46, 2000, {.forwards = true, .maxSpeed = 55},false);
     //high.move_voltage(-12000);
     //pros::delay(200);
    ChangeIntakeState(1, 2);

    pros::delay(2500);
 chassis.moveToPoint(-45, -46, 2000, {.forwards = false},false);
    chassis.turnToHeading(140, 2000, {}, false);
chassis.moveToPoint(-32, -60, 2000, {.forwards = true},false);
 chassis.turnToHeading(90, 2000, {}, false);

doinker2.set_value(true);
delay(200);

doinker2.set_value(false);
chassis.moveToPoint(-10, -60, 2000, {.forwards = true, .maxSpeed = 55},false);


doinker2.set_value(true);


}

void BlueRightAuton(){
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-53.5,-9,295, false);
    ChangeIntakeState(2, 1);
    chassis.moveToPoint(-24, -23, 3000, {.forwards = false, .maxSpeed = 60},false);

    //chassis.moveToPoint(-12.5, -12.5, 1000, {.forwards = false},false); 
    chassis.turnToHeading(225, 2000, {}, false);

    //chassis.moveToPoint(-11, -10, 1000, {.forwards = false},false);
    //ChangeIntakeState(7, 1);
   // pros::delay(1000);
   // ChangeIntakeState(2, 1);

   // pros::delay(200);

    chassis.moveToPoint(-47, -48, 2000, {.forwards = true},false);

    doinker.set_value(true);
    ChangeIntakeState(2, 1);

    chassis.turnToHeading(90, 2000, {}, false);

    chassis.moveToPoint(-63.25, -46, 1000, {.forwards = false, .maxSpeed = 55},false);

    pros::delay(800);
    ChangeIntakeState(5, 1);
    //chassis.turnToHeading(90, 2000, {}, false);

    //pros::delay(2000);



    
////
    chassis.moveToPoint(-25, -46, 2000, {.forwards = true,},false);
    // high.move_voltage(-12000);
    // pros::delay(200);
    //ChangeIntakeState(1, 1);

    pros::delay(2500);
 chassis.moveToPoint(-45, -46, 2000, {.forwards = false},false);
    chassis.turnToHeading(140, 2000, {}, false);
chassis.moveToPoint(-32, -60, 2000, {.forwards = true},false);
 chassis.turnToHeading(90, 2000, {}, false);

doinker2.set_value(true);
delay(200);

doinker2.set_value(false);
chassis.moveToPoint(-10, -60, 2000, {.forwards = true,},false);

//doinker2.set_value(true);



}






// Function to normalize angle to -180 to 180 degrees
double normalizeAngle(double angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

// Main turning function that can be called anywhere
void turnToAngle(double targetAngle) {
    // PID Constants - adjust these based on your robot
    const double kP = 1.7;
    const double kI = 0.015;
    const double kD = 19;
    
    // Tolerance settings
    const double angleTolerance = 5;  // Degrees
    const double settledTime = 250;     // Milliseconds
    double settledTimer = 0;
    
    // Reset integral and previous error
    integral = 0;
    prevError = 0;
    
    // Main control loop
    while (true) {
        // Get current angle and calculate error
        double currentAngle = Gyro.get_rotation();
        double error = normalizeAngle(targetAngle - currentAngle);
        
        // Check if within tolerance
        if (fabs(error) < angleTolerance) {
            settledTimer += 10;
            if (settledTimer >= settledTime) {
                break;  // Exit loop when settled
            }
        } else {
            settledTimer = 0;
        }
        
        // Calculate integral with anti-windup
        if (fabs(error) < 30) {
            integral += error;
        } else {
            integral = 0;
        }
        
        // Calculate derivative
        double derivative = error - prevError;
        prevError = error;
        
        // Calculate motor output
        double output = (kP * error) + (kI * integral) + (kD * derivative);
        
        // Limit output
        if (output > 90) output = 90;
        if (output < -90) output = -90;
        
        // Apply to motors
       Left.move(-output);
        Right.move(output);
        
        pros::delay(10);
        pros::lcd::print(4, "Output: %f", (float) output);
        pros::lcd::print(5, "Error: %f", (float) error);
    }
    
    // Stop motors
    Left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    Right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    Left.brake();
    Right.brake();
   
}

// Example usage in main program

void turnAuton(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(0,0,0, false);

    chassis.turnToHeading( 90, 2000, {}, false);
    pros::delay(350);

    chassis.turnToHeading( 180, 2000, {}, false);
    pros::delay(350);

    chassis.turnToHeading( 270, 2000, {}, false);
    pros::delay(350);

    chassis.turnToHeading( 0, 2000, {}, false);
    pros::delay(350);


}

void movetest(){

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(0,0,0, false);

    chassis.moveToPoint(0,36, 3000, {.forwards = true }, true);
    pros::delay(600);

    chassis.moveToPoint(0,0, 3000, {.forwards = false }, true);
    pros::delay(600);


}
void BlueNegQual(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    chassis.setPose(53,9.5,297, false);

    ChangeLiftState(4, 0);

    chassis.moveToPoint(54,9, 620, {.forwards = false }, true);

   
    delay(620);

    chassis.moveToPoint(24,22, 2000, {.forwards = true , .maxSpeed = 70 , .minSpeed = 40}, false);

    chassis.waitUntil(10);
    ChangeLiftState(5, 0);
    chassis.waitUntil(31);
    clamp.set_value(true);
    chassis.waitUntilDone();
    pros::delay(100);
  

    chassis.turnToHeading(115, 2000, {}, false);
    
    ChangeIntakeState(1, 1);

    chassis.moveToPoint(11.5,32,4000, {.forwards = false,.maxSpeed = 80, .minSpeed = 50},false);
   
    chassis.moveToPose(8,57, 180,4000, {.forwards = false,.maxSpeed = 70, .minSpeed = 36},false);

    chassis.swingToHeading(145, lemlib::DriveSide::LEFT, 2000, {}, false);
    chassis.moveToPoint(18,29 ,4000, {.forwards = true, .minSpeed = 75, },false);
    chassis.turnToHeading(180, 2000, {}, false);

    chassis.moveToPoint(24, 42,4000, {.forwards = false, .minSpeed = 75, },false);

    chassis.turnToHeading(270, 2000, {}, false);

    chassis.moveToPoint(51,50,4000, {.forwards = false,.maxSpeed = 85, .minSpeed = 60},false);

    ChangeLiftState(4, 0);

    chassis.turnToHeading(225, 2000, {}, false);

    pros::delay(200);

    chassis.moveToPoint(66,66, 800, {.forwards = false,.maxSpeed = 100, .minSpeed = 55},false);

    delay(200);
    Left.move(8000);
        Right.move(8000);

        pros::delay(420);

        Left.move(0);
        Right.move(0);

    pros::delay(400);


    Left.move(-8500);
    Right.move(-8500);

    pros::delay(300);

    Left.move(0);
    Right.move(0);

    pros::delay(300);


    Left.move(8500);
    Right.move(8500);

    pros::delay(200);

    Left.move(0);
    Right.move(0);

    chassis.turnToHeading(45, 2000, {}, false);

    chassis.moveToPoint(24,24, 800, {.forwards = false,.maxSpeed = 100, .minSpeed = 75},false);

    chassis.waitUntil(10);
    ChangeLiftState(5, 0);

     chassis.waitUntil(30);

     ChangeLiftState(3, 0);


    
    //chassis.moveToPoint(52, 48,3000, {.forwards = true,  .minSpeed = 70, },false);

  
}

void RedNegQual(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    chassis.setPose(-53,9.5,63, false);

    ChangeLiftState(4, 26);

    chassis.moveToPoint(-54,9, 620, {.forwards = false }, true);

   
    delay(620);

    chassis.moveToPoint(-24,22, 2000, {.forwards = true , .maxSpeed = 70 , .minSpeed = 40}, false);

    chassis.waitUntil(10);
    ChangeLiftState(0, 26);

    chassis.waitUntil(31);
    clamp.set_value(true);
    chassis.waitUntilDone();
    pros::delay(100);
  
    chassis.turnToHeading(245, 2000, {}, false);

    ChangeIntakeState(1, 2);

    chassis.moveToPoint(-11.5,32,4000, {.forwards = false,.maxSpeed = 80, .minSpeed = 50},false);

    chassis.moveToPose(-8,57, 180,4000, {.forwards = false,.maxSpeed = 70, .minSpeed = 36},false);

    
    chassis.swingToHeading(215, lemlib::DriveSide::RIGHT, 2000, {}, false);
    chassis.moveToPoint(-18,29 ,4000, {.forwards = true, .minSpeed = 75, },false);
    chassis.turnToHeading(180, 2000, {}, false);

 
    chassis.moveToPoint(-24, 42,4000, {.forwards = false, .minSpeed = 75, },false);

    chassis.turnToHeading(90, 2000, {}, false);

    chassis.moveToPoint(-49.5,50,4000, {.forwards = false,.maxSpeed = 85, .minSpeed = 60},false);

    chassis.turnToHeading(135, 2000, {}, false);

    pros::delay(200);

    chassis.moveToPoint(-66,66, 800, {.forwards = false,.maxSpeed = 100, .minSpeed = 55},false);

    delay(200);
    Left.move(8000);
    Right.move(8000);

        pros::delay(420);

        Left.move(0);
        Right.move(0);
    pros::delay(400);


    Left.move(-8500);
    Right.move(-8500);

    pros::delay(300);

    Left.move(0);
    Right.move(0);

    pros::delay(300);


    Left.move(8500);
    Right.move(8500);

    pros::delay(200);

    Left.move(0);
    Right.move(0);

    chassis.turnToHeading(315, 2000, {}, false);

    chassis.moveToPoint(-24,24, 800, {.forwards = false,.maxSpeed = 100, .minSpeed = 75},false);

    chassis.waitUntil(10);
    ChangeLiftState(5, 0);

     chassis.waitUntil(30);

     ChangeLiftState(3, 0);
}

void BlueRing_Alliance(){

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    chassis.setPose(53,9.5,297, false);

    ChangeLiftState(4, 0);

    chassis.moveToPoint(54,9, 620, {.forwards = false }, true);

   
    delay(620);

    chassis.moveToPoint(24,22, 2000, {.forwards = true , .maxSpeed = 70 , .minSpeed = 40}, false);

    chassis.waitUntil(10);
    ChangeLiftState(5, 0);
    chassis.waitUntil(31);
    clamp.set_value(true);
    chassis.waitUntilDone();
    pros::delay(100);
  

    chassis.turnToHeading(115, 2000, {}, false);
    
    ChangeIntakeState(1, 1);

    chassis.moveToPoint(11.5,32,4000, {.forwards = false,.maxSpeed = 80, .minSpeed = 50},false);
   
    chassis.moveToPose(8,57, 180,4000, {.forwards = false,.maxSpeed = 70, .minSpeed = 36},false);

    chassis.swingToHeading(145, lemlib::DriveSide::LEFT, 2000, {}, false);
    chassis.moveToPoint(18,29 ,4000, {.forwards = true, .minSpeed = 75, },false);
    chassis.turnToHeading(180, 2000, {}, false);

    chassis.moveToPoint(24, 42,4000, {.forwards = false, .minSpeed = 75, },false);

    chassis.turnToHeading(270, 2000, {}, false);

    chassis.moveToPoint(51,50,4000, {.forwards = false,.maxSpeed = 85, .minSpeed = 60},false);

    ChangeLiftState(4, 0);

    chassis.turnToHeading(225, 2000, {}, false);

    pros::delay(200);

    chassis.moveToPoint(66,66, 800, {.forwards = false,.maxSpeed = 100, .minSpeed = 55},false);

    delay(200);
    Left.move(8000);
        Right.move(8000);

        pros::delay(420);

        Left.move(0);
        Right.move(0);

    pros::delay(400);


    Left.move(-8500);
    Right.move(-8500);

    pros::delay(300);

    Left.move(0);
    Right.move(0);

    pros::delay(300);


    Left.move(8500);
    Right.move(8500);

    pros::delay(200);

    Left.move(0);
    Right.move(0);

    //chassis.moveToPoint(52, 48,3000, {.forwards = true,  .minSpeed = 70, },false);

    chassis.turnToHeading(0, 2000, {}, false);
    intakelift.set_value(true);

    chassis.moveToPoint(52, 10,3000, {.forwards = false,  .minSpeed = 90, },false);


    intakelift.set_value(false);

    chassis.moveToPoint(52, 20,3000, {.forwards = true,  .minSpeed = 50, },false);

}

void RedRing_Alliance(){

    
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    chassis.setPose(-53,9.5,63, false);

    ChangeLiftState(4, 26);

    chassis.moveToPoint(-54,9, 620, {.forwards = false }, true);

   
    delay(620);

  
   

 // chassis.turnToHeading(270, 2000, {}, false);

   // chassis.moveToPoint(35,17, 2000, {.forwards = true , .minSpeed = 90}, false);
    chassis.moveToPoint(-24,22, 2000, {.forwards = true , .maxSpeed = 70 , .minSpeed = 40}, false);

    chassis.waitUntil(10);
    ChangeLiftState(0, 26);
    
    //chassis.waitUntil(10);
    

    chassis.waitUntil(31);
    clamp.set_value(true);
    chassis.waitUntilDone();
    pros::delay(100);
  

    chassis.turnToHeading(245, 2000, {}, false);

    


    ChangeIntakeState(1, 2);

    chassis.moveToPoint(-11.5,32,4000, {.forwards = false,.maxSpeed = 80, .minSpeed = 50},false);

    
   
    chassis.moveToPose(-8,57, 180,4000, {.forwards = false,.maxSpeed = 70, .minSpeed = 36},false);

    
    chassis.swingToHeading(215, lemlib::DriveSide::RIGHT, 2000, {}, false);
    chassis.moveToPoint(-18,29 ,4000, {.forwards = true, .minSpeed = 75, },false);
    chassis.turnToHeading(180, 2000, {}, false);

 
    chassis.moveToPoint(-24, 42,4000, {.forwards = false, .minSpeed = 75, },false);

    chassis.turnToHeading(90, 2000, {}, false);

    chassis.moveToPoint(-49.5,50,4000, {.forwards = false,.maxSpeed = 85, .minSpeed = 60},false);

    chassis.turnToHeading(135, 2000, {}, false);

    pros::delay(200);
    

  

    chassis.moveToPoint(-66,66, 800, {.forwards = false,.maxSpeed = 100, .minSpeed = 55},false);

    delay(200);
    Left.move(8000);
        Right.move(8000);

        pros::delay(420);

        Left.move(0);
        Right.move(0);

    



    //chassis.moveToPoint(50,50, 4000, {.forwards = true,.maxSpeed = 70, .minSpeed = 55},false);

    //chassis.turnToHeading(225, 2000, {}, false);

   

    pros::delay(400);


    Left.move(-8500);
    Right.move(-8500);

    pros::delay(300);

    Left.move(0);
    Right.move(0);

    pros::delay(300);


    Left.move(8500);
    Right.move(8500);

    pros::delay(200);

    Left.move(0);
    Right.move(0);

    //chassis.moveToPoint(52, 48,3000, {.forwards = true,  .minSpeed = 70, },false);

    chassis.turnToHeading(0, 2000, {}, false);
    intakelift.set_value(true);

    chassis.moveToPoint(-52, 15,3000, {.forwards = false,  .minSpeed = 90, },false);

  
   


    intakelift.set_value(false);




}

void BluePos6Ring(){

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    
    chassis.setPose(53,-9.5,241, false);

    chassis.moveToPoint(25.5, -23.5,2000, {.forwards = true, .maxSpeed = 85, .minSpeed = 65, },false);

    chassis.waitUntil(31);

    clamp.set_value(true);

    ChangeIntakeState(1, 1);

    chassis.waitUntilDone();

    delay(370);

    ChangeIntakeState(0, 1);

    chassis.turnToHeading(135, 2000, {}, false);
   // intakelift.set_value(true);

    chassis.moveToPoint(10.75, -10.5,4000, {.forwards = false},false);

    chassis.turnToHeading(161, 2000, {}, false);

    doinker.set_value(true);

    pros::delay(300);
    chassis.turnToHeading(110,2000, {}, false);

    chassis.moveToPoint(28, -16,2000, {.forwards = true,  .minSpeed = 45, },false);
    intakelift.set_value(true);


    chassis.turnToHeading(35, 2000, {}, false);
    doinker.set_value(false);



    chassis.turnToHeading(0, 2000, {}, false);

    intakelift.set_value(false);

    
    
    ChangeIntakeState(1, 1);

    chassis.moveToPoint(33, -29, 2000, {.forwards = false , .maxSpeed = 60},false);

    //chassis.turnToHeading(15, 2000, {}, false);


    chassis.moveToPoint(24, -48, 2000, {.forwards = false , .maxSpeed = 50, .minSpeed = 30, },false);

    chassis.turnToHeading(270, 2000, {}, false);

    chassis.moveToPoint(45, -53, 2000, {.forwards = false , .minSpeed = 65},false);


    chassis.turnToHeading(315, 2000, {}, false);

    ChangeLiftState(4, 0);


    chassis.moveToPoint(65,-66, 800, {.forwards = false,.maxSpeed = 100, .minSpeed = 55},false);

    

    delay(200);
    Left.move(11000);
    Right.move(11000);

    pros::delay(350);

    Left.move(0);
    Right.move(0);


    pros::delay(350);


    Left.move(-8500);
    Right.move(-8500);

    pros::delay(300);

    Left.move(0);
    Right.move(0);

    pros::delay(300);


    Left.move(8500);
    Right.move(8500);

    pros::delay(450);

    Left.move(0);
    Right.move(0);

    
    
}

void RedPos6Ring(){

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    
    chassis.setPose(-53,-9.5,119, false);

   // ChangeLiftState(0, 0);

    chassis.moveToPoint(-25.5, -23.5,2000, {.forwards = true, .maxSpeed = 100, .minSpeed = 65, },false);

    chassis.waitUntil(31);

    clamp.set_value(true);

    

    ChangeIntakeState(1, 2);

    chassis.waitUntilDone();

    delay(370);

    ChangeIntakeState(0, 2);

    chassis.turnToHeading(225, 2000, {}, false);
    //intakelift.set_value(true);

    //ChangeIntakeState(1, 1);

    chassis.moveToPoint(-10.75, -10,4000, {.forwards = false},false);
    intakelift.set_value(true);

    chassis.turnToHeading(199, 2000, {}, false);

   // pros::delay(150);

    doinker2.set_value(true);

    pros::delay(300);
    chassis.turnToHeading(250,2000, {}, false);

    chassis.moveToPoint(-28, -16,2000, {.forwards = true,  .minSpeed = 45, },false);


    chassis.turnToHeading(325, 2000, {}, false);
    doinker2.set_value(false);
    intakelift.set_value(false);

    //intakelift.set_value(false);

    //pros::delay(150);  

   

    chassis.turnToHeading(0, 2000, {}, false);

    
    
    ChangeIntakeState(1, 2);

    chassis.moveToPoint(-27, -30, 2000, {.forwards = false , .maxSpeed = 60},false);

   // chassis.turnToHeading(-15, 2000, {}, false);


    chassis.moveToPoint(-24, -48, 2000, {.forwards = false , .maxSpeed = 50, .minSpeed = 30, },false);

    chassis.turnToHeading(90, 2000, {}, false);

    chassis.moveToPoint(-45, -53, 2000, {.forwards = false , .minSpeed = 65,},false);


    chassis.turnToHeading(45, 2000, {}, false);

   // ChangeLiftState(4, 0);


    chassis.moveToPoint(-63.5,-66, 800, {.forwards = false,.maxSpeed = 100, .minSpeed = 60},false);

    delay(200);
    Left.move(11000);
    Right.move(11000);

    pros::delay(350);

    Left.move(0);
    Right.move(0);

    pros::delay(350);
    Left.move(-8500);
    Right.move(-8500);

    pros::delay(300);

    Left.move(0);
    Right.move(0);

    pros::delay(300);


    Left.move(8500);
    Right.move(8500);

    pros::delay(500);

    Left.move(0);
    Right.move(0);

   
    

}

void BluePosDisrupt(){

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(50,-32,60, false);
    ChangeIntakeState(3, 1);
    chassis.moveToPoint(24, -48,2000, {.forwards = false,  .minSpeed = 85, },false);


    chassis.turnToHeading(90, 2000, {}, false);

    ChangeLiftState(4, 26);

    chassis.moveToPoint(12, -48,2000, {.forwards = false,  .minSpeed = 85, },false);

    delay(200);

    chassis.moveToPoint(8, -48,2000, {.forwards = false,  .minSpeed = 85, },false);



    chassis.turnToHeading(180, 2000, {}, false);
    

    


}

void BlueRingSide(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(52,24,270, false);
    chassis.moveToPose(22,24, 270, 4000, {.forwards = true, .minSpeed = 50 },true);
    chassis.waitUntil(28);
    clamp.set_value(true);
    chassis.waitUntilDone();

    chassis.turnToHeading(130, 2000, {}, false);

    ChangeIntakeState(1, 1);
    
    chassis.moveToPose(8, 40, 130,4000, {.forwards = false, .horizontalDrift = 8, .lead = 0.6, .maxSpeed = 80, .minSpeed = 40 , },false);
    chassis.moveToPose(8,60, 180,4000, {.forwards = false, .horizontalDrift = 8, .lead = 0.6, .maxSpeed = 80, .minSpeed = 40},false);

    chassis.turnToHeading(165, 2000, {}, false);
    chassis.moveToPose(12, 32, 165,4000, {.forwards = true,.horizontalDrift = 8, .lead = 0.6,  .minSpeed = 65, },false);
    chassis.turnToHeading(225, 2000, {}, false);
    chassis.moveToPose(29, 53, 225,4000, {.forwards = false, .horizontalDrift = 8, .lead = 0.6, .minSpeed = 70, },false);
    chassis.moveToPose(24, 48, 225,4000, {.forwards = true, .horizontalDrift = 8, .lead = 0.6, .minSpeed = 65, },false);
    chassis.turnToHeading(270, 2000, {}, false);
    chassis.moveToPose(48, 48, 270,4000, {.forwards = false, .horizontalDrift = 8, .lead = 0.6, .minSpeed = 65, },false);
    chassis.turnToHeading(225, 2000, {}, false);
    chassis.moveToPose(64, 64, 225,3000, {.forwards = false, .horizontalDrift = 8, .lead = 0.6, .minSpeed = 85, },false);
    chassis.moveToPose(56.5, 56.5, 225,3000, {.forwards = true, .horizontalDrift = 8, .lead = 0.6, .minSpeed = 65, },false);
    pros::delay(300);

    intakelift.set_value(true);
    pros::delay(200);
    chassis.moveToPose(64, 64, 225,3000, {.forwards = false, .horizontalDrift = 8, .lead = 0.6, .minSpeed = 85, },false);
    intakelift.set_value(false);
    chassis.moveToPose(56.5, 56.5, 225,3000, {.forwards = true,.horizontalDrift = 8, .lead = 0.6,  .minSpeed = 65, },false);

    

 

   


}