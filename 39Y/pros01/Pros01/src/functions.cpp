#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <iostream>
#include <vector>

int IntakeState= 0;
int IntakeSort = 0;

int Liftmode = 0;
int reset = 0;



using namespace pros;

void ChangeIntakeState(int state, int sort ){
    IntakeState = state;//0 stop, 1 on, 2 reverse
    IntakeSort = sort; //`0` for none, `1` for red, `2` for blue

}
void ChangeLiftState(int state, int resetvalue){
    Liftmode = state;
    reset = resetvalue;

}
void drive(){
    while(true){
    int input_x = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int input_y = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    int right_output;
    int left_output;

    right_output = -input_x + input_y; //Cursed Arcade Drive Test, 上限更高，会很难开
    left_output = -input_x - input_y;

    if (right_output > 127) {
        right_output = 127;
    }  
    if (right_output < -127) {
        right_output = -127;
    }
    if (left_output > 127) {
        left_output = 127;
    }
    if (left_output < -127) {
        left_output = -127;
    }

    Left.move(-left_output);
    Right.move(-right_output);
    pros::delay(20);

    }

}

void drivetank(){
    while(true){
    int input_x = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int input_y = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y  );

    int right_output;
    int left_output;

    right_output = input_x; //Cursed Arcade Drive Test, 上限更高，会很难开
    left_output = input_y;

    if (right_output > 127) {
        right_output = 127;
    }  
    if (right_output < -127) {
        right_output = -127;
    }
    if (left_output > 127) {
        left_output = 127;
    }
    if (left_output < -127) {
        left_output = -127;
    }

    Left.move(-left_output);
    Right.move(-right_output);
    pros::delay(20);

    }

}


void smartintake(){

    bool waitTillRing = false;
    hooks.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    hooks.set_brake_mode(MotorBrake::brake);

    while(1){
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) || IntakeState == 1){
            
            float degree;

            double colorValue = color.get_hue();
            double proximity = color.get_proximity();


            bool red = (colorValue > 330 || colorValue < 30) && (proximity>120);
            bool blue = (colorValue > 180 && colorValue < 270) && (proximity>120);   

            
            degree = hooks.get_position();
            float output = fmod(degree, 387.08); //387.5 //387 too less //387.25 too much //387.20 too much//387.11 too much //

            hooks.move_voltage(12000);
            roller.move_voltage(12000);


            if (IntakeSort == 1){

            if(red){
                waitTillRing = true;
            }
            }
            else if (IntakeSort == 2){
            if(blue){
                waitTillRing = true;
            }
            }

            if(waitTillRing){
                if(output > 300 && output < 330){
                    hooks.move_voltage(0);
                    roller.move_voltage(0);
                    pros::delay(200);
                    hooks.move_voltage(12000);
                    roller.move_voltage(12000);
                    waitTillRing = false;
                }
            }

            if (hooks.get_actual_velocity() < 10 && competition::is_autonomous()){
                delay(200);
                if (hooks.get_actual_velocity() < 10){
                    hooks.move_voltage(-12000);
                    roller.move_voltage(-12000);
                    delay(300);

                }
            }

            

        }

        else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) || IntakeState == 2){
            hooks.move_voltage(-12000);
            roller.move_voltage(-12000);
        }
        else if (IntakeState == 3){
            roller.move_voltage(12000);
        }
        
        else{
            hooks.move_voltage(0);
            roller.move_voltage(0);
        }

        pros::delay(10);
    }
}
void smartlift(){
   
   
    float angle;
 
    float error;
    float liftPower;

    float kP = 150;

    float offset = 0;

    bool robot_in_auton = true;


    Lift.set_encoder_units(MotorEncoderUnits::degrees);
    Lift.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
    Lift2.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

    std::vector<float> targets = {0,34,155, 163, 180, -30};

    pros::delay(2000);

    while(1){
        angle = ((Lift.get_position())/3) - offset;

        robot_in_auton = competition::is_autonomous();

        

        if (!robot_in_auton){
            if(reset == 1){
            Lift.move_voltage(-12000);
            Lift2.move_voltage(-12000);
            pros::delay(1000);
            offset = ((Lift.get_position())/3);

            
            pros::delay(200);
    
            Lift.move_voltage(0);
            Lift2.move_voltage(0);
            reset = 0;

            Liftmode = 0;

            

            }
            
        }


		
        //angleNot = Lift.get_position();

    
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
            Liftmode = Liftmode!=1 ? 1 : 0;
        }
        if (master.get_digital(E_CONTROLLER_DIGITAL_R2) && !master.get_digital(E_CONTROLLER_DIGITAL_R1)){
            Liftmode = 2;
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
            Liftmode = 3;
        }

        error = targets[Liftmode] - angle;

        liftPower = kP*error;

        if(Liftmode == 0 || Liftmode == 1 || Liftmode == 3 || Liftmode == 4 || Liftmode == 5){
            Lift.move_voltage(liftPower);
            Lift2.move_voltage(liftPower);
        }
        else if(Liftmode == 2){
            if(master.get_digital(E_CONTROLLER_DIGITAL_R2)){
                Lift.move_voltage(12000);
                Lift2.move_voltage(12000);
            }
            else{
                Lift.brake();
                Lift2.brake();
            }
        }

        


        pros::delay(10);
    }
}

void pushback_intake(){

    bool waitTillRing = false;
    hooks.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    hooks.set_brake_mode(MotorBrake::brake);

    while(1){
        int colorflag = 0;
            if (color.get_hue() < 40) {
            colorflag = 1;
        } else if (color.get_hue() > 70) {
            colorflag = 2;
        }
        
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            
            top.move_voltage(12000);
            high.move_voltage(12000);
            mid.move_voltage(8000);
            low.move_voltage(12000);
        }

        else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            top.move_voltage(-12000);
            high.move_voltage(12000);
            mid.move_voltage(12000);
            low.move_voltage(-12000);
            
        }

        else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            top.move_voltage(12000);
            high.move_voltage(-12000);
            mid.move_voltage(- 12000);
            low.move_voltage(12000);
            
        }

        else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            top.move_voltage(5000);
            high.move_voltage(-12000);
            mid.move_voltage(12000);
            low.move_voltage(12000);
            
        }

      //sort 
        else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
            
        

        if (colorflag == 2){
            top.move_voltage(-12000);
            high.move_voltage(-12000);
            mid.move_voltage(8000);
            low.move_voltage(10000);
            pros::delay(250);
        }


            
            top.move_voltage(-12000);
            high.move_voltage(6000);
            mid.move_voltage(8000);
            low.move_voltage(10000);

            
        }

     
        else{
            top.move_voltage(0);
            high.move_voltage(0);
            mid.move_voltage(0);
            low.move_voltage(0);
            
        }

        pros::delay(5);
    }
}

/*
void smartintake(void* alliance_mode) {
    int colorflag{0};
    int mode = *(int*)alliance_mode;

    while (true)
    {
        if (color.get_hue() < 20) {
            colorflag = 1;
        } else if (color.get_hue() > 70) {
            colorflag = 2;
        }

        if (!master.get_digital(E_CONTROLLER_DIGITAL_L1) && !master.get_digital(E_CONTROLLER_DIGITAL_L2)) {
            roller.move(0);
        } else if (master.get_digital(E_CONTROLLER_DIGITAL_L1) && !master.get_digital(E_CONTROLLER_DIGITAL_L2)) {
            if (colorflag == mode) {
                roller.move(-127);
            } else {
                roller.move(127);
            }
        } else if (!master.get_digital(E_CONTROLLER_DIGITAL_L1) && master.get_digital(E_CONTROLLER_DIGITAL_L2)) {
            roller.move(127);
        } else if (master.get_digital(E_CONTROLLER_DIGITAL_L1) && master.get_digital(E_CONTROLLER_DIGITAL_L2)) {
            roller.brake();
        }

        delay(20);
    }
}
*/
