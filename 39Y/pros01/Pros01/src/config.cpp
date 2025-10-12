#ifndef CONFIG_HPP
#define CONFIG_HPP

#include "config.hpp"
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "main.h"
#include "pros/optical.hpp"

// Motor ports
pros::v5::MotorGroup Right({-11, 12, 13}, pros::MotorGearset::blue);
pros::v5::MotorGroup Left({14, -15, -16}, pros::MotorGearset::blue);

pros::v5::Motor Lift(0);
pros::v5::Motor Lift2(0);
pros::v5::Motor roller(0);
pros::v5::Motor hooks(0);
// Sensor ports
//pros::v5::Rotation liftrotation(7);
pros::v5::Imu Gyro(18);
pros::v5::Distance Dis(7);
pros::v5::Optical color(2);
pros::v5::Rotation horizontalencoder(-9);// Yet to declare
pros::v5::Rotation verticalencoder(-8);// Yet to declare

pros::v5::Motor low(-17);
pros::v5::Motor mid(-18);
pros::v5::Motor high(-19);
pros::v5::Motor top(20);
// Pneumatic ports
pros::adi::DigitalOut doinker2('A');
pros::adi::DigitalOut doinker('B');
pros::adi::DigitalOut intakelift('D');
pros::adi::DigitalOut clamp('C');

pros::v5::Controller master(pros::E_CONTROLLER_MASTER);

pros::v5::Optical ColorSensor(5);
//  Controller
// pros::Controller master;
// lemlib::ControllerSettings lateral_controller;
// lemlib::ControllerSettings augular_controller;
// lemlib::Chassis chassis;
// Button 

#endif
