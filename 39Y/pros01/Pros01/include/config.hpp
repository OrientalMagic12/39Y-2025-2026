#ifndef CONFIG_HPP
#define CONFIG_HPP

#include "api.h"
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "pros/misc.hpp"

// Motor ports
extern pros::v5::MotorGroup Left;
extern pros::v5::MotorGroup Right;

extern pros::v5::Motor Lift;
extern pros::v5::Motor Lift2;
extern pros::v5::Motor roller;
extern pros::v5::Motor hooks;

extern pros::v5::Motor low;
extern pros::v5::Motor mid;
extern pros::v5::Motor high;
extern pros::v5::Motor top;

// Sensor ports
extern pros::v5::Rotation liftrotation;
extern pros::v5::Imu Gyro;
extern pros::v5::Distance Dis;
extern pros::v5::Optical ColorSensor;
extern pros::v5::Optical color;

extern pros::v5::Rotation horizontalencoder; // Yet to declare
extern pros::v5::Rotation verticalencoder; // Yet to declare
extern pros::v5::Controller master;

// Pneumatic ports
extern pros::adi::DigitalOut doinker2;
extern pros::adi::DigitalOut doinker;
extern pros::adi::DigitalOut intakelift;
extern pros::adi::DigitalOut clamp;





// Button

#endif
