#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "main.h"

#include "pros/rtos.h"



lemlib::Drivetrain drivetrain(&Left,
                              &Right,
                              11.5, //change
                              lemlib::Omniwheel::NEW_325, //change
                              450,
                              20 // horizontal drift, we can go fast on corners now
);

//11.5/2 -4.45

lemlib::TrackingWheel xPod(&horizontalencoder, lemlib::Omniwheel::NEW_2, 1.83); // tune this value please
lemlib::TrackingWheel yPod(&verticalencoder, lemlib::Omniwheel::NEW_275, -0.75);

lemlib::OdomSensors sensors(&yPod, nullptr, nullptr, nullptr, &Gyro);

lemlib::ControllerSettings lateral_controller(6, // proportional gain (kP)
                                              1, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(1.9,// proportional gain (kP)
                                              0.1, // integral gain (kI)
                                            19, // derivative gain (kD)
                                              20, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              4, // large error range, in degrees
                                              150, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);
