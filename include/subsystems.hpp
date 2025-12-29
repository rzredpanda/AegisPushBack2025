#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples


inline pros::Motor lever(11);
inline pros::Motor intake(20);
inline ez:Piston wings('d', false);
inline ez:Piston park('b', false);
inline ez:Piston matchloaders('c', false);
// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');


//pros::Motor intake(20);
//pros::Motor lever(11);
//pros::Motor hood(10);
//pros::MotorGroup long_goal({19,-18,-10});
//pros::adi::Pneumatics matchloader('H',);
//pros::adi::DigitalOut park('D');
//pros::adi::Pneumatics matchloaders('c', false);
//pros::adi::Pneumatics park('b', false);
//pros::adi::Pneumatics wings('d', false);