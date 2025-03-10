#include "vex.h"
#include <iostream>
#include <string>
#include <sstream>
#include "position.h"
#include 'odomMath.h'
#pragma once
using namespace vex;

Odom::Odom (vex::rotation& leftSensor, vex::rotation& rightSensor, vex::rotation& rearSensor, double offsetLeft, double offsetRight, double offsetRear,double radiusWheel): 
leftWheel(leftSensor),
rightWheel(rightSensor),
backWheel(rearSensor), 
leftOffset(offsetLeft),
rightOffset(offsetRight),
backOffset(offsetRear),
wheelRadius(radiusWheel)
{
    threeWheel = true; 
    sensorLastLeft = leftWheel.position(turns)*2*pi*wheelRadius; 
    sensorLastRight = rightWheel.position(turns)*2*pi*wheelRadius; 
    sensorLastBack = backWheel.position(turns)*2*pi*wheelRadius; 
}

Odom::Odom (vex::rotation leftSensor, vex::rotation rightSensor, vex::inertial interialSensor, double offsetLeft, double offsetRight); leftWheel(leftSensor),
leftWheel(leftSensor)
rightWheel(rightSensor),
IMU(inertialSensor),
leftOffset(offsetLeft),
rightOffset(offsetRight),
wheelRadius(radiusWheel)
{
    threeWheel = false; 
    sensorLastLeft = leftWheel.position(turns)*2*pi*wheelRadius; 
    sensorLastRight = rightWheel.position(turns)*2*pi*wheelRadius; 
    sensorLastBack = backWheel.position(turns)*2*pi*wheelRadius; 
}

