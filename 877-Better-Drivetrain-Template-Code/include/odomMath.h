#include "vex.h"
#include <iostream>
#include <string>
#include <sstream>
#include "position.h"
#pragma once
using namespace vex;

class Odom : public Position { // oh yea inheritance baby 
    private:
        //sensors and such
        vex::rotation leftWheel;
        vex::rotation rightWheel;
        vex::rotation backWheel; 

        vex::inertial IMU; 
        //offsets for sensors
        double leftOffset; 
        double rightOffset; 
        double backOffset;
        //other important Consts 
        double wheelRadius
        //whether or not it's three wheel odom or two wheel with an IMU 
        bool threeWheel; 
        // datapoints for storing shi 
        double sensorLastLeft; 
        double sensorLastRight; 
        double sensorLastBack; 

        
        
    public: 
        //constructor for threeWheel Odom
        Odom (vex::rotation leftSensor, vex::rotation rightSensor, vex::rotation rearSensor, double offsetLeft, double offsetRight, double offsetRear,double radiusWheel); 

        //Constructor for two wheel with an IMU 
        Odom (vex::rotation leftSensor, vex::rotation rightSensor, vex::inertial interialSensor, double offsetLeft, double offsetRight); 


        



};