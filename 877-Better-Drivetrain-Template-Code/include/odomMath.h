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
        double wheelRadius;
        //whether or not it's three wheel odom or two wheel with an IMU 
        bool threeWheel; 
        // datapoints for storing shi 
        double sensorLastLeft; 
        double sensorLastRight; 
        double sensorLastBack; 

        double sensorLastIMU; 

        double pi = 3.1459;

        
        
    public: 
        //constructor for threeWheel Odom
        Odom (vex::rotation& leftSensor, vex::rotation& rightSensor, vex::rotation& rearSensor, double offsetLeft, double offsetRight, double offsetRear,double radiusWheel); 

        //Constructor for two wheel with an IMU 
        Odom (vex::rotation& leftSensor, vex::rotation& rightSensor, vex::inertial& inertialSensor, double offsetLeft, double offsetRight, double radiusWheel); 


        double calculateDeltaTheta(double deltaLeft, double deltaRight); // returns the angle of the arc of the movement in radians (for threeWheel) 

        double calculateY(double deltaRight,double deltaTheta); // returns the forward arc Component of travel
        
        double calculateX(double deltaBack,double deltaTheta); // returns the perpendicular component of travel

        Position updatePosition(); // returns the new Position of the robot

        Position updatePositionThreeWheel();// for three wheel mode
        Position updatePositionIMU(); // for IMU mode

        Position coordinateRotator( Position p, double rotater)



};