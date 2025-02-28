#include "vex.h"
#include <iostream>
#include <string>
#include <sstream>
#include "position.h"
#pragma once
using namespace vex;

class Odom {
    private: 
        Position pos;

        // we need to figure out trial and error what the sign of these positions mean either left of center of the bot is positive or right is 
        double offPara1 ;
        double offPara2 ;
        double offRear ;

        double const pi = 3.14159;
    public:
        Odom( double O1,double O2,double O3);
        Odom( double O1,double O2,double O3, Position p );
        Odom( double O1,double O2,double O3, double x, double y, double theta);
        
    private: 

        // this function uses the arc lengths to calculate the change in angle for both arcs

        //keep in mind that this is change in orientation in the context of the global field. 
        double localTheta(double s1, double s2);
        //gets the chord of the first arc, the  y coord in the local rotated plane
        double localY (double s1, double dTheta);
        //gets the deviation arc chord, which is the x axis in our
        double localX (double sR, double dTheta);
        //this is just code to automatically calculate an angle in polar coodinates based off a cartesian x & y. accounts for arctan domain automatically
        double thetaOffset(double localX , double localY );
        //calculates the change in the x coord 
        double deltaX (double s1, double sR, double dTheta);
        //calculates the change in the y coord
        double deltaY (double s1, double sR, double dTheta);
        
        //finally we need one public function here that returns a position object which is the updated positon 
          // this function will be called very often during our control loops in order to have a higher, and more accurate sample rate 
    public:
        // puts everything together. 
        Position newPosition(double s1, double s2, double sR);
        //this is for when you only have two sensors, you need to mention if it is in degrees or not though
        Position newPosition(double s1 , double sR, double theta, bool degrees);

        //get functions for all relavent position data. 
        Position getPosition();
        double getX();
        double getY();
        double getTheta();
};