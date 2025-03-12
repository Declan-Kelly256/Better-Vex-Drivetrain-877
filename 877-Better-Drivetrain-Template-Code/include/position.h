#include "vex.h"
#include <iostream>
#include <string>
#include <sstream>
#pragma once 


/*
    This is the position object, it stores three datapoints, an x postion, a y position, and a heading variable theta, which for the purposes of odom is stored in standard, but it really can be anything. it's just a storage object
*/

class Position {

    protected: 
        double x ;
        double y ;
        double theta; 
        const float pi = 3.14159;
    public:
        Position();    
        Position(double xVal, double yVal);
        Position(double xVal, double yVal,double dir);

    double getX();
    double getY();
    double getTheta();
    double setX(double newX);
    double setY(double newY);
    double setTheta(double newTheta);

    void setPosition(Position p);

};  